# process_controller.py

from typing import Optional, List, Tuple, Dict, Any
from PyQt6.QtCore import QObject, QTimer, pyqtSignal as Signal, pyqtSlot as Slot
from dataclasses import dataclass
from enum import Enum

# --- Enum 및 Dataclass 정의 ---
class ActionType(str, Enum):
    IG_CMD = "IG_CMD"
    RGA_SCAN = "RGA_SCAN"
    MFC_CMD = "MFC_CMD"
    FADUINO_CMD = "FADUINO_CMD"
    DELAY = "DELAY"
    DC_POWER_SET = "DC_POWER_SET"
    RF_POWER_SET = "RF_POWER_SET"
    DC_POWER_STOP = "DC_POWER_STOP"
    RF_POWER_STOP = "RF_POWER_STOP"
    OES_RUN = "OES_RUN"

@dataclass
class ProcessStep:
    action: ActionType
    message: str
    value: Optional[float] = None
    params: Optional[Tuple] = None
    duration: Optional[int] = None
    parallel: bool = False
    polling: bool = False
    no_wait: bool = False  # 확인응답 없이 즉시 다음 스텝 진행

    def __post_init__(self):
        if self.action == ActionType.DELAY:
            if self.duration is None:
                raise ValueError("DELAY 액션은 duration이 필요합니다.")
            if self.parallel:
                raise ValueError("DELAY는 병렬 블록에 포함할 수 없습니다.")
        if self.action in (ActionType.DC_POWER_SET, ActionType.RF_POWER_SET, ActionType.IG_CMD):
            if self.value is None:
                raise ValueError(f"{self.action.name} 액션은 value가 필요합니다.")
        if self.action == ActionType.FADUINO_CMD:
            if not self.params or len(self.params) != 2:
                raise ValueError("FADUINO_CMD params는 (cmd:str, arg:any) 형태여야 합니다.")
        if self.action == ActionType.MFC_CMD:
            if not self.params or len(self.params) != 2 or not isinstance(self.params[1], dict):
                raise ValueError("MFC_CMD params는 (cmd:str, args:dict) 형태여야 합니다.")
        if self.action == ActionType.OES_RUN:
            if not self.params or len(self.params) != 2:
                raise ValueError("OES_RUN params는 (process_time:float, integration_ms:int) 형태여야 합니다.")

class ParallelExecution:
    def __init__(self, steps: List[ProcessStep], end_index: int):
        self.steps = steps
        self.completed = 0
        self.end_index = end_index
        self.total = len(steps)

    def mark_completed(self) -> bool:
        self.completed += 1
        return self.completed >= self.total

    @property
    def progress(self) -> float:
        return self.completed / self.total if self.total > 0 else 1.0

class ProcessController(QObject):
    # 1) 장치로 보낼 신호
    update_faduino_port = Signal(str, bool)
    mfc_command_requested = Signal(str, dict)
    oes_command_requested = Signal(float, int)
    dc_power_command_requested = Signal(float)
    dc_power_stop_requested = Signal()
    rf_power_command_requested = Signal(float)
    rf_power_stop_requested = Signal()
    ig_command_requested = Signal(float)
    rga_external_scan_requested = Signal()

    # 2) UI/상태
    log_message = Signal(str, str)
    process_status_changed = Signal(bool)
    process_started = Signal(dict)
    process_finished = Signal(bool)
    update_process_state = Signal(str)

    # 3) 폴링 제어
    set_polling = Signal(bool)

    # 4) 비상정지 브로드캐스트
    process_aborted = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.process_sequence: List[ProcessStep] = []
        self._current_step_idx = -1
        self.is_running = False
        self.current_params: Dict[str, Any] = {}
        self._aborting = False
        self._accept_completions = True
        self._in_emergency = False
        self._countdown_active = False

        # 스텝 타이머
        self.step_timer = QTimer(self)
        self.step_timer.setSingleShot(True)
        self.step_timer.timeout.connect(self.on_step_completed)

        # 병렬 실행 관리자
        self._px: Optional[ParallelExecution] = None

        # 카운트다운
        self._countdown_timer = QTimer(self)
        self._countdown_timer.setInterval(1000)
        self._countdown_timer.timeout.connect(self._on_countdown_tick)
        self._countdown_remaining_ms = 0
        self._countdown_base_message = ""

        # MFC 확인 화이트리스트(폴링으로 오는 READ_*는 스텝완료로 취급 X)
        # self._mfc_ok_cmds = {
        #     "FLOW_SET", "FLOW_ON", "FLOW_OFF",
        #     "SP1_SET", "SP1_ON", "SP4_ON",
        #     "MFC_ZEROING", "PS_ZEROING",
        #     "VALVE_OPEN", "VALVE_CLOSE",
        #     "FLOW_ONOFF_BATCH",
        # }

    # ---------------- 시퀀스 구성 ----------------
    def _create_process_sequence(self, params: Dict[str, Any]) -> List[ProcessStep]:
        common_info = self._get_common_process_info(params)
        use_dc, use_rf, use_ms = common_info['use_dc'], common_info['use_rf'], common_info['use_ms']
        gas_info, gun_shutters = common_info['gas_info'], common_info['gun_shutters']

        base_pressure = float(params.get("base_pressure", 1e-5))
        working_pressure = float(params.get("working_pressure", 0))
        process_time_min = float(params.get("process_time", 0))
        shutter_delay_min = float(params.get("shutter_delay", 0))
        shutter_delay_sec = shutter_delay_min * 60.0
        process_time_sec = process_time_min * 60.0
        dc_power = float(params.get("dc_power", 0))
        rf_power = float(params.get("rf_power", 0))
        integration_ms = int(params.get("integration_time", 60))

        steps: List[ProcessStep] = []

        # --- 초기화 ---
        self._add_initialization_steps(steps, base_pressure, gas_info)

        # --- 가스 주입 ---
        self._add_gas_injection_steps(steps, params, gas_info)

        # --- 압력 제어 시작 ---
        self._add_pressure_control_steps(steps, working_pressure)

        # --- 파워/셔터 ---
        self._add_power_and_shutter_steps(
            steps, params, gun_shutters, use_dc, use_rf, use_ms,
            dc_power, rf_power, shutter_delay_sec, shutter_delay_min
        )

        # --- 메인 공정 시간 ---
        if process_time_sec > 0:
            steps.append(ProcessStep(
                action=ActionType.OES_RUN,
                params=(process_time_sec, integration_ms),
                message=f'OES 측정 시작 ({process_time_min}분, {integration_ms}ms)',
                no_wait=True  # 백그라운드로 돌리고 곧바로 다음 스텝(딜레이)로
            ))

            steps.append(ProcessStep(
                action=ActionType.DELAY,
                duration=int(process_time_sec * 1000),
                message=f'메인 공정 진행 ({process_time_min}분)',
                polling=True  # 메인 공정 중 데이터 폴링
            ))

        # --- 종료 시퀀스 ---
        steps.extend(self._create_shutdown_sequence(params))

        return steps

    def _add_initialization_steps(self, steps: List[ProcessStep], base_pressure: float, gas_info: Dict):
        self.log_message.emit("Process", "전체 공정 모드로 시작 (Base Pressure 대기 포함).")

        steps.extend([
            ProcessStep(
                action=ActionType.IG_CMD,
                value=base_pressure,
                message=f'베이스 압력({base_pressure:.1e}) 도달 대기'
            ),
            # 외부 RGA 스캔 트리거 → 별도 완료 신호가 없으므로 no_wait
            # ProcessStep(
            #     action=ActionType.RGA_SCAN,
            #     message='RGA 측정 시작',
            # )
        ])

        # 모든 채널 Flow OFF
        for gas, info in gas_info.items():
            steps.append(ProcessStep(
                action=ActionType.MFC_CMD,
                params=('FLOW_OFF', {'channel': info["channel"]}),
                message=f'Ch{info["channel"]}({gas}) Flow Off'
            ))

        steps.extend([
            ProcessStep(action=ActionType.MFC_CMD, params=('VALVE_OPEN', {}), message='MFC Valve Open'),
            ProcessStep(action=ActionType.MFC_CMD, params=('PS_ZEROING', {}), message='압력 센서 Zeroing')
        ])

        for gas, info in gas_info.items():
            steps.append(ProcessStep(
                action=ActionType.MFC_CMD,
                params=('MFC_ZEROING', {'channel': info["channel"]}),
                message=f'Ch{info["channel"]}({gas}) Zeroing'
            ))

    def _add_gas_injection_steps(self, steps: List[ProcessStep], params: Dict[str, Any], gas_info: Dict):
        steps.append(ProcessStep(
            action=ActionType.FADUINO_CMD,
            params=('MV', True),
            message='메인 밸브 열기'
        ))

        for gas, info in gas_info.items():
            if params.get(f"use_{gas.lower()}", False):
                flow_value = float(params.get(f"{gas.lower()}_flow", 0))
                steps.extend([
                    ProcessStep(
                        action=ActionType.FADUINO_CMD,
                        params=(gas, True),
                        message=f'{gas} 밸브 열기'
                    ),
                    ProcessStep(
                        action=ActionType.MFC_CMD,
                        params=('FLOW_SET', {'channel': info["channel"], 'value': flow_value}),
                        message=f'Ch{info["channel"]}({gas}) 유량 {flow_value}sccm 설정'
                    ),
                    ProcessStep(
                        action=ActionType.MFC_CMD,
                        params=('FLOW_ON', {'channel': info["channel"]}),
                        message=f'Ch{info["channel"]}({gas}) 유량 공급 시작'
                    )
                ])

    def _add_pressure_control_steps(self, steps: List[ProcessStep], working_pressure: float):
        sp1_value = working_pressure / 10.0
        steps.extend([
            ProcessStep(action=ActionType.MFC_CMD, params=('SP4_ON', {}), message='압력 제어(SP4) 시작'),
            ProcessStep(action=ActionType.MFC_CMD, params=('SP1_SET', {'value': sp1_value}),
                        message=f'목표 압력(SP1) {working_pressure:.2f} 설정'),
            ProcessStep(action=ActionType.DELAY, duration=60000, message='압력 안정화 대기 (60초)')
        ])

    def _add_power_and_shutter_steps(
        self, steps: List[ProcessStep], params: Dict[str, Any],
        gun_shutters: List[str], use_dc: bool, use_rf: bool, use_ms: bool,
        dc_power: float, rf_power: float, shutter_delay_sec: float, shutter_delay_min: float
    ):
        # 건 셔터 열기
        for shutter in gun_shutters:
            if params.get(f"use_{shutter.lower()}", False):
                steps.append(ProcessStep(
                    action=ActionType.FADUINO_CMD,
                    params=(shutter, True),
                    message=f'Gun Shutter {shutter} 열기'
                ))

        # 파워 인가(둘 다 쓰면 병렬), 램핑 완료 대기는 각 컨트롤러 내부에서 처리 → no_wait
        want_parallel = use_dc and use_rf
        if use_dc:
            steps.append(ProcessStep(
                action=ActionType.DC_POWER_SET,
                value=dc_power,
                message=f'DC Power {dc_power}W 설정',
                parallel=want_parallel
            ))
        if use_rf:
            steps.append(ProcessStep(
                action=ActionType.RF_POWER_SET,
                value=rf_power,
                message=f'RF Power {rf_power}W 설정',
                parallel=want_parallel
            ))

        steps.append(ProcessStep(
            action=ActionType.MFC_CMD,
            params=('SP1_ON', {}),
            message='압력 제어(SP1) 시작'
        ))

        if shutter_delay_sec > 0:
            steps.append(ProcessStep(
                action=ActionType.DELAY,
                duration=int(shutter_delay_sec * 1000),
                message=f'Shutter Delay {shutter_delay_min}분'
            ))

        if use_ms:
            steps.append(ProcessStep(
                action=ActionType.FADUINO_CMD,
                params=('MS', True),
                message='Main Shutter 열기'
            ))

    def _create_shutdown_sequence(self, params: Dict[str, Any]) -> List[ProcessStep]:
        shutdown_steps: List[ProcessStep] = []
        common_info = self._get_common_process_info(params)
        gun_shutters, gas_info = common_info['gun_shutters'], common_info['gas_info']

        shutdown_steps.append(ProcessStep(action=ActionType.FADUINO_CMD, params=('MS', False), message='Main Shutter 닫기 (항상 닫음)'))

        if common_info['use_dc']:
            shutdown_steps.append(ProcessStep(action=ActionType.DC_POWER_STOP, message='DC Power Off'))
        if common_info['use_rf']:
            shutdown_steps.append(ProcessStep(action=ActionType.RF_POWER_STOP, message='RF Power Off'))

        for gas, info in gas_info.items():
            shutdown_steps.append(ProcessStep(
                action=ActionType.MFC_CMD,
                params=('FLOW_OFF', {'channel': info["channel"]}),
                message=f'Ch{info["channel"]}({gas}) Flow Off'
            ))

        shutdown_steps.append(ProcessStep(action=ActionType.MFC_CMD, params=('VALVE_OPEN', {}), message='전체 MFC Valve Open'))

        for shutter in gun_shutters:
            if params.get(f"use_{shutter.lower()}", False):
                shutdown_steps.append(ProcessStep(
                    action=ActionType.FADUINO_CMD, params=(shutter, False), message=f'Gun Shutter {shutter} 닫기'
                ))

        for gas in gas_info:
            shutdown_steps.append(ProcessStep(
                action=ActionType.FADUINO_CMD, params=(gas, False), message=f'Faduino {gas} 밸브 닫기'
            ))

        shutdown_steps.append(ProcessStep(action=ActionType.FADUINO_CMD, params=('MV', False), message='메인 밸브 닫기'))

        self.log_message.emit("Process", "종료 절차가 생성되었습니다.")
        return shutdown_steps

    def _get_common_process_info(self, params: Dict[str, Any]) -> Dict[str, Any]:
        return {
            'use_ms': bool(params.get("use_ms", False)),
            'use_dc': bool(params.get("use_dc_power", False)) and float(params.get("dc_power", 0)) > 0,
            'use_rf': bool(params.get("use_rf_power", False)) and float(params.get("rf_power", 0)) > 0,
            'gas_info': {"Ar": {"channel": 1}, "O2": {"channel": 2}, "N2": {"channel": 3}},
            'gun_shutters': ["G1", "G2", "G3"]
        }

    # ---------------- 실행/진행 ----------------
    def start_process(self, params: Dict[str, Any]):
        if self.is_running:
            self.log_message.emit("Process", "오류: 이미 다른 공정이 실행 중입니다.")
            return
        try:
            self.current_params = params
            self.process_sequence = self._create_process_sequence(params)

            ok, errors = self.validate_process_sequence()
            if not ok:
                for msg in errors:
                    self.log_message.emit("Process", f"[시퀀스 오류] {msg}")
                raise ValueError("공정 시퀀스 검증 실패")

            self._current_step_idx = -1
            self.is_running = True
            self._aborting = False
            self._accept_completions = True
            self._in_emergency = False

            self.process_status_changed.emit(True)
            self.process_started.emit(params)

            process_name = params.get('process_note', '무제')
            self.log_message.emit("Process", f"=== '{process_name}' 공정 시작 (총 {len(self.process_sequence)}단계) ===")

            self.on_step_completed()  # 첫 스텝으로 진입
        except Exception as e:
            self.log_message.emit("Process", f"공정 시작 오류: {str(e)}")
            self._finish_process(False)

    def _run_next_step(self, step: ProcessStep, step_index: int):
        if not (self._countdown_active and step.action != ActionType.DELAY):
            self.update_process_state.emit(step.message)
    
        if not self.is_running or self._aborting:
            return

        action = step.action
        message = step.message

        self.update_process_state.emit(message)
        self.log_message.emit("Process", f"[STEP {step_index + 1}/{len(self.process_sequence)}] {message}")

        try:
            if action == ActionType.DELAY:
                duration = step.duration or 100
                self.step_timer.start(duration)
                if duration >= 1000:
                    self._start_countdown(duration, message)
                else:
                    self._stop_countdown()
                return

            handlers = {
                ActionType.DC_POWER_SET: lambda: self.dc_power_command_requested.emit(step.value),
                ActionType.DC_POWER_STOP: lambda: self.dc_power_stop_requested.emit(),
                ActionType.RF_POWER_SET: lambda: self.rf_power_command_requested.emit(step.value),
                ActionType.RF_POWER_STOP: lambda: self.rf_power_stop_requested.emit(),
                ActionType.IG_CMD:       lambda: self.ig_command_requested.emit(step.value),
                ActionType.RGA_SCAN:     lambda: self.rga_external_scan_requested.emit(),
                ActionType.FADUINO_CMD:  lambda: self.update_faduino_port.emit(*step.params),
                ActionType.MFC_CMD:      lambda: self.mfc_command_requested.emit(*step.params),
                ActionType.OES_RUN:      lambda: self.oes_command_requested.emit(*step.params)
            }

            handler = handlers.get(action)
            if handler:
                handler()
                if step.no_wait:
                    self._accept_completions = False
                    QTimer.singleShot(0, self._advance_after_nowait)
            else:
                raise ValueError(f"알 수 없는 Action: {action}")

        except Exception as e:
            self.log_message.emit("Process", f"스텝 실행 오류: {str(e)}")
            self.abort_process()

    def on_step_completed(self):
        sender = self.sender()
        if sender is self.step_timer:
            self._stop_countdown()

        if not self._accept_completions or not self.is_running:
            return

        # 병렬 처리 중
        if self._px is not None:
            if not self._px.mark_completed():
                return
            self.log_message.emit("Process", f"병렬 작업 {self._px.total}개 모두 완료")
            self._current_step_idx = self._px.end_index
            self._px = None

        # 폴링 잠시 정지
        self.set_polling.emit(False)

        # 다음 스텝으로
        self._current_step_idx += 1
        if self._current_step_idx >= len(self.process_sequence):
            # ✅ 정지/비상정지로 내려온 경우엔 성공으로 간주하지 않음
            success = not (self._aborting or self._in_emergency)
            self._finish_process(success)
            return

        current_step = self.process_sequence[self._current_step_idx]

        if current_step.parallel:
            # 병렬 블록 수집
            parallel_steps: List[Tuple[ProcessStep, int]] = []
            t = self._current_step_idx
            while t < len(self.process_sequence) and self.process_sequence[t].parallel:
                parallel_steps.append((self.process_sequence[t], t))
                t += 1

            self._px = ParallelExecution([s for (s, _) in parallel_steps], end_index=t - 1)
            self._current_step_idx = self._px.end_index

            # 블록 전체에서 polling 필요 여부를 한 번만 계산
            need_polling = any(s.polling for s, _ in parallel_steps)
            self.set_polling.emit(need_polling)

            self.log_message.emit("Process", f"병렬 작업 {len(parallel_steps)}개 동시 시작...")
            for step, idx in parallel_steps:
                self._run_next_step(step, idx)
        else:
            self.set_polling.emit(current_step.polling)
            self._run_next_step(current_step, self._current_step_idx)

    # ---------------- 장치에서 오는 완료/실패 슬롯 ----------------
    @Slot()
    def on_device_step_ok(self):
        """장치에서 '해당 스텝 완료' 신호를 받을 때 호출."""
        if not self.is_running:
            return
        
        if self._aborting:
            cs = self.current_step
            if not cs or cs.action not in {ActionType.FADUINO_CMD, ActionType.MFC_CMD,
                                        ActionType.DC_POWER_STOP, ActionType.RF_POWER_STOP}:
                return
            
        self.on_step_completed()

    @Slot(str)
    def on_mfc_confirmed(self, cmd: str):
        """MFCController.command_confirmed(cmd) 연결용"""
        if not self.is_running:
            return

        if self._px is not None:
            # 병렬 블록 안에 해당 cmd를 기대하는 MFC_CMD가 있으면 1개 완료로 카운트
            if any(s.action == ActionType.MFC_CMD and s.params and s.params[0] == cmd for s in self._px.steps):
                self.on_device_step_ok()
            else:
                self.log_message.emit("MFC", f"확인 무시: '{cmd}' (현재 병렬 블록 기대와 불일치)")
            return

        # 단일 스텝 로직(기존 유지)
        step = self.current_step
        if not step or step.action != ActionType.MFC_CMD:
            self.log_message.emit("MFC", f"확인 무시: '{cmd}' (현재 스텝: {step.action.name if step else '없음'})")
            return
        expected_cmd = step.params[0] if (step.params and len(step.params) >= 1) else None
        if cmd == expected_cmd:
            self.on_step_completed()
        else:
            self.log_message.emit("MFC", f"확인 무시: '{cmd}', 기대: '{expected_cmd}'")

    @Slot(str, str)
    def on_mfc_failed(self, cmd: str, why: str):
        self.on_step_failed("MFC", f"{cmd}: {why}")

    @Slot(str)
    def on_faduino_confirmed(self, cmd: str):
        """FaduinoController.command_confirmed(cmd) 연결용"""
        if not self.is_running:
            return

        if self._px is not None:
            # 병렬 블록에 FADUINO_CMD가 포함되어 있을 때만 1개 완료로 카운트
            if any(s.action == ActionType.FADUINO_CMD for s in self._px.steps):
                self.on_device_step_ok()
            else:
                self.log_message.emit("Faduino", f"확인 무시: '{cmd}' (현재 병렬 블록에 FADUINO_CMD 없음)")
            return

        # 단일 스텝 상황
        if self.current_step and self.current_step.action == ActionType.FADUINO_CMD:
            self.on_device_step_ok()
        else:
            self.log_message.emit("Faduino", f"확인 무시: '{cmd}' (현재 스텝: {self.current_step.action.name if self.current_step else '없음'})")

    @Slot(str, str)
    def on_faduino_failed(self, cmd: str, why: str):
        self.on_step_failed("Faduino", f"{cmd}: {why}")

    @Slot()
    def on_ig_ok(self):
        """IGController의 '대기 완료' 신호 연결"""
        self.on_device_step_ok()

    @Slot(str, str)
    def on_ig_failed(self, src: str, why: str):
        self.on_step_failed(src or "IG", why)

    @Slot()
    def on_oes_ok(self):
        """OES는 메인 공정 DELAY와 병렬로 동작 → 완료해도 스텝 진행 X"""
        self.log_message.emit("OES", "OES 측정 종료(정상). 메인 공정은 계속 진행됩니다.")

    @Slot(str, str)
    def on_oes_failed(self, src: str, why: str):
        self.on_step_failed(src or "OES", why)

    @Slot()
    def on_rga_finished(self):
        # RGA 스캔이 끝났으니 현재 스텝 완료로 넘어가기
        self.on_step_completed()

    @Slot(str, str)
    def on_rga_failed(self, src: str, why: str):
        # 실패는 표준 실패 핸들러로 전달
        self.on_step_failed(src or "RGA", why)

    @Slot()
    def on_dc_target_reached(self):
        """DCPowerController.target_reached → DC_POWER_SET 스텝 완료"""
        if not self.is_running:
            return
        # 병렬 블록: 현재 병렬 셋에 DC_POWER_SET가 있으면 1개 완료로 카운트
        if self._px is not None:
            if any(s.action == ActionType.DC_POWER_SET for s in self._px.steps):
                self.on_device_step_ok()
            return
        # 단일 스텝: 현재 스텝이 DC_POWER_SET일 때만 완료
        if self.current_step and self.current_step.action == ActionType.DC_POWER_SET:
            self.on_device_step_ok()

    @Slot()
    def on_rf_target_reached(self):
        """RFPowerController.target_reached → RF_POWER_SET 스텝 완료"""
        if not self.is_running:
            return
        if self._px is not None:
            if any(s.action == ActionType.RF_POWER_SET for s in self._px.steps):
                self.on_device_step_ok()
            return
        if self.current_step and self.current_step.action == ActionType.RF_POWER_SET:
            self.on_device_step_ok()

    # ---------------- 실패 처리 ----------------
    @Slot(str, str)
    def on_step_failed(self, source: str, reason: str):
        if not self.is_running:
            return

        if self._aborting:
            self.log_message.emit("Process", f"경고: 종료 중 '{source}' 단계 검증 실패({reason}). 다음 단계로 진행합니다.")
            self.on_step_completed()
            return

        full_reason = f"[{source} - {reason}]"
        self.log_message.emit("Process", f"오류 발생: {full_reason}. 공정을 중단합니다.")
        self.abort_process()

    # ---------------- 카운트다운 ----------------
    def _start_countdown(self, duration_ms: int, base_message: str):
        self._countdown_active = True
        self._countdown_remaining_ms = duration_ms
        self._countdown_base_message = base_message
        self._countdown_timer.start()
        self._on_countdown_tick()

    def _stop_countdown(self):
        self._countdown_timer.stop()
        self._countdown_remaining_ms = 0
        self._countdown_base_message = ""
        self._countdown_timer.stop()
        self._countdown_active = False

    def _on_countdown_tick(self):
        if self._countdown_remaining_ms <= 0:
            self._stop_countdown()
            return
        remaining_sec = self._countdown_remaining_ms // 1000
        minutes = remaining_sec // 60
        seconds = remaining_sec % 60
        time_str = f"{minutes}분 {seconds}초" if minutes > 0 else f"{seconds}초"
        self.update_process_state.emit(f"{self._countdown_base_message} (남은 시간: {time_str})")
        self._countdown_remaining_ms -= 1000

    # ---------------- 종료/비상 ----------------
    def _finish_process(self, success: bool):
        if not self.is_running:
            return
        self.is_running = False
        self._px = None
        self.step_timer.stop()
        self._stop_countdown()
        self.set_polling.emit(False)

        if success:
            self.log_message.emit("Process", "=== 공정이 성공적으로 완료되었습니다 ===")
            self.update_process_state.emit("공정 완료")
        else:
            self.log_message.emit("Process", "=== 공정이 중단되었습니다 ===")
            self.update_process_state.emit("공정 중단됨")
            self.process_aborted.emit()

        self.process_status_changed.emit(False)
        self.process_finished.emit(success)
        self._aborting = False
        self._in_emergency = False

    def abort_process(self):
        if not self.is_running:
            return
        if self._aborting:  # ✅ 이미 중단 중이면 무시
            self.log_message.emit("Process", "(중복) 긴급 중단 진행 중 - 추가 호출 무시")
            return
        #self.process_aborted.emit()

        self.step_timer.stop()
        self._stop_countdown()
        self.set_polling.emit(False)
        self._px = None

        self.log_message.emit("Process", "공정 긴급 중단을 시작합니다...")
        self._aborting = True
        self._accept_completions = False

        emergency_steps = self._create_emergency_shutdown_sequence()
        if emergency_steps:
            self.process_sequence = emergency_steps
            self._current_step_idx = -1
            self._px = None
            self._accept_completions = True
            self.on_step_completed()
        else:
            self._finish_process(False)

    def _advance_after_nowait(self):
        self._accept_completions = True
        self.on_step_completed()

    def _create_emergency_shutdown_sequence(self) -> List[ProcessStep]:
        if not self.current_params:
            return []
        emergency_steps: List[ProcessStep] = []
        common_info = self._get_common_process_info(self.current_params)

        emergency_steps.append(ProcessStep(
            action=ActionType.FADUINO_CMD, params=('MS', False),
            message='[긴급] Main Shutter 즉시 닫기', no_wait=True
        ))

        both = common_info['use_dc'] and common_info['use_rf']
        if common_info['use_dc']:
            emergency_steps.append(ProcessStep(
                action=ActionType.DC_POWER_STOP, message='[긴급] DC Power 즉시 차단',
                parallel=both, no_wait=True
            ))
        if common_info['use_rf']:
            emergency_steps.append(ProcessStep(
                action=ActionType.RF_POWER_STOP, message='[긴급] RF Power 즉시 차단',
                parallel=both, no_wait=True
            ))

        for gas in ["Ar", "O2", "N2"]:
            if self.current_params.get(f"use_{gas.lower()}", False):
                emergency_steps.append(ProcessStep(
                    action=ActionType.FADUINO_CMD, params=(gas, False),
                    message=f'[긴급] {gas} 가스 즉시 차단', no_wait=True
                ))

        emergency_steps.append(ProcessStep(
            action=ActionType.FADUINO_CMD, params=('MV', False),
            message='[긴급] 메인 밸브 즉시 닫기', no_wait=True
        ))

        self.log_message.emit("Process", "긴급 종료 절차가 생성되었습니다.")
        return emergency_steps

    # ---------------- 조회/검증 ----------------
    @property
    def current_step(self) -> Optional[ProcessStep]:
        if 0 <= self._current_step_idx < len(self.process_sequence):
            return self.process_sequence[self._current_step_idx]
        return None

    @property
    def progress(self) -> float:
        if not self.process_sequence:
            return 0.0
        return (self._current_step_idx + 1) / len(self.process_sequence)

    def get_remaining_steps(self) -> List[ProcessStep]:
        if self._current_step_idx < 0:
            return self.process_sequence.copy()
        return self.process_sequence[self._current_step_idx + 1:]

    def get_process_summary(self) -> Dict[str, Any]:
        return {
            'total_steps': len(self.process_sequence),
            'current_step': self._current_step_idx + 1,
            'progress': self.progress,
            'is_running': self.is_running,
            'is_parallel': self._px is not None,
            'parallel_progress': self._px.progress if self._px else 0.0,
            'current_step_info': ({
                'action': self.current_step.action.name,
                'message': self.current_step.message,
                'parallel': self.current_step.parallel
            } if self.current_step else None),
            'process_name': self.current_params.get('process_note', '무제')
        }

    def validate_process_sequence(self) -> Tuple[bool, List[str]]:
        errors = []
        try:
            in_parallel = False
            for i, step in enumerate(self.process_sequence):
                if step.parallel and not in_parallel:
                    in_parallel = True
                elif not step.parallel and in_parallel:
                    in_parallel = False

                step_num = i + 1
                if step.action == ActionType.DELAY and step.duration is None:
                    errors.append(f"Step {step_num}: DELAY 액션에 duration이 없습니다.")
                if step.action in [ActionType.DC_POWER_SET, ActionType.RF_POWER_SET, ActionType.IG_CMD]:
                    if step.value is None:
                        errors.append(f"Step {step_num}: {step.action.name} 액션에 value가 없습니다.")
                if step.action in [ActionType.FADUINO_CMD, ActionType.MFC_CMD, ActionType.OES_RUN]:
                    if step.params is None:
                        errors.append(f"Step {step_num}: {step.action.name} 액션에 params가 없습니다.")
                if step.parallel and i > 0:
                    prev = self.process_sequence[i-1]
                    if not prev.parallel:
                        errors.append(f"Step {step_num}: 병렬 스텝이 비병렬 스텝 뒤에 있습니다.")
        except Exception as e:
            errors.append(f"검증 중 오류 발생: {str(e)}")
        return len(errors) == 0, errors

    def get_estimated_duration(self) -> int:
        return sum((s.duration or 0) for s in self.process_sequence if s.action == ActionType.DELAY)

    def print_process_sequence(self):
        print(f"\n=== 공정 시퀀스 (총 {len(self.process_sequence)}단계) ===")
        for i, step in enumerate(self.process_sequence):
            parallel_mark = "[병렬]" if step.parallel else ""
            polling_mark = "[폴링]" if step.polling else ""
            print(f"{i+1:3d}. {step.action.name:15s} {parallel_mark}{polling_mark}: {step.message}")
            if step.value is not None:
                print(f"     └─ value: {step.value}")
            if step.params is not None:
                print(f"     └─ params: {step.params}")
            if step.duration is not None:
                print(f"     └─ duration: {step.duration}ms")
        estimated_time = self.get_estimated_duration()
        print(f"\n예상 소요 시간: {estimated_time/1000:.1f}초 ({estimated_time/60000:.1f}분)")
        print("="*50)

    # ---------------- 안전/리셋 ----------------
    def stop_now(self):   # 새로 추가 (작은 public 메서드)
        if not self.is_running:
            return
        self._finish_process(False)  # ← 내부에서 step_timer/폴링 중지 + process_aborted emit

    def emergency_stop(self):
        self.log_message.emit("Process", "*** 비상 정지 활성화 ***")
        self._in_emergency = True
        self.update_faduino_port.emit('MS', False)
        self.dc_power_stop_requested.emit()
        self.rf_power_stop_requested.emit()
        self.abort_process()

    def reset_controller(self):
        self.step_timer.stop()
        self._stop_countdown()
        self.is_running = False
        self._aborting = False
        self._accept_completions = True
        self._in_emergency = False
        self._current_step_idx = -1
        self._px = None
        self.process_sequence.clear()
        self.current_params.clear()
        self.process_status_changed.emit(False)
        self.update_process_state.emit("대기 중")
        self.log_message.emit("Process", "프로세스 컨트롤러가 리셋되었습니다.")
