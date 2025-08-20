# process_controller.py (완성된 버전)

from typing import Optional, List, Tuple, Dict, Any
from PyQt6.QtCore import QObject, QTimer, pyqtSignal as Signal, pyqtSlot as Slot
from dataclasses import dataclass
from enum import Enum

# --- Enum 및 Dataclass 정의 ---
class ActionType(str, Enum):
    """공정 단계에서 사용될 모든 Action의 종류를 정의합니다."""
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
    """하나의 공정 단계를 정의하는 데이터클래스입니다."""
    action: ActionType
    message: str
    value: Optional[float] = None
    params: Optional[Tuple] = None
    duration: Optional[int] = None
    parallel: bool = False
    polling: bool = False
    no_wait: bool = False  # 확인응답 없이 즉시 다음 스텝으로 진행
    
    def __post_init__(self):
        """유효성 검사를 수행합니다."""
        # 기본 필수값
        if self.action == ActionType.DELAY:
            if self.duration is None:
                raise ValueError("DELAY 액션은 duration이 필요합니다.")
            if self.parallel:
                raise ValueError("DELAY는 병렬 블록에 포함할 수 없습니다. (단일 타이머)")
        if self.action in (ActionType.DC_POWER_SET, ActionType.RF_POWER_SET, ActionType.IG_CMD):
            if self.value is None:
                raise ValueError(f"{self.action.name} 액션은 value가 필요합니다.")

        # params 구조 검증
        if self.action == ActionType.FADUINO_CMD:
            if not self.params or len(self.params) != 2:
                raise ValueError("FADUINO_CMD params는 (cmd:str, arg:any) 형태여야 합니다.")
        if self.action == ActionType.MFC_CMD:
            if not self.params or len(self.params) != 2 or not isinstance(self.params[1], dict):
                raise ValueError("MFC_CMD params는 (cmd:str, args:dict) 형태여야 합니다.")
        if self.action == ActionType.OES_RUN:
            if not self.params or len(self.params) != 2:
                raise ValueError("OES_RUN params는 (process_time:float, integration_ms:int) 형태여야 합니다.")

# 병렬처리를 위한 클래스
class ParallelExecution:
    """병렬 실행되는 스텝들을 관리하는 클래스입니다."""
    
    def __init__(self, steps: List[ProcessStep], end_index: int):
        self.steps = steps
        self.completed = 0
        self.end_index = end_index
        self.total = len(steps)

    def mark_completed(self) -> bool:
        """스텝 완료를 표시하고 모든 스텝이 완료되었는지 반환합니다."""
        self.completed += 1
        return self.completed >= self.total

    @property
    def progress(self) -> float:
        """완료 진행률을 0.0~1.0으로 반환합니다."""
        return self.completed / self.total if self.total > 0 else 1.0

class ProcessController(QObject):
    """
    전체 증착 공정의 시퀀스를 관리하고 각 장치 컨트롤러에 명령을 내리는 클래스.
    공정 스텝은 ProcessStep 객체의 리스트로 관리되며, 각 스텝의 실행을 통제합니다.
    """
    
    # 1. 각 장치 컨트롤러에 보낼 신호
    update_faduino_port = Signal(str, bool)
    mfc_command_requested = Signal(str, dict)
    oes_command_requested = Signal(float, int)
    dc_power_command_requested = Signal(float)
    dc_power_stop_requested = Signal()
    rf_power_command_requested = Signal(float)
    rf_power_stop_requested = Signal()
    ig_command_requested = Signal(float)
    rga_external_scan_requested = Signal()
    
    # 2. UI 및 상태 제어용 신호
    log_message = Signal(str, str)
    process_status_changed = Signal(bool)
    process_started = Signal(dict)
    process_finished = Signal(bool)
    update_process_state = Signal(str)
    
    # 3. 모든 장치의 폴링을 제어할 단일 신호
    set_polling = Signal(bool)
    
    # 4. 비상 정지 상태를 모든 장치에 전파하기 위한 신호
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
        
        # 스텝 타이머
        self.step_timer = QTimer(self)
        self.step_timer.setSingleShot(True)
        self.step_timer.timeout.connect(self.on_step_completed)

        # 병렬 실행 관리자
        self._px: Optional[ParallelExecution] = None

        # 카운트다운 타이머
        self._countdown_timer = QTimer(self)
        self._countdown_timer.setInterval(1000)
        self._countdown_timer.timeout.connect(self._on_countdown_tick)
        self._countdown_remaining_ms = 0
        self._countdown_base_message = ""

    def _create_process_sequence(self, params: Dict[str, Any]) -> List[ProcessStep]:
        """파라미터를 기반으로 공정 시퀀스를 생성합니다."""
        common_info = self._get_common_process_info(params)
        use_dc, use_rf, use_ms = common_info['use_dc'], common_info['use_rf'], common_info['use_ms']
        gas_info, gun_shutters = common_info['gas_info'], common_info['gun_shutters']
        
        # 파라미터 추출 및 변환
        base_pressure = float(params.get("base_pressure", 1e-5))
        working_pressure = float(params.get("working_pressure", 0))
        process_time_min = float(params.get("process_time", 0))
        shutter_delay_min = float(params.get("shutter_delay", 0))
        shutter_delay_sec = shutter_delay_min * 60.0
        process_time_sec = process_time_min * 60.0
        dc_power = float(params.get("dc_power", 0))
        rf_power = float(params.get("rf_power", 0))
        
        steps: List[ProcessStep] = []

        # === 초기화 단계 ===
        self._add_initialization_steps(steps, base_pressure, gas_info)
        
        # === 가스 주입 단계 ===
        self._add_gas_injection_steps(steps, params, gas_info)
        
        # === 압력 제어 시작 ===
        self._add_pressure_control_steps(steps, working_pressure)
        
        # === 파워 인가 및 셔터 제어 ===
        self._add_power_and_shutter_steps(steps, params, gun_shutters, use_dc, use_rf, use_ms, 
                                        dc_power, rf_power, shutter_delay_sec, shutter_delay_min)
        
        # === 메인 공정 ===
        if process_time_sec > 0:
            steps.append(ProcessStep(
                action=ActionType.DELAY, 
                duration=int(process_time_sec * 1000), 
                message=f'메인 공정 진행 ({process_time_min}분)', 
                polling=True  # 메인 공정 중에는 데이터 수집
            ))

        # === 종료 단계 ===
        shutdown_sequence = self._create_shutdown_sequence(params)
        steps.extend(shutdown_sequence)

        return steps

    def _add_initialization_steps(self, steps: List[ProcessStep], base_pressure: float, gas_info: Dict):
        """초기화 단계 스텝들을 추가합니다."""
        self.log_message.emit("Process", "전체 공정 모드로 시작 (Base Pressure 대기 포함).")
        
        steps.extend([
            ProcessStep(
                action=ActionType.IG_CMD, 
                value=base_pressure, 
                message=f'베이스 압력({base_pressure:.1e}) 도달 대기'
            ),
            ProcessStep(
                action=ActionType.RGA_SCAN, 
                message='RGA 측정 시작'
            )
        ])
        
        # MFC 초기화
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
        
        # MFC Zeroing
        for gas, info in gas_info.items():
            steps.append(ProcessStep(
                action=ActionType.MFC_CMD, 
                params=('MFC_ZEROING', {'channel': info["channel"]}), 
                message=f'Ch{info["channel"]}({gas}) Zeroing'
            ))

    def _add_gas_injection_steps(self, steps: List[ProcessStep], params: Dict[str, Any], gas_info: Dict):
        """가스 주입 단계 스텝들을 추가합니다."""
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
        """압력 제어 단계 스텝들을 추가합니다."""
        sp1_value = working_pressure / 10.0
        steps.extend([
            ProcessStep(
                action=ActionType.MFC_CMD, 
                params=('SP4_ON', {}), 
                message='압력 제어(SP4) 시작'
            ),
            ProcessStep(
                action=ActionType.MFC_CMD, 
                params=('SP1_SET', {'value': sp1_value}), 
                message=f'목표 압력(SP1) {working_pressure:.2f} 설정'
            ),
            ProcessStep(
                action=ActionType.DELAY, 
                duration=60000, 
                message='압력 안정화 대기 (60초)'
            )
        ])

    def _add_power_and_shutter_steps(self, steps: List[ProcessStep], params: Dict[str, Any], 
                                   gun_shutters: List[str], use_dc: bool, use_rf: bool, use_ms: bool,
                                   dc_power: float, rf_power: float, shutter_delay_sec: float, 
                                   shutter_delay_min: float):
        """파워 인가 및 셔터 제어 단계 스텝들을 추가합니다."""
        # Gun Shutter 열기
        for shutter in gun_shutters:
            if params.get(f"use_{shutter.lower()}", False):
                steps.append(ProcessStep(
                    action=ActionType.FADUINO_CMD, 
                    params=(shutter, True), 
                    message=f'Gun Shutter {shutter} 열기'
                ))

        # 파워 인가 (병렬 처리 가능)
        want_parallel_power = use_dc and use_rf
        if use_dc:
            steps.append(ProcessStep(
                action=ActionType.DC_POWER_SET, 
                value=dc_power, 
                message=f'DC Power {dc_power}W 설정', 
                parallel=want_parallel_power
            ))
        if use_rf:
            steps.append(ProcessStep(
                action=ActionType.RF_POWER_SET, 
                value=rf_power, 
                message=f'RF Power {rf_power}W 설정', 
                parallel=want_parallel_power
            ))
        
        steps.append(ProcessStep(
            action=ActionType.MFC_CMD, 
            params=('SP1_ON', {}), 
            message='압력 제어(SP1) 시작'
        ))

        # Shutter Delay
        if shutter_delay_sec > 0:
            steps.append(ProcessStep(
                action=ActionType.DELAY, 
                duration=int(shutter_delay_sec * 1000), 
                message=f'Shutter Delay {shutter_delay_min}분'
            ))
        
        # Main Shutter
        if use_ms:
            steps.append(ProcessStep(
                action=ActionType.FADUINO_CMD, 
                params=('MS', True), 
                message='Main Shutter 열기'
            ))
    
    def _create_shutdown_sequence(self, params: Dict[str, Any]) -> List[ProcessStep]:
        """종료 시퀀스를 생성합니다."""
        shutdown_steps: List[ProcessStep] = []
        common_info = self._get_common_process_info(params)
        gun_shutters, gas_info = common_info['gun_shutters'], common_info['gas_info']

        # Main Shutter 닫기
        shutdown_steps.append(ProcessStep(
            action=ActionType.FADUINO_CMD, 
            params=('MS', False), 
            message='Main Shutter 닫기'
        ))

        # 파워 종료
        if common_info['use_dc']:
            shutdown_steps.append(ProcessStep(
                action=ActionType.DC_POWER_STOP, 
                message='DC Power Off'
            ))
        if common_info['use_rf']:
            shutdown_steps.append(ProcessStep(
                action=ActionType.RF_POWER_STOP, 
                message='RF Power Off'
            ))

        # 가스 Flow 종료
        for gas, info in gas_info.items():
            shutdown_steps.append(ProcessStep(
                action=ActionType.MFC_CMD, 
                params=('FLOW_OFF', {'channel': info["channel"]}), 
                message=f'Ch{info["channel"]}({gas}) Flow Off'
            ))

        # MFC Valve 열기
        shutdown_steps.append(ProcessStep(
            action=ActionType.MFC_CMD, 
            params=('VALVE_OPEN', {}), 
            message='전체 MFC Valve Open'
        ))
        
        # Gun Shutter 닫기
        for shutter in gun_shutters:
            if params.get(f"use_{shutter.lower()}", False):
                shutdown_steps.append(ProcessStep(
                    action=ActionType.FADUINO_CMD, 
                    params=(shutter, False), 
                    message=f'Gun Shutter {shutter} 닫기'
                ))

        # 가스 밸브 닫기
        for gas in gas_info:
            shutdown_steps.append(ProcessStep(
                action=ActionType.FADUINO_CMD, 
                params=(gas, False), 
                message=f'Faduino {gas} 밸브 닫기'
            ))

        # 메인 밸브 닫기
        shutdown_steps.append(ProcessStep(
            action=ActionType.FADUINO_CMD, 
            params=('MV', False), 
            message='메인 밸브 닫기'
        ))
                
        self.log_message.emit("Process", "종료 절차가 생성되었습니다.")
        return shutdown_steps
    
    def _get_common_process_info(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """공통 공정 정보를 추출합니다."""
        return {
            'use_ms': bool(params.get("use_ms", False)),
            'use_dc': bool(params.get("use_dc_power", False)) and float(params.get("dc_power", 0)) > 0,
            'use_rf': bool(params.get("use_rf_power", False)) and float(params.get("rf_power", 0)) > 0,
            'gas_info': {
                "Ar": {"channel": 1}, 
                "O2": {"channel": 2}, 
                "N2": {"channel": 3}
            },
            'gun_shutters': ["G1", "G2", "G3"]
        }

    def start_process(self, params: Dict[str, Any]):
        """공정을 시작합니다."""
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
            
            self.process_status_changed.emit(True)
            self.process_started.emit(params)
            
            process_name = params.get('process_note', '무제')
            self.log_message.emit("Process", f"=== '{process_name}' 공정 시작 (총 {len(self.process_sequence)}단계) ===")
            
            # 첫 번째 스텝 실행
            self.on_step_completed()
            
        except Exception as e:
            self.log_message.emit("Process", f"공정 시작 오류: {str(e)}")
            self._finish_process(False)
    
    def _run_next_step(self, step: ProcessStep, step_index: int):
        """다음 스텝을 실행합니다."""
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
            
            # 각 액션별 신호 발송
            action_handlers = {
                ActionType.DC_POWER_SET: lambda: self.dc_power_command_requested.emit(step.value),
                ActionType.DC_POWER_STOP: lambda: self.dc_power_stop_requested.emit(),
                ActionType.RF_POWER_SET: lambda: self.rf_power_command_requested.emit(step.value),
                ActionType.RF_POWER_STOP: lambda: self.rf_power_stop_requested.emit(),
                ActionType.IG_CMD: lambda: self.ig_command_requested.emit(step.value),
                ActionType.RGA_SCAN: lambda: self.rga_external_scan_requested.emit(),
                ActionType.FADUINO_CMD: lambda: self.update_faduino_port.emit(*step.params),
                ActionType.MFC_CMD: lambda: self.mfc_command_requested.emit(*step.params),
                ActionType.OES_RUN: lambda: self.oes_command_requested.emit(*step.params)
            }
            
            handler = action_handlers.get(action)
            if handler:
                handler()
                
                # no_wait 플래그가 설정된 경우 1초 후 다음 스텝으로 진행
                if step.no_wait:
                    # 0ms로 바로 다음 이벤트 루프 턴에 넘기면서,
                    # 잠깐 완료 수신을 막아 이중 완료를 방지
                    self._accept_completions = False
                    QTimer.singleShot(0, self._advance_after_nowait)
            else:
                raise ValueError(f"알 수 없는 Action: {action}")
                
        except Exception as e:
            self.log_message.emit("Process", f"스텝 실행 오류: {str(e)}")
            self.abort_process()
            
    def on_step_completed(self):
        """스텝 완료 처리를 담당합니다."""
        sender = self.sender()
        if sender is self.step_timer:
            self._stop_countdown()
            
        if not self._accept_completions or not self.is_running:
            return
            
        # 병렬 실행 중인 경우
        if self._px is not None:
            if not self._px.mark_completed():
                # 아직 완료되지 않은 병렬 작업이 있음
                return
            # 모든 병렬 작업 완료
            self.log_message.emit("Process", f"병렬 작업 {self._px.total}개 모두 완료")
            self._current_step_idx = self._px.end_index
            self._px = None
        
        # 폴링 일시 중지
        self.set_polling.emit(False)
        
        # 다음 스텝으로 진행
        self._current_step_idx += 1
        if self._current_step_idx >= len(self.process_sequence):
            self._finish_process(True)
            return

        current_step = self.process_sequence[self._current_step_idx]

        # 병렬 실행 처리
        if current_step.parallel:
            parallel_steps: List[Tuple[ProcessStep, int]] = []
            temp_idx = self._current_step_idx
            
            # 연속된 병렬 스텝들을 수집
            while temp_idx < len(self.process_sequence) and self.process_sequence[temp_idx].parallel:
                parallel_steps.append((self.process_sequence[temp_idx], temp_idx))
                temp_idx += 1

            self._px = ParallelExecution([s for (s, _) in parallel_steps], end_index=temp_idx - 1)
            self._current_step_idx = self._px.end_index

            # 2) 병렬 묶음 전체에서 polling 필요 여부를 한 번만 계산/토글
            need_polling = any(s.polling for s, _ in parallel_steps)
            self.set_polling.emit(need_polling)

            self.log_message.emit("Process", f"병렬 작업 {len(parallel_steps)}개 동시 시작...")

            # 3) 병렬 스텝들 실행 (루프는 한 번만!)
            for step, step_idx in parallel_steps:
                self._run_next_step(step, step_idx)

        else:
            # 단일 스텝 실행도 묶음 기준으로 한 번만 토글
            self.set_polling.emit(current_step.polling)
            self._run_next_step(current_step, self._current_step_idx)

    @Slot(str, str)
    def on_step_failed(self, source: str, reason: str):
        """장치로부터 실패 신호를 받았을 때 호출되는 범용 슬롯입니다."""
        if not self.is_running:
            return

        if self._aborting:
            # 중단 절차 중 발생하는 실패는 무시하고 다음 종료 스텝으로 진행
            self.log_message.emit(
                "Process",
                f"경고: 종료 중 '{source}' 단계 검증 실패({reason}). 다음 단계로 진행합니다."
            )
            self.on_step_completed() # 다음 종료 스텝을 실행하기 위해 호출
            return

        full_reason = f"[{source} - {reason}]"
        self.log_message.emit("Process", f"오류 발생: {full_reason}. 공정을 중단합니다.")
        self.abort_process()

    def _start_countdown(self, duration_ms: int, base_message: str):
        """카운트다운을 시작합니다."""
        self._countdown_remaining_ms = duration_ms
        self._countdown_base_message = base_message
        self._countdown_timer.start()
        self._on_countdown_tick()  # 즉시 첫 번째 업데이트

    def _stop_countdown(self):
        """카운트다운을 중지합니다."""
        self._countdown_timer.stop()
        self._countdown_remaining_ms = 0
        self._countdown_base_message = ""
        self._countdown_timer.stop()

    def _on_countdown_tick(self):
        """카운트다운 틱 처리"""
        if self._countdown_remaining_ms <= 0:
            self._stop_countdown()
            return
            
        remaining_sec = self._countdown_remaining_ms // 1000
        minutes = remaining_sec // 60
        seconds = remaining_sec % 60
        
        if minutes > 0:
            time_str = f"{minutes}분 {seconds}초"
        else:
            time_str = f"{seconds}초"
            
        countdown_message = f"{self._countdown_base_message} (남은 시간: {time_str})"
        self.update_process_state.emit(countdown_message)
        
        self._countdown_remaining_ms -= 1000

    def _finish_process(self, success: bool):
        """공정을 종료합니다."""
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
        
        self.process_status_changed.emit(False)
        self.process_finished.emit(success)

    def abort_process(self):
        """공정을 긴급 중단합니다."""
        if not self.is_running:
            return
            
        self.step_timer.stop()
        self._stop_countdown()
        self.set_polling.emit(False)
        self._px = None

        self.log_message.emit("Process", "공정 긴급 중단을 시작합니다...")
        self._aborting = True
        self._accept_completions = False
        
        # 모든 장치에 중단 신호 전파
        self.process_aborted.emit()
        
        # 긴급 종료 시퀀스 실행
        emergency_steps = self._create_emergency_shutdown_sequence()
        if emergency_steps:
            self.process_sequence = emergency_steps
            self._current_step_idx = -1
            self._px = None
            self._accept_completions = True
            self.on_step_completed()
        else:
            self._finish_process(False)

    # 클래스 메서드 추가
    def _advance_after_nowait(self):
        # no_wait용: 순간적으로 완료 수용을 막았다가 다시 열고 완료 처리
        self._accept_completions = True
        self.on_step_completed()

    def _create_emergency_shutdown_sequence(self) -> List[ProcessStep]:
        """긴급 종료 시퀀스를 생성합니다."""
        if not self.current_params:
            return []
            
        emergency_steps: List[ProcessStep] = []
        common_info = self._get_common_process_info(self.current_params)
        
        # Main Shutter 가장 먼저 즉시 닫기 (증착 차단이 최우선)
        emergency_steps.append(ProcessStep(
            action=ActionType.FADUINO_CMD, 
            params=('MS', False), 
            message='[긴급] Main Shutter 즉시 닫기',
            no_wait=True  # 확인응답 없이 즉시 다음 스텝
        ))
        
        # 모든 파워 즉시 병렬 차단
        both_power_used = common_info['use_dc'] and common_info['use_rf']
        if common_info['use_dc']:
            emergency_steps.append(ProcessStep(
                action=ActionType.DC_POWER_STOP, 
                message='[긴급] DC Power 즉시 차단',
                parallel=both_power_used,
                no_wait=True
            ))
        if common_info['use_rf']:
            emergency_steps.append(ProcessStep(
                action=ActionType.RF_POWER_STOP, 
                message='[긴급] RF Power 즉시 차단',
                parallel=both_power_used,
                no_wait=True
            ))
        
        # 주요 가스만 즉시 차단 (Ar 등 주 가스)
        gas_info = common_info['gas_info']
        for gas in ["Ar", "O2", "N2"]:  # 모든 가스 즉시 차단
            if self.current_params.get(f"use_{gas.lower()}", False):
                emergency_steps.append(ProcessStep(
                    action=ActionType.FADUINO_CMD, 
                    params=(gas, False), 
                    message=f'[긴급] {gas} 가스 즉시 차단',
                    no_wait=True
                ))
        
        # 메인 밸브 즉시 닫기
        emergency_steps.append(ProcessStep(
            action=ActionType.FADUINO_CMD, 
            params=('MV', False), 
            message='[긴급] 메인 밸브 즉시 닫기',
            no_wait=True
        ))
        
        self.log_message.emit("Process", "긴급 종료 절차가 생성되었습니다.")
        return emergency_steps

    # === 프로퍼티 및 상태 조회 메서드 ===
    @property
    def current_step(self) -> Optional[ProcessStep]:
        """현재 실행 중인 스텝을 반환합니다."""
        if 0 <= self._current_step_idx < len(self.process_sequence):
            return self.process_sequence[self._current_step_idx]
        return None

    @property
    def progress(self) -> float:
        """공정 진행률을 0.0~1.0으로 반환합니다."""
        if not self.process_sequence:
            return 0.0
        return (self._current_step_idx + 1) / len(self.process_sequence)

    def get_remaining_steps(self) -> List[ProcessStep]:
        """남은 스텝들을 반환합니다."""
        if self._current_step_idx < 0:
            return self.process_sequence.copy()
        return self.process_sequence[self._current_step_idx + 1:]

    def get_process_summary(self) -> Dict[str, Any]:
        """현재 공정의 요약 정보를 반환합니다."""
        return {
            'total_steps': len(self.process_sequence),
            'current_step': self._current_step_idx + 1,
            'progress': self.progress,
            'is_running': self.is_running,
            'is_parallel': self._px is not None,
            'parallel_progress': self._px.progress if self._px else 0.0,
            'current_step_info': {
                'action': self.current_step.action.name if self.current_step else None,
                'message': self.current_step.message if self.current_step else None,
                'parallel': self.current_step.parallel if self.current_step else False
            } if self.current_step else None,
            'process_name': self.current_params.get('process_note', '무제')
        }

    # === 디버깅 및 검증 메서드 ===
    def validate_process_sequence(self) -> Tuple[bool, List[str]]:
        """공정 시퀀스의 유효성을 검사합니다."""
        errors = []
        
        try:
            # (기존 병렬 오류 체크 삭제하고) 병렬 블록의 연속성만 확인
            in_parallel = False
            for i, step in enumerate(self.process_sequence):
                if step.parallel and not in_parallel:
                    in_parallel = True  # 병렬 블록 시작
                elif not step.parallel and in_parallel:
                    in_parallel = False  # 병렬 블록 종료
            # 별도 오류 없음: 병렬은 '연속된 True' 구간이면 충분

                step_num = i + 1
                
                # 필수 값 검증
                if step.action == ActionType.DELAY and step.duration is None:
                    errors.append(f"Step {step_num}: DELAY 액션에 duration이 없습니다.")
                    
                if step.action in [ActionType.DC_POWER_SET, ActionType.RF_POWER_SET, ActionType.IG_CMD]:
                    if step.value is None:
                        errors.append(f"Step {step_num}: {step.action.name} 액션에 value가 없습니다.")
                        
                if step.action in [ActionType.FADUINO_CMD, ActionType.MFC_CMD, ActionType.OES_RUN]:
                    if step.params is None:
                        errors.append(f"Step {step_num}: {step.action.name} 액션에 params가 없습니다.")
                
                # 병렬 실행 검증
                if step.parallel and i > 0:
                    prev_step = self.process_sequence[i-1]
                    if not prev_step.parallel:
                        errors.append(f"Step {step_num}: 병렬 스텝이 비병렬 스텝 뒤에 있습니다.")
                        
        except Exception as e:
            errors.append(f"검증 중 오류 발생: {str(e)}")
            
        return len(errors) == 0, errors

    def get_estimated_duration(self) -> int:
        return sum((s.duration or 0) for s in self.process_sequence if s.action == ActionType.DELAY)

    def print_process_sequence(self):
        """공정 시퀀스를 콘솔에 출력합니다 (디버깅용)."""
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

    # === 안전 및 에러 처리 ===
    def emergency_stop(self):
        """모든 장치를 즉시 정지시키는 비상 정지 기능."""
        self.log_message.emit("Process", "*** 비상 정지 활성화 ***")
        self._in_emergency = True

        # Main Shutter 즉시 닫기
        self.update_faduino_port.emit('MS', False)
        
        # 모든 파워 즉시 차단
        self.dc_power_stop_requested.emit()
        self.rf_power_stop_requested.emit()
        
        # 공정 중단
        self.abort_process()

    def reset_controller(self):
        """컨트롤러를 초기 상태로 리셋합니다."""
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

    # === 외부 이벤트 처리 ===
    # @Slot()
    # def on_device_error(self, device_name: str, error_message: str):
    #     """장치 오류 발생 시 호출됩니다."""
    #     self.log_message.emit("Process", f"장치 오류 발생 [{device_name}]: {error_message}")
    #     if self.is_running:
    #         self.abort_process()

    # @Slot()
    # def on_safety_interlock(self, interlock_name: str):
    #     """안전 인터록 발생 시 호출됩니다."""
    #     self.log_message.emit("Process", f"안전 인터록 발생 [{interlock_name}] - 비상 정지 실행")
    #     self.emergency_stop()

    # # === 설정 및 구성 ===
    # def set_device_timeout(self, device: str, timeout_ms: int):
    #     """특정 장치의 타임아웃을 설정합니다."""
    #     # 구현 예정: 각 장치별 타임아웃 관리
    #     pass

    # def get_supported_actions(self) -> List[str]:
    #     """지원되는 모든 액션 타입을 반환합니다."""
    #     return [action.name for action in ActionType]
