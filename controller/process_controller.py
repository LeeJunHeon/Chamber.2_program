# process_controller.py

from PyQt6.QtCore import QObject, QTimer, pyqtSignal as Signal, pyqtSlot as Slot

class ProcessController(QObject):
    """
    전체 증착 공정의 시퀀스를 관리하고 각 장치 컨트롤러에 명령을 내리는 클래스.
    공정 스텝은 딕셔너리 리스트로 관리되며, 각 스텝의 실행을 통제합니다.
    """
    # 1. 각 장치 컨트롤러에 보낼 신호
    update_faduino_port = Signal(str, bool)    # Faduino의 시리얼 포트 연결/해제를 요청합니다. (포트 이름, 연결 상태)
    mfc_command_requested = Signal(str, dict)  # MFC 컨트롤러에 특정 명령을 요청합니다. (명령어, 파라미터 딕셔너리)
    oes_command_requested = Signal(float, int) # OES 컨트롤러에 측정을 요청합니다. (총 측정 시간(초), 적분 시간(ms))
    dc_power_command_requested = Signal(float) # DC 파워 서플라이에 파워 설정을 요청합니다. (파워 값(W))
    dc_power_stop_requested = Signal()         # DC 파워 정지를 요청하는 신호
    rf_power_command_requested = Signal(float) # RF 파워 서플라이에 파워 설정을 요청합니다. (파워 값(W))
    rf_power_stop_requested = Signal()         # RF 파워 정지를 요청하는 신호
    ig_command_requested = Signal(float)       # IG 컨트롤러에 압력 설정을 요청합니다. (목표 압력 값)
    rga_external_scan_requested = Signal()     # 외부 RGA 분석 프로그램을 실행하도록 요청합니다.

    # 2. UI 및 상태 제어용 신호
    log_message = Signal(str, str)             # UI의 로그 창에 메시지를 보냅니다. (메시지 출처, 내용)
    process_status_changed = Signal(bool)      # 공정의 실행/정지 상태를 UI에 알려 버튼 등을 활성/비활성화합니다. (실행 중: True)
    process_started = Signal(dict)             # 새 공정이 시작될 때 DataLogger 등 다른 모듈에 공정 파라미터를 전달합니다.
    process_finished = Signal(bool)            # 공정이 완료(성공/실패)되었음을 알립니다. (성공 시: True)
    update_process_state = Signal(str)         # UI의 특정 라벨(상태 표시줄)에 현재 진행 중인 스텝의 메시지를 표시합니다.

    # 3. 모든 장치의 폴링을 제어할 단일 신호
    set_polling = Signal(bool)                 # 데이터 수집이 필요한 모든 장치(Faduino, MFC 등)의 폴링을 동시에 켜거나 끕니다. (True: 켜기, False: 끄기)

    # [신규] 비상 정지 상태를 모든 장치에 전파(방송)하기 위한 신호
    process_aborted = Signal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.process_sequence = []
        self._current_step_idx = -1
        self.is_running = False
        self.current_params = {} # abort시 사용될 현재 공정 파라미터 저장
        self._aborting = False
        
        # 스텝 사이의 Delay 타이머
        self.step_timer = QTimer(self)
        self.step_timer.setSingleShot(True)
        self.step_timer.timeout.connect(self.on_step_completed)

        # 완료해야 할 병렬 작업 수를 세는 카운터
        self._parallel_tasks_remaining = 0

        # UI timer를 위한 코드
        self._countdown_timer = QTimer(self)
        self._countdown_timer.setInterval(1000)
        self._countdown_timer.timeout.connect(self._on_countdown_tick)

        self._countdown_remaining_ms = 0
        self._countdown_base_message = ""

    def _create_process_sequence(self, params: dict) -> list:
        """
        UI 입력을 바탕으로 전체 공정 순서를 정의합니다.
        기존 _process_steps의 모든 로직(주석 포함)을 딕셔너리 스텝 방식으로 완벽하게 재구성했습니다.
        """
        # ======================================================================
        # 1. 파라미터 추출 및 기본값 설정
        # ======================================================================
        common_info = self._get_common_process_info(params)

        use_dc = common_info['use_dc']
        use_rf = common_info['use_rf']
        use_ms = common_info['use_ms']
        gas_info = common_info['gas_info']
        gun_shutters = common_info['gun_shutters']

        base_pressure = float(params.get("base_pressure", 1e-5))
        working_pressure = float(params.get("working_pressure", 0))

        process_time_min = float(params.get("process_time", 0))
        shutter_delay_min = float(params.get("shutter_delay", 0))
        shutter_delay_sec = shutter_delay_min * 60.0
        process_time_sec  = process_time_min * 60.0

        dc_power = float(params.get("dc_power", 0))
        rf_power = float(params.get("rf_power", 0))
        
        steps = []

        # ======================================================================
        # 2. 공정 단계 리스트 생성 (딕셔너리 방식으로 재구성)
        # ======================================================================

        # --- 2-1. 초기화 단계 ---
        self.log_message.emit("Process", "전체 공정 모드로 시작 (Base Pressure 대기 포함).")
        steps.append({'action': 'IG_CMD', 'value': base_pressure, 'message': f'베이스 압력({base_pressure:.1e}) 도달 대기'})
        #steps.append({'action': 'RGA_SCAN', 'message': 'RGA 측정 시작'})
        
        for gas, info in gas_info.items():
            steps.append({'action': 'MFC_CMD', 'params': ('FLOW_OFF', {'channel': info["channel"]}), 'message': f'Ch{info["channel"]}({gas}) Flow Off'})
        
        steps.append({'action': 'MFC_CMD', 'params': ('VALVE_OPEN', {}), 'message': 'MFC Valve Open'})
        steps.append({'action': 'MFC_CMD', 'params': ('PS_ZEROING', {}), 'message': '압력 센서 Zeroing'})
        
        for gas, info in gas_info.items():
            steps.append({'action': 'MFC_CMD', 'params': ('MFC_ZEROING', {'channel': info["channel"]}), 'message': f'Ch{info["channel"]}({gas}) Zeroing'})

        # --- 2-2. 가스 주입 단계 ---
        # (추가) 개별 가스 밸브를 열기 전에 메인 밸브를 먼저 엽니다.
        steps.append({'action': 'FADUINO_CMD', 'params': ('MV', True), 'message': '메인 밸브 열기'})

        for gas, info in gas_info.items():
            if params.get(f"use_{gas.lower()}", False):
                flow_value = params.get(f"{gas.lower()}_flow", 0)
                steps.append({'action': 'FADUINO_CMD', 'params': (gas, True), 'message': f'{gas} 밸브 열기'})
                steps.append({'action': 'MFC_CMD', 'params': ('FLOW_SET', {'channel': info["channel"], 'value': flow_value}), 'message': f'Ch{info["channel"]}({gas}) 유량 {flow_value}sccm 설정'})
                steps.append({'action': 'MFC_CMD', 'params': ('FLOW_ON', {'channel': info["channel"]}), 'message': f'Ch{info["channel"]}({gas}) 유량 공급 시작'})

        # --- 2-3. 압력 제어 시작 ---
        sp1_value = working_pressure / 10.0
        steps.append({'action': 'MFC_CMD', 'params': ('SP4_ON', {}), 'message': '압력 제어(SP4) 시작'})
        steps.append({'action': 'MFC_CMD', 'params': ('SP1_SET', {'value': sp1_value}), 'message': f'목표 압력(SP1) {working_pressure:.2f} 설정'})

        # delay 1분
        steps.append({'action': 'DELAY', 'duration': 60000, 'message': '압력 안정화 대기 (60초)'})

        # --- 2-4. 파워 인가 및 셔터 제어 ---
        # Gun shutter 개방
        for shutter in gun_shutters:
            if params.get(f"use_{shutter.lower()}", False):
                steps.append({'action': 'FADUINO_CMD', 'params': (shutter, True), 'message': f'Gun Shutter {shutter} 열기'})

        # DC/RF 파워 동시 설정 (병렬 처리)
        want_parallel_power = use_dc and use_rf

        if use_dc:
            steps.append({'action': 'DC_POWER_SET', 'value': dc_power, 'message': f'DC Power {dc_power}W 설정', 'parallel': want_parallel_power})
        if use_rf:
            steps.append({'action': 'RF_POWER_SET', 'value': rf_power, 'message': f'RF Power {rf_power}W 설정', 'parallel': want_parallel_power})
        
        # SP1 on
        steps.append({'action': 'MFC_CMD', 'params': ('SP1_ON', {}), 'message': '압력 제어(SP1) 시작'})

        # Shutter delay
        if shutter_delay_sec > 0:
            steps.append({'action': 'DELAY', 'duration': int(shutter_delay_sec * 1000), 'message': f'Shutter Delay {shutter_delay_min}분'})
        
        # Main shutter open
        if use_ms:
            steps.append({'action': 'FADUINO_CMD', 'params': ('MS', True), 'message': 'Main Shutter 열기'})

        # --- 2-5. 메인 공정 (데이터 수집) ---
        if process_time_sec > 0:
            oes_integration_time = int(params.get("integration_time", 1000))
            #steps.append({'action': 'OES_RUN', 'params': (process_time_sec, oes_integration_time), 'message': f'OES 측정 시작 ({process_time_min}분)', 'parallel': True})
            # DELAY 액션에 polling:True 플래그를 추가하여 이 시간 동안만 데이터 수집
            steps.append({'action': 'DELAY', 'duration': int(process_time_sec * 1000), 'message': f'메인 공정 진행 ({process_time_min}분)'})
            #, 'polling': True, 'parallel': True 삭제

        # --- 2-6. 종료 단계 ---
        shutdown_sequence = self._create_shutdown_sequence(params=params)
        steps.extend(shutdown_sequence)

        return steps
    
    def _create_shutdown_sequence(self, params: dict) -> list:
        """
        공정 종료 절차를 생성합니다. 모든 종료 시나리오를 이 함수에서 관리합니다.

        Args:
            params (dict): 어떤 장비(DC, RF, MS)가 사용되었는지 확인하기 위한 공정 파라미터.

        Returns:
            list: 생성된 종료 스텝 딕셔셔리 리스트.
        """
        shutdown_steps = []
        common_info = self._get_common_process_info(params)
        gun_shutters = common_info['gun_shutters']
        gas_info = common_info['gas_info']

        # 1. Main Shutter close (항상)
        shutdown_steps.append({'action': 'FADUINO_CMD', 'params': ('MS', False), 'message': 'Main Shutter 닫기'})

        # 2. 사용한 Power off
        if common_info['use_dc']:
            shutdown_steps.append({'action': 'DC_POWER_STOP', 'message': 'DC Power Off'})
        if common_info['use_rf']:
            shutdown_steps.append({'action': 'RF_POWER_STOP', 'message': 'RF Power Off'})

        # 3. MFC Flow off (모든 flow)
        for gas, info in gas_info.items():
            # 사용 여부와 관계없이 모든 가스를 끄도록 설정
            shutdown_steps.append({
                'action': 'MFC_CMD',
                'params': ('FLOW_OFF', {'channel': info["channel"]}),
                'message': f'Ch{info["channel"]}({gas}) Flow Off'
            })

        # 4. MFC Valve open (항상)
        shutdown_steps.append({'action': 'MFC_CMD', 'params': ('VALVE_OPEN', {}), 'message': '전체 MFC Valve Open'})
        
        # 5. Gun Shutter close (모든 건 셔터 닫기)
        for shutter in gun_shutters:
            if params.get(f"use_{shutter.lower()}", False):
                shutdown_steps.append({'action': 'FADUINO_CMD', 'params': (shutter, False), 'message': f'Gun Shutter {shutter} 닫기'})

        # 6. Faduino Gas Valve close (모든 가스 밸브 닫기)
        for gas in gas_info:
            shutdown_steps.append({'action': 'FADUINO_CMD', 'params': (gas, False), 'message': f'Faduino {gas} 밸브 닫기'})

        # 7. Faduino Main Gas close (항상)
        shutdown_steps.append({'action': 'FADUINO_CMD', 'params': ('MV', False), 'message': '메인 밸브 닫기'})
                
        self.log_message.emit("Process", "종료 절차가 생성되었습니다.")

        return shutdown_steps
    
    def _get_common_process_info(self, params: dict) -> dict:
        """
        공정 파라미터를 기반으로 중복 사용되는 정보(장비 사용 플래그, 가스 정보 등)를
        한 번에 생성하여 반환하는 헬퍼 메서드입니다.
        """
        info = {
            'use_ms': bool(params.get("use_ms", False)),
            'use_dc': bool(params.get("use_dc_power", False)) and float(params.get("dc_power", 0)) > 0,
            'use_rf': bool(params.get("use_rf_power", False)) and float(params.get("rf_power", 0)) > 0,
            'gas_info': {"Ar": {"channel": 1}, "O2": {"channel": 2}, "N2": {"channel": 3}},
            'gun_shutters': ["G1", "G2", "G3"]
        }
        return info

    def start_process(self, params: dict):
        """UI 입력 기반으로 공정 시퀀스를 생성하고 실행을 시작합니다."""
        if self.is_running:
            self.log_message.emit("Process", "오류: 이미 다른 공정이 실행 중입니다.")
            return
        
        self.current_params = params

        # 1. 공정 계획서(sequence) 생성
        self.process_sequence = self._create_process_sequence(params)
        
        # 2. 공정 상태 초기화 및 시작
        self._current_step_idx = -1
        self.is_running = True
        self.process_status_changed.emit(True)
        
        # 3. 데이터 로거 및 UI에 공정 시작 알림
        self.process_started.emit(params)
        self.log_message.emit("Process", f"=== '{params.get('process_note', '무제')}' 공정 시작 ===")
        
        # 4. 첫 스텝 실행
        self.on_step_completed()

    def _run_next_step(self, step: dict, step_index: int):
        """
        주어진 스텝(step)과 인덱스(step_index)를 기반으로 action을 해석하여
        각 장비에 정확한 명령 신호를 보냅니다.
        """
        if not self.is_running:
            return
    
        action = step.get('action')
        message = step.get('message', '')

        # step_index를 직접 사용하여 로그를 출력하므로 .index() 호출이 불필요
        self.update_process_state.emit(message)
        self.log_message.emit("Process", f"[STEP {step_index + 1}/{len(self.process_sequence)}] {message}")

        # Action에 따라 해당하는 시그널을 올바른 파라미터 형식으로 emit
        if action == 'DELAY':
            duration = int(step.get('duration', 100))
            msg = step.get('message', '')
            self.step_timer.start(duration)

            # 1초 미만이면 카운트다운 생략
            if duration >= 1000:
                self._start_countdown(duration, msg)
            else:
                self._stop_countdown()
            return
        
        elif action == 'DC_POWER_SET':
            self.dc_power_command_requested.emit(step['value'])
            return  # 비동기 작업이므로 완료 신호를 기다려야 함
        
        elif action == 'DC_POWER_STOP':
            self.dc_power_stop_requested.emit()
            return

        elif action == 'RF_POWER_SET':
            self.rf_power_command_requested.emit(step['value'])
            return  # 비동기 작업이므로 완료 신호를 기다려야 함

        elif action == 'RF_POWER_STOP':
            self.rf_power_stop_requested.emit()
            return # 비동기 작업이므로 완료 신호를 기다리기 위해 리턴

        elif action == 'FADUINO_CMD':
            cmd, args = step['params']
            self.update_faduino_port.emit(cmd, args)
            return # 비동기 작업이므로 완료 신호를 기다리기 위해 리턴

        elif action == 'MFC_CMD':
            cmd, args = step['params']
            self.mfc_command_requested.emit(cmd, args)
            return  # MFC 명령은 비동기이므로 완료 신호를 기다려야 함

        elif action == 'OES_RUN':
            process_time, integration_time = step['params']
            self.oes_command_requested.emit(process_time, integration_time)
            return  # OES 측정은 비동기이므로 완료 신호를 기다려야 함

        elif action == 'RGA_SCAN':
            self.rga_external_scan_requested.emit()
            return  # RGA 실행은 비동기이므로 완료 신호를 기다려야 함

        elif action == 'IG_CMD':
            target_pressure = step.get('value')
            self.ig_command_requested.emit(target_pressure)
            return # 비동기 작업이므로, 완료 신호를 기다리기 위해 여기서 리턴
        else:
            self.log_message.emit("Process", f"치명적 오류: 알 수 없는 Action ('{action}')을 만나 공정을 중단합니다.")
            self.abort_process()
            return
            
    @Slot()
    def on_step_completed(self):
        """
        한 스텝이 완료되면 다음 스텝을 준비하고 실행합니다.
        병렬 처리('parallel': True) 스텝을 인식하여 동시에 실행합니다.
        """

        sender = self.sender()
        # ✅ Delay(= self.step_timer) 종료일 때만 카운트다운 중지
        if sender is self.step_timer:
            self._stop_countdown()

        # --- 1. 병렬 작업이 진행 중이었는지 먼저 확인 ---
        if self._parallel_tasks_remaining > 0:
            self._parallel_tasks_remaining -= 1
            if self._parallel_tasks_remaining > 0:
                return
        
        if not self.is_running: 
            return
        
        self.set_polling.emit(False)

        # --- 2. 다음 스텝 준비 ---
        self._current_step_idx += 1
        if self._current_step_idx >= len(self.process_sequence):
            self._finish_process(True)
            return

        # --- 3. 다음 스텝(들)이 병렬인지 확인하고 실행 ---
        first_step = self.process_sequence[self._current_step_idx]

        if first_step.get('parallel', False):
            # ✅ 연속된 parallel=True 스텝만 수집 (비병렬 스텝은 포함하지 않음)
            parallel_steps_to_run = []
            temp_idx = self._current_step_idx
            while temp_idx < len(self.process_sequence) and self.process_sequence[temp_idx].get('parallel', False):
                parallel_steps_to_run.append((self.process_sequence[temp_idx], temp_idx))
                temp_idx += 1

            # ✅ 현재 인덱스를 '마지막 병렬 스텝'으로 고정
            #    → 모든 병렬 작업이 완료되어(_parallel_tasks_remaining==0) on_step_completed가 다시 호출되면
            #      _current_step_idx += 1 로 첫 비병렬 스텝으로 정확히 이동
            self._current_step_idx = temp_idx - 1

            self.log_message.emit("Process", f"병렬 작업 {len(parallel_steps_to_run)}개 동시 시작...")
            self._parallel_tasks_remaining = len(parallel_steps_to_run)

            for step, step_idx in parallel_steps_to_run:
                if step.get('polling', False):
                    self.set_polling.emit(True)
                self._run_next_step(step, step_idx)
            return

        else:
            # ✅ 병렬 아님: 단일 스텝만 실행
            step = first_step
            if step.get('polling', False):
                self.set_polling.emit(True)
            self._run_next_step(step, self._current_step_idx)
            return

    @Slot(str, str)
    def on_step_failed(self,source: str, reason: str):
        """
        장치(IG, MFC 등)로부터 실패 신호를 받았을 때 호출되는 범용 슬롯입니다.
        """
        if not self.is_running:
            return

        # 신호를 보낸 객체(sender)로부터 연결을 끊어 중복 호출을 방지합니다.
        sender = self.sender()
        if sender:
            try:
                sender.disconnect(self.on_step_completed)
                sender.disconnect(self.on_step_failed)
            except TypeError:
                pass # 이미 끊긴 경우 발생하는 오류는 무시합니다.

        if self._aborting:
            self.log_message.emit(
                "Process",
                f"경고: 종류 중 '{source}' 단계 검증 실패({reason}). 실패를 무시하고 다음 단계로 진행합니다."
            )
            self.on_step_completed()
            return

        full_reason = f"[{source} {reason}]"
        self.log_message.emit("Process", f"오류 발생: {full_reason}. 공정을 중단합니다.")
        # 공정을 비상 종료하는 abort_process() 메소드를 호출합니다.
        self.abort_process()

    def _start_countdown(self, duration_ms: int, base_message: str):
        self._countdown_remaining_ms = max(0, int(duration_ms))
        self._countdown_base_message = base_message or ""
        self._emit_countdown_label()
        if not self._countdown_timer.isActive():
            self._countdown_timer.start()

    def _stop_countdown(self):
        if self._countdown_timer.isActive():
            self._countdown_timer.stop()
        self._countdown_remaining_ms = 0
        self._countdown_base_message = ""

    def _on_countdown_tick(self):
        self._countdown_remaining_ms = max(0, self._countdown_remaining_ms - 1000)
        self._emit_countdown_label()
        if self._countdown_remaining_ms <= 0:
            self._stop_countdown()

    def _emit_countdown_label(self):
        total_sec = self._countdown_remaining_ms // 1000
        mm, ss = divmod(total_sec, 60)
        if self._countdown_base_message:
            self.update_process_state.emit(f"{self._countdown_base_message} (남은 {mm:02d}:{ss:02d})")


    def _finish_process(self, was_successful: bool):
        """공정 완료 또는 중단 시 호출됩니다."""

        self._stop_countdown()

        if self._aborting:
            was_successful = False      # ← 중단은 실패로 보고
            self._aborting = False

        if not self.is_running:
            return
            
        self.step_timer.stop()
        self.set_polling.emit(False) # 확실하게 폴링 종료
        
        self.is_running = False
        self.process_status_changed.emit(False)
        
        status_msg = "성공적으로 완료" if was_successful else "실패 또는 중단"
        self.log_message.emit("Process", f"=== 공정 {status_msg} ===")

        # [추가] 모든 작업(정상/비정상 종료 포함)이 끝났음을 알린 후,
        # main.py에 연결된 모든 장비의 cleanup을 호출하도록 신호를 보냅니다.
        if not was_successful:
            self.process_aborted.emit()

        # main.py의 공정 큐가 다음 공정을 진행할지 결정하도록 결과를 보고합니다.
        self.process_finished.emit(was_successful)

    @Slot()
    def abort_process(self):
        """[수정됨] 외부에서 공정을 강제 중단시킬 때, 안전 종료 절차를 '시작'합니다."""

        if getattr(self, "_aborting", False):
            self.log_message.emit("Process", "이미 종료 중입니다. 추가 정지 요청은 무시합니다.")
            return

        self._stop_countdown()
        self._aborting = True

        # 이미 중단 절차가 시작되었거나 공정이 실행 중이 아니면 아무것도 하지 않음
        if not self.is_running:
            return

        self.log_message.emit("Process", "비상 정지 요청됨. 안전 종료 시퀀스를 시작합니다.")

        # 1. 현재 진행 중인 모든 타이머와 상태를 정지
        self.is_running = False
        self.step_timer.stop()
        self.set_polling.emit(False)
            
        # ✅ 추가: 이전 병렬 작업 카운터 초기화
        self._parallel_tasks_remaining = 0

        # 2. '전체 종료' 스텝을 생성하여 비상 종료 절차로 사용
        emergency_steps = self._create_shutdown_sequence(params=self.current_params)

        # 3. 만약 생성된 종료 스텝이 없다면, 즉시 finish_process 호출
        if not emergency_steps:
            self._finish_process(was_successful=False)
            return

        # 4. 현재 공정 시퀀스를 '비상 종료 시퀀스'로 교체하고 실행
        self.process_sequence = emergency_steps
        self._current_step_idx = -1
        self.is_running = True # 비상 종료 시퀀스를 실행하기 위해 잠시 True로 설정
        self.process_status_changed.emit(True)
        self.on_step_completed()