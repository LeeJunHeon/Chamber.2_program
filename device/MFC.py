# MFC.py (비동기 방식 수정본)
import serial
from PyQt6.QtCore import QObject, QThread, QTimer, pyqtSignal as Signal, pyqtSlot as Slot
# config 파일에 N2 관련 설정이 추가되었다고 가정합니다.
from lib.config import (
    MFC_PORT, 
    MFC_BAUD, 
    MFC_COMMANDS, 
    FLOW_ERROR_TOLERANCE, 
    FLOW_ERROR_MAX_COUNT,
    MFC_SCALE_FACTORS
    )

class MFCController(QObject):
    # --- 시그널 정의 (기존과 유사) ---
    status_message = Signal(str, str)
    update_flow = Signal(str, float)      # UI 업데이트용: (가스이름, 유량값)
    update_pressure = Signal(str)         # UI 업데이트용: (압력값 문자열)
    command_failed = Signal(str, str)     # 실패 시: (명령, 원인)
    command_confirmed = Signal(str)       # 성공 시: (명령)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.serial_mfc = None
        self._is_running = False
        self._is_aborted = False # 명령 중단(abort) 상태를 저장할 플래그

        # gas 안정화를 위한 변수
        self._stabilizing_channel = None
        self._stabilizing_target = 0.0
        self._pending_cmd_for_timer = None

        # [GAS 채널 3] 지원을 위해 자료구조
        self.last_setpoints = {1: 0.0, 2: 0.0, 3: 0.0}
        self.flow_error_counters = {1: 0, 2: 0, 3: 0.0}
        self.gas_map = {1: "Ar", 2: "O2", 3: "N2"}

        # [ASYNCHRONOUS] Polling을 위한 메인 타이머
        self.polling_timer = QTimer(self)
        self.polling_timer.setInterval(1000)
        self.polling_timer.timeout.connect(self._poll_status)

        # [ASYNCHRONOUS] 유량 안정화 검사를 위한 별도 타이머
        self.stabilization_timer = QTimer(self)
        self.stabilization_timer.setInterval(1000)
        self.stabilization_timer.timeout.connect(self._check_flow_stabilization)
        self.stabilization_attempts = 0

    # --- 연결 및 기본 통신 ---
    def connect_mfc_device(self):
        try:
            self.serial_mfc = serial.Serial(MFC_PORT, MFC_BAUD, timeout=1)
            self.status_message.emit("MFC", f"{MFC_PORT} 연결 성공")
            QThread.msleep(100) # 짧은 대기
            if self.serial_mfc.in_waiting > 0: self.serial_mfc.read_all()
            self.serial_mfc.reset_input_buffer()
            self.serial_mfc.reset_output_buffer()
            self.status_message.emit("MFC", "입출력 버퍼를 리셋했습니다.")
            return True
        except Exception as e:
            self.status_message.emit("MFC", f"{MFC_PORT} 연결 실패: {e}")
            self.serial_mfc = None
            return False

    def _send_command(self, cmd_str: str):
        if not self.serial_mfc: 
            self.status_message.emit("ERROR[_send_command]", "MFC 시리얼 연결 오류")
            return False
        try:
            if not cmd_str.endswith('\r'): 
                cmd_str += '\r'
            self.serial_mfc.write(cmd_str.encode('ascii'))
            self.status_message.emit("MFC > 전송", f"{cmd_str.strip()}")
            return True
        except Exception as e:
            self.status_message.emit("ERROR[_send_command]", f"MFC 명령 전송 실패: {e}"); 
            return False

    def _read_response(self):
        if not self.serial_mfc: 
            return None
        try:
            return self.serial_mfc.readline().decode('ascii').strip()
        except Exception: 
            return None

    # --- 비동기 폴링 로직 ---
    @Slot(bool)
    def set_process_status(self, should_poll: bool):
        """시그널을 통해 폴링 상태를 안전하게 제어합니다."""
        if should_poll:
            self.start_polling()
        else:
            self.stop_polling()

    @Slot()
    def start_polling(self):
        if not self.polling_timer.isActive():
            self.status_message.emit("MFC", "주기적 읽기(Polling) 시작")
            self.polling_timer.start()

    @Slot()
    def stop_polling(self):
        if self.polling_timer.isActive():
            self.polling_timer.stop()
            self.status_message.emit("MFC", "주기적 읽기(Polling) 중지")

    def _poll_status(self):
        for ch, name in self.gas_map.items():
            flow = self._execute_read_command('READ_FLOW', {'channel': ch})
            if flow and '+' in flow:
                try:
                    # 장비에서 직접 읽은 하드웨어 값
                    flow_val = float(flow.split('+')[1])

                    # [수정] UI에 표시하기 위해 원래 스케일로 복원
                    scale_factor = MFC_SCALE_FACTORS.get(ch, 1.0)
                    flow_ui_val = flow_val / scale_factor

                    # UI에는 복원된 값을, 내부 모니터링에는 실제 하드웨어 값을 사용
                    self.update_flow.emit(name, flow_ui_val)
                    self._monitor_flow(ch, flow_val)
                except (ValueError, IndexError) as e: 
                    self.status_message.emit("ERROR[_poll_status]", f"FLOW 읽기 실패 {e}")
                    pass
        
        pressure = self._execute_read_command('READ_PRESSURE')
        if pressure: 
            self.update_pressure.emit(pressure)

    # ProcessController의 stop_process 신호와 연결될 슬롯 
    @Slot()
    def abort_current_command(self):
        """진행 중인 모든 비동기 작업을 즉시 중단합니다."""
        if not self._is_aborted:
            self.status_message.emit("MFC", "상위 컨트롤러에 의해 현재 명령이 중단되었습니다.")
            self._stabilizing_channel = None
            self._stabilizing_target = 0.0
            self._pending_cmd_for_timer = None
            self._is_aborted = True
            if self.stabilization_timer.isActive(): self.stabilization_timer.stop()

    # --- 비동기 명령 처리 로직 ---
    @Slot(str, dict)
    def handle_command(self, cmd: str, params: dict):
        """[핵심] ProcessController로부터 모든 명령을 받아 처리. 즉시 반환되며, 결과는 시그널로 전달."""
        self._is_aborted = False

        # [수정] FLOW_SET 명령일 경우, 전송 전에 스케일 적용
        if cmd == 'FLOW_SET':
            channel = params.get('channel')
            original_value = float(params.get('value', 0.0))
            
            # config에서 스케일 팩터를 가져옴 (없으면 1.0)
            scale_factor = MFC_SCALE_FACTORS.get(channel, 1.0)
            scaled_value = original_value * scale_factor
            
            # params 딕셔너리의 'value'를 스케일이 적용된 값으로 교체
            params['value'] = scaled_value
            self.status_message.emit("MFC", f"Ch{channel} 유량 스케일 적용: {original_value:.1f}sccm -> 장비 전송값: {scaled_value:.2f}")

        self.current_cmd = cmd
        self.current_params = params
        self.retry_attempts = 0
        self._try_command()

    def _try_command(self):
        """명령 실행 시도. 실패 시 재시도 로직 포함."""
        if self.retry_attempts >= 3:
            self.command_failed.emit(self.current_cmd, "최대 재시도 횟수(3회) 초과")
            return

        self.retry_attempts += 1
        command_lambda = MFC_COMMANDS.get(self.current_cmd)
        if not command_lambda:
            self.command_failed.emit(self.current_cmd, "알 수 없는 명령어")
            return

        command_str = command_lambda(**self.current_params) if self.current_params else command_lambda
        self._send_command(command_str)
        # 명령 전송 후 500ms 뒤에 검증 단계 실행
        QTimer.singleShot(500, self._verify_response)

    def _parse_r69_bits(self, resp: str) -> str:
        """
        R69 응답을 상태비트 문자열(예: '1000')로 변환.
        매뉴얼 규격: 응답은 'L0' 접두사 다음에 채널별 0/1 나열.
        예) 'L01000' -> '1000'
        """
        s = (resp or "").strip()
        if s.startswith("L0"):
            payload = s[2:]
        elif s.startswith("L"):           # 방어적 처리
            payload = s[1:]
        else:
            payload = s
        # 중간 공백/문자 섞여도 안전하게 0/1만 추출
        bits = "".join(ch for ch in payload if ch in "01")
        # 최대 4채널까지만 사용
        return bits[:4]

    def _is_channel_on(self, ch: int) -> bool:
        """R69 기반 채널 ON/OFF 판정 (1-base 채널번호)."""
        resp = self._execute_read_command('READ_MFC_ON_OFF_STATUS')
        bits = self._parse_r69_bits(resp)
        return (1 <= ch <= len(bits)) and (bits[ch-1] == '1')

    def _verify_response(self):
        """[수정됨] 명령어 전송 후 응답을 검증하는 단계. 모든 검증 로직이 여기에 통합됩니다."""

        if self._is_aborted: return

        cmd, params = self.current_cmd, self.current_params

        # 각 명령어에 맞는 검증 로직 수행
        try:
            success = False
            if cmd == "FLOW_SET":
                # [수정됨] FLOW_SET 검증 로직 전체
                ch = params['channel']
                # params['value']에는 handle_command에서 변환한 '스케일링된' 값이 들어있습니다.
                scaled_value = float(params['value'])
                scale_factor = MFC_SCALE_FACTORS.get(ch, 1.0)
                expected_hw = scaled_value / scale_factor   

                read_val_str = self._execute_read_command('READ_FLOW_SET', {'channel': ch})
                
                # 1. 검증: 장비에서 읽은 값과 '스케일링된' 목표값을 비교합니다.
                if read_val_str and '+' in read_val_str and abs(float(read_val_str.split('+')[1]) - scaled_value) < 0.1:
                    # 2. 내부 저장: 유량 안정화 모니터링을 위해 '스케일링된' 값을 저장합니다.
                    self.last_setpoints[ch] = scale_factor
                    
                    # 3. UI 표시: 사용자 확인을 위해 다시 원래 값으로 변환합니다.
                    original_value = expected_hw
                    
                    self.status_message.emit("MFC < 확인", f"Ch{ch} 목표값 {original_value:.1f} sccm 설정 완료.")
                    success = True
                else:
                    self.status_message.emit("MFC(경고)", f"Ch{ch} FLOW_SET 확인 실패 (재시도 {self.retry_attempts}/3)")
                    self._try_command()
                return

            elif cmd == "FLOW_ON":
                ch = params['channel']
                if self._is_channel_on(ch):
                    self._stabilizing_channel = ch
                    self._stabilizing_target = float(self.last_setpoints.get(ch, 0.0))
                    self._pending_cmd_for_timer = "FLOW_ON"

                    self.status_message.emit("MFC < 확인", f"Ch{ch} Flow ON 확인. 유량 안정화 시작...")
                    self.stabilization_attempts = 0
                    self.stabilization_timer.start()
                    return
                else:
                    self.status_message.emit("MFC(경고)", f"Ch{ch} Flow ON 미확인 (재시도 {self.retry_attempts}/3)")
                    self._try_command()
                    return

            elif cmd == "FLOW_OFF":
                ch = params['channel']
                if not self._is_channel_on(ch):
                    self.status_message.emit("MFC < 확인", f"Ch{ch} Flow OFF 확인.")
                    success = True
                else:
                    self.status_message.emit("MFC(경고)", f"Ch{ch} Flow OFF 미확인 (재시도 {self.retry_attempts}/3)")
                    self._try_command()
                    return
            
            elif cmd in ["VALVE_CLOSE", "VALVE_OPEN"]:
                self.status_message.emit("MFC", "밸브 이동 대기 (3초)...")
                # 3초 후 밸브 위치를 확인하는 함수를 예약하고 즉시 리턴
                QTimer.singleShot(3000, self._check_valve_position)
                return # 중요: 타이머가 결과를 보고하므로 여기서 함수 종료

            elif cmd == "SP1_SET":
                val = float(params['value'])
                read_val_str = self._execute_read_command('READ_SP1_VALUE')
                if read_val_str and '+' in read_val_str and abs(float(read_val_str.split('+')[1]) - val) < 0.1:
                    self.status_message.emit("MFC < 확인", f"SP1 목표값 {val:.2f} 설정 완료.")
                    success = True

            elif cmd in ["SP1_ON", "SP4_ON"]:
                read_str = self._execute_read_command('READ_SYSTEM_STATUS')
                expected_char = '1' if cmd == "SP1_ON" else '4'
                if read_str and read_str.startswith("M") and read_str[1] == expected_char:
                    self.status_message.emit("MFC < 확인", f"{cmd} 활성화 확인.")
                    success = True

            # READ_FLOW 명령 처리 (main.py의 update_mfc_flow_ui 슬롯과 연동)
            elif cmd == "READ_FLOW":
                ch = params['channel']
                flow_str = self._execute_read_command('READ_FLOW', {'channel': ch})
                if flow_str and '+' in flow_str:
                    try:
                        flow_val = float(flow_str.split('+')[1]) 
                        scale_factor = MFC_SCALE_FACTORS.get(ch, 1.0)
                        flow_ui = flow_val / scale_factor

                        gas_name = self.gas_map.get(ch) 
                        if gas_name:
                            self.update_flow.emit(gas_name, flow_ui)
                        success = True
                    except (ValueError, IndexError):
                        success = False
                else:
                    success = False

            # READ_PRESSURE 명령 처리 (main.py의 update_mfc_pressure_ui 슬롯과 연동)
            elif cmd == "READ_PRESSURE":
                pressure_str = self._execute_read_command('READ_PRESSURE')
                if pressure_str:
                    # 1. 읽어온 문자열(pressure_str)을 그대로 사용
                    # 2. main.py의 슬롯 형식에 맞춰 (str) 시그널 발생
                    self.update_pressure.emit(pressure_str)
                    success = True
                else:
                    success = False

            elif cmd in ["MFC_ZEROING", "PS_ZEROING", "READ"]:
                self.status_message.emit("MFC < 확인", f"{cmd} 명령은 별도 확인 응답 없음. 성공으로 간주.")
                success = True

            # --- 검증 결과에 따른 후처리 ---
            if success:
                self.command_confirmed.emit(cmd)
            else:
                self.status_message.emit("MFC(경고)", f"명령({cmd}) 검증 실패 (시도 {self.retry_attempts}/3)")
                self._try_command()  # 실패 시 재시도

        except (ValueError, IndexError, TypeError) as e:
            self.status_message.emit("오류", f"검증 중 응답 파싱 오류: {e} (시도 {self.retry_attempts}/3)")
            self._try_command() # 예외 발생 시 재시도

    def _check_valve_position(self):
        """[신규] 3초 대기 후 밸브 위치를 확인하는 비동기 헬퍼 함수"""

        if self._is_aborted: return

        cmd = self.current_cmd
        success = False
        try:
            read_str = self._execute_read_command('READ_VALVE_POSITION')
            if read_str and '+' in read_str:
                pos = float(read_str.split('+')[1])
                if cmd == "VALVE_CLOSE" and pos < 1.0:
                    self.status_message.emit("MFC < 확인", "밸브 닫힘 확인.")
                    success = True
                elif cmd == "VALVE_OPEN" and pos > 99.0:
                    self.status_message.emit("MFC < 확인", "밸브 열림 확인.")
                    success = True
        
        except (ValueError, IndexError, TypeError) as e:
            self.status_message.emit("오류", f"밸브 위치 응답 파싱 오류: {e}")

        # --- 검증 결과에 따른 후처리 ---
        if success:
            self.command_confirmed.emit(cmd)
        else:
            self.status_message.emit("MFC(경고)", f"명령({cmd}) 검증 실패 (시도 {self.retry_attempts}/3)")
            self._try_command() # 실패 시 재시도

    def _check_flow_stabilization(self):
        """[비동기] 1초마다 호출되어 유량 안정화를 검사하는 함수."""

        if self._is_aborted: return

        ch = self._stabilizing_channel
        target_flow = float(self._stabilizing_target)
        pending_cmd = self._pending_cmd_for_timer or "FLOW_ON"

        if ch is None or target_flow <= 0:
            # 비정상 상태: 타이머 정리 후 실패 처리
            if self.stabilization_timer.isActive():
                self.stabilization_timer.stop()
            self.command_failed.emit(pending_cmd, "안정화 대상 채널/목표 없음")
            # 스냅샷 리셋
            self._stabilizing_channel = None
            self._stabilizing_target = 0.0
            self._pending_cmd_for_timer = None
            return

        tolerance = target_flow * FLOW_ERROR_TOLERANCE
        self.stabilization_attempts += 1

        flow_str = self._execute_read_command('READ_FLOW', {'channel': ch})
        actual_flow = -1.0
        if flow_str and '+' in flow_str:
            try: 
                actual_flow = float(flow_str.split('+')[1])
            except (ValueError, IndexError): 
                pass

        scale_factor = MFC_SCALE_FACTORS.get(ch, 1.0)
        self.status_message.emit(
            "MFC", 
            f"유량 확인 중... (목표: {target_flow:.1f} / {target_flow/scale_factor:.1f}sccm," 
            f"현재: {actual_flow:.1f} / {actual_flow/scale_factor:.1f}sccm)"
        )

        if actual_flow != -1 and abs(actual_flow - target_flow) <= tolerance:
            self.stabilization_timer.stop()
            self.status_message.emit("MFC < 확인", f"Ch{ch} 유량 안정화 완료.")
                # ▶ current_cmd 대신 스냅샷 커맨드로 보고 (변조 방지)
            self.command_confirmed.emit(pending_cmd)
            # 스냅샷 리셋
            self._stabilizing_channel = None
            self._stabilizing_target = 0.0
            self._pending_cmd_for_timer = None
        elif self.stabilization_attempts >= 30: # 30초 초과 시
            self.stabilization_timer.stop()
            self.command_failed.emit(self.current_cmd, "유량 안정화 시간 초과")
            # 스냅샷 리셋
            self._stabilizing_channel = None
            self._stabilizing_target = 0.0
            self._pending_cmd_for_timer = None

    def _execute_read_command(self, cmd_key, params=None):
        """읽기 명령을 보내고 응답을 즉시 반환하는 내부 함수."""
        read_cmd_lambda = MFC_COMMANDS.get(cmd_key)
        if not read_cmd_lambda: return None
        read_cmd_str = read_cmd_lambda(**params) if params else read_cmd_lambda
        if self._send_command(read_cmd_str):
            QThread.msleep(100) # 응답을 기다리기 위한 짧은 대기
            resp = self._read_response()
            self.status_message.emit("MFC < 응답", f"{cmd_key} => '{resp}'")
            return resp
        return None

    def _monitor_flow(self, channel, actual_flow):
        """유량 모니터링 로직 (기존과 거의 동일)"""
        target_flow = self.last_setpoints.get(channel, 0.0)
        if target_flow < 0.1: self.flow_error_counters[channel] = 0; return

        if abs(actual_flow - target_flow) > (target_flow * FLOW_ERROR_TOLERANCE):
            self.flow_error_counters[channel] += 1
            if self.flow_error_counters[channel] >= FLOW_ERROR_MAX_COUNT:
                self.status_message.emit("MFC(경고)", f"Ch{channel} 유량 불안정! (목표: {target_flow:.1f}, 현재: {actual_flow:.1f})")
                self.flow_error_counters[channel] = 0
        else:
            self.flow_error_counters[channel] = 0

    @Slot()
    def cleanup(self):
        self.polling_timer.stop()
        self.stabilization_timer.stop()
        self._stabilizing_channel = None
        self._stabilizing_target = 0.0
        self._pending_cmd_for_timer = None
        if self.serial_mfc and self.serial_mfc.is_open:
            self.serial_mfc.close()
            self.status_message.emit("MFC", "시리얼 포트를 안전하게 닫았습니다.")