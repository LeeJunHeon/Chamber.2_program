# RFpower.py (단순화된 전체 코드)
import time
from PyQt6.QtCore import QObject, QTimer, pyqtSignal as Signal, pyqtSlot as Slot
from lib.config import (
    RF_MAX_POWER, 
    RF_RAMP_STEP, 
    RF_MAINTAIN_STEP,
    RF_TOLERANCE_POWER,
    RF_PARAM_WATT_TO_DAC,
    RF_OFFSET_WATT_TO_DAC,
)

class RFPowerController(QObject):
    # Faduino로 보낼 신호
    send_rf_power_value = Signal(int)
    send_rf_power_value_unverified = Signal(int)
    request_status_read = Signal()

    # UI 및 ProcessController로 보낼 신호
    update_rf_status_display = Signal(float, float)
    status_message = Signal(str, str)
    target_reached = Signal()
    target_failed = Signal(str, str)
    power_off_finished = Signal()

    # 자신의 실행 상태를 알리는 신호
    state_changed = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.target_power = 0.0
        self._is_running = False
        self._is_ramping_down = False
        self.current_power_step = 0.0

        # RFpower.py - __init__ 내부
        self._hold_dac = 0                 # 도달 순간의 DAC(유지 기준)
        self._last_sent_dac = None         # 마지막 전송 DAC(중복 전송 방지)
        self._maintain_need_consecutive = 2  # 오차가 연속 2회 넘었을 때만 보정(필요시 조정)
        self._maintain_count = 0
        self._dac_deadband = 2             # 2카운트 이하는 전송 생략
        self._rampdown_dac = 0

        # 상태 머신
        self.state = "IDLE"
        self.previous_state = "IDLE"
        self.ref_p_wait_start_time = None
        
        # 제어 타이머
        self.control_timer = QTimer(self)
        self.control_timer.setInterval(1000)
        self.control_timer.timeout.connect(self._on_timer_tick)

        # (추가) Ramp-down 전용 타이머를 인스턴스 변수로 선언
        self.ramp_down_timer = QTimer(self)
        self.ramp_down_timer.setInterval(50)  # 50ms 간격
        self.ramp_down_timer.timeout.connect(self._ramp_down_step)

    def _power_to_dac(self, power_watt: float) -> int:
        """
        목표 파워(Watt)를 보정된 DAC 값으로 변환합니다.
        수식: DAC = param * Watts + offset
        """
        if power_watt <= 0:
            return 0
        
        # 수식을 이용해 DAC 값 계산
        dac_float = (RF_PARAM_WATT_TO_DAC * power_watt) + RF_OFFSET_WATT_TO_DAC
        
        return int(round(dac_float))

    @Slot(float) 
    def start_process(self, target_power: float):
        if self._is_running:
            return
        
        self.target_power = min(target_power, RF_MAX_POWER)
        self.current_power_step = 30.0
        self._is_running = True
        self.state_changed.emit(True)

        self.state = "RAMPING_UP"
        self.control_timer.start()
        self.status_message.emit("RFpower", f"프로세스 시작. 목표: {self.target_power}W")

    @Slot()
    def stop_process(self):
        if self._is_ramping_down or not self._is_running: return
        self.status_message.emit("RFpower", "정지 신호 수신됨.")
        self._is_running = False
        self.state_changed.emit(False)

        self.state = "IDLE"
        self.control_timer.stop()
        self.ramp_down()

    def _on_timer_tick(self):
        if not self._is_running:
            self.control_timer.stop()
            return
        self.request_status_read.emit()

    @Slot(float, float)
    def update_measurements(self, for_p, ref_p):
        if not self._is_running: return
        self.update_rf_status_display.emit(for_p, ref_p)

        if ref_p is not None and ref_p > 3.0:
            if self.state != "REF_P_WAITING":
                self.previous_state = self.state
                self.state = "REF_P_WAITING"
                self.ref_p_wait_start_time = time.time()
                self.status_message.emit("RFpower(대기)", f"반사파({ref_p:.1f}W) 안정화 대기를 시작합니다. (최대 60초)")
            
            if time.time() - (self.ref_p_wait_start_time or 0) > 60:
                reason = "반사파 안정화 시간(60초) 초과"
                self.status_message.emit("RFpower(오류)", f"{reason}. 즉시 중단합니다.")
                # [수정] 실패 신호를 보내고 stop_process 호출
                self.target_failed.emit("RFpower", reason)
                self.stop_process()
            return

        if self.state == "REF_P_WAITING":
            self.status_message.emit("RFpower(정보)", f"반사파 안정화 완료 ({ref_p:.1f}W). 공정을 재개합니다.")
            self.state = self.previous_state
            self.ref_p_wait_start_time = None

        if self.state == "RAMPING_UP":
            power_difference = self.target_power - for_p

            if abs(power_difference) <= RF_TOLERANCE_POWER:
                self.status_message.emit("RFpower", f"{self.target_power}W 도달. 파워 유지 시작")
                # ★ 추가: 방금까지 보낸 DAC을 유지 기준으로 기록
                self._hold_dac = self._last_sent_dac if self._last_sent_dac is not None else self._power_to_dac(self.target_power)
                self.state = "MAINTAINING"
                self.target_reached.emit()
                return

            if power_difference > 0: # 현재 파워가 목표보다 낮을 경우 -> 파워 상승
                self.current_power_step = self.current_power_step + RF_RAMP_STEP
                self.current_power_step = min(self.current_power_step, self.target_power)
            else: # 현재 파워가 목표보다 높을 경우 (오버슈팅) -> 파워 하강
                # 이전 스텝 값에서 계속해서 RAMP_STEP만큼 낮춤
                self.current_power_step = self.current_power_step - RF_MAINTAIN_STEP
                # 0 미만으로 내려가지 않도록 보정
                self.current_power_step = max(0, self.current_power_step)
                self.status_message.emit("RFpower", f"목표 파워 초과. 출력 하강 시도...")

            dac_to_send = self._power_to_dac(self.current_power_step)
            self.send_rf_power_value.emit(dac_to_send)
            self._last_sent_dac = dac_to_send     # ★ 추가: 마지막 전송 DAC 저장
            self.status_message.emit("RFpower", f"Ramp-Up... 목표스텝:{self.current_power_step:.1f}W, 현재:{for_p:.1f}W (DAC:{dac_to_send})")

        elif self.state == "MAINTAINING":
            error = self.target_power - for_p

            # 오차가 허용범위 안이면: 아무 것도 하지 않고 '유지'
            if abs(error) <= RF_TOLERANCE_POWER:
                self._maintain_count = 0
                return

            # 허용오차를 연속 N회 벗어날 때만 보정(노이즈 내성)
            self._maintain_count += 1
            if self._maintain_count < self._maintain_need_consecutive:
                return
            self._maintain_count = 0

            # 목표(anchor) 기준으로 ±한 스텝만 조정 → DAC로 변환
            step_w = RF_MAINTAIN_STEP if error > 0 else -RF_MAINTAIN_STEP
            new_dac = self._power_to_dac(self.target_power + step_w)

            # 아주 작은 변화는 전송 생략(시리얼 스팸 방지)
            if (self._last_sent_dac is None) or (abs(new_dac - self._last_sent_dac) >= self._dac_deadband):
                self._hold_dac = new_dac
                self._last_sent_dac = new_dac
                self.send_rf_power_value.emit(new_dac)
                self.status_message.emit("RFpower",
                    f"유지 보정: meas={for_p:.1f}W, target={self.target_power:.1f}W → DAC {new_dac}")

    # (추가) ramp_down_step 함수를 클래스 메소드로 변경
    def _ramp_down_step(self):
        # Watt 스텝을 DAC 스텝으로 환산(최소 1카운트)
        dac_step = max(1, int(round(RF_PARAM_WATT_TO_DAC * RF_RAMP_STEP)))
        if self._rampdown_dac <= 0:
            self.ramp_down_timer.stop()
            self.send_rf_power_value_unverified.emit(0)
            self.update_rf_status_display.emit(0.0, 0.0)
            self.status_message.emit("RFpower", "RF 파워 ramp-down 완료")
            self.power_off_finished.emit()
            self._is_ramping_down = False
            return

        # ★ 단조 감소 강제: 현재보다 절대 커지지 않게
        self._rampdown_dac = max(0, self._rampdown_dac - dac_step)
        self._last_sent_dac = self._rampdown_dac
        self.send_rf_power_value_unverified.emit(self._rampdown_dac)
    
    # (수정) ramp_down 함수를 간결하게 변경
    def ramp_down(self):
        if self._is_ramping_down: return
        self._is_ramping_down = True
        self.status_message.emit("RFpower", "RF 파워 ramp-down 시작")
        
        # 마지막으로 보낸 DAC을 시작점으로. 없으면 현재 step을 DAC으로 환산
        self._rampdown_dac = self._last_sent_dac if self._last_sent_dac is not None else self._power_to_dac(self.current_power_step)

        # 미리 만들어 둔 인스턴스 타이머를 시작하기만 함
        self.ramp_down_timer.start()
