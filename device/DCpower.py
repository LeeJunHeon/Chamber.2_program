# device/DCPower.py
from PyQt6.QtCore import QObject, QTimer, pyqtSignal as Signal, pyqtSlot as Slot
from lib.config import (
    DC_MAX_POWER, 
    DC_TOLERANCE_POWER, 
    DC_RAMP_STEP, 
    DC_MAINTAIN_STEP, 
    DC_INTERVAL_MS,
    DC_PARAM_WATT_TO_DAC,
    DC_OFFSET_WATT_TO_DAC,
)

class DCPowerController(QObject):
    """
    Faduino를 통해 DC Power Supply를 비동기적으로 제어하는 클래스.
    - 직접 시리얼 포트에 연결하지 않고, FaduinoController로 명령을 보냅니다.
    - FaduinoController로부터 현재 전력 값을 받아 목표치에 맞게 DAC 값을 자동 조절합니다.
    """
    # Faduino 컨트롤러로 보낼 명령 신호 (예: "DA 1234")
    send_dc_power_value  = Signal(int)
    send_dc_power_value_unverified = Signal(int)

    # UI 로깅을 위한 상태 메시지 신호
    status_message = Signal(str, str)

    # 공정 컨트롤러에게 목표 파워에 도달했음을 알리는 신호
    target_reached = Signal()
    target_failed = Signal(str, str)
    power_off_finished = Signal()   # 출력 off 완료

    # Faduino 상태 읽기 요청 시그널
    request_status_read  = Signal()

    # 자신의 실행 상태를 알리는 신호
    state_changed = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 제어 상태 변수
        self.state = "IDLE"  # "IDLE", "RAMPING_UP", "MAINTAINING"
        self._is_running = False
        
        # 목표 및 현재 상태 값
        self.target_power = 0.0
        self.current_power = 0.0
        self.current_dac_value = 0

        # 주기적인 제어를 위한 QTimer
        self.control_timer = QTimer(self)
        self.control_timer.setInterval(DC_INTERVAL_MS)
        self.control_timer.timeout.connect(self._on_timer_tick)

    def _power_to_dac(self, power_watt: float) -> int:
        """
        목표 DC 파워(Watt)를 사용자의 실험 데이터 기반으로 보정된 DAC 값으로 변환합니다.
        수식: DAC = param * Watts + offset
        """
        if power_watt <= 0:
            return 0
        
        # 수식을 이용해 FADUINO로 보낼 DAC 값 계산
        dac_float = (DC_PARAM_WATT_TO_DAC * power_watt) + DC_OFFSET_WATT_TO_DAC
        
        return int(round(dac_float))

    @Slot(float)
    def start_process(self, target_power: float):
        """공정 시작 슬롯. 목표 파워로 초기 설정 후 보정 상태로 전환합니다."""
        if self._is_running:
            self.status_message.emit("DCpower", "경고: DC 파워가 이미 동작 중입니다.")
            return

        self.target_power = min(target_power, DC_MAX_POWER)
        self.status_message.emit("DCpower", f"프로세스 시작. 목표 파워 {self.target_power}W로 설정 및 보정 시작.")
        
        # 목표 파워에 해당하는 DAC 값을 계산하여 초기값으로 설정
        self.current_dac_value = self._power_to_dac(self.target_power)
        self.send_dc_power_value.emit(self.current_dac_value)
        
        # 보정(RAMPING_UP) 상태로 전환하고 타이머 시작
        self._is_running = True
        self.state_changed.emit(True)
        self.state = "RAMPING_UP"
        self.control_timer.start()

    @Slot()
    def stop_process(self):
        """공정 중지 슬롯. 제어 루프를 멈추고 출력을 0으로 설정합니다."""
        if not self._is_running:
            self.send_dc_power_value_unverified.emit(0)
            self.status_message.emit("DCpower", "출력 OFF. 대기 상태로 전환합니다.")
            self.power_off_finished.emit()
            return

        self.status_message.emit("DCpower", "정지 신호 수신됨.")
        self._is_running = False
        self.state_changed.emit(False)
        self.state = "IDLE"
        self.control_timer.stop()
        
        # 파워 출력을 즉시 0으로 설정
        self.current_dac_value = 0
        self.send_dc_power_value_unverified.emit(self.current_dac_value)
        self.status_message.emit("DCpower", "출력 OFF. 대기 상태로 전환합니다.")
        self.power_off_finished.emit()

    def _on_timer_tick(self):
        """타이머에 의해 주기적으로 호출되어 상태에 맞는 제어 작업을 수행합니다."""
        if not self._is_running:
            self.control_timer.stop()
            return
        self.request_status_read.emit()

    @Slot(float, float, float)
    def update_measurements(self, power: float, voltage: float, current: float):
        """Faduino로부터 피드백을 받았을 때 모든 제어 로직을 처리 (핵심 변경)"""
        if not self._is_running:
            return
            
        self.current_power = power if power is not None else 0.0
        self.status_message.emit("DCpower", f"피드백 수신 - Power: {power:.2f}W, Voltage: {voltage:.2f}V, Current: {current:.2f}A")
        
        power_difference = self.target_power - self.current_power

        # 1. 파워 상승(RAMPING_UP) 상태
        if self.state == "RAMPING_UP":
            # 목표 파워에 도달했는지 확인
            if abs(power_difference) <= DC_TOLERANCE_POWER:
                self.status_message.emit("DCpower", f"목표 파워 {self.target_power}W 도달. 파워 유지를 시작합니다.")
                self.state = "MAINTAINING"
                self.target_reached.emit()  # 공정 컨트롤러에 다음 단계 진행을 알림
                return

            adjustment = DC_RAMP_STEP if power_difference > 0 else -DC_RAMP_STEP
            self.current_dac_value += adjustment
            self.send_dc_power_value.emit(self.current_dac_value)
            self.status_message.emit("DCpower", f"파워 조정... Target DAC: {self.current_dac_value}")

        # 2. 파워 유지(MAINTAINING) 상태
        elif self.state == "MAINTAINING":
            # 파워가 허용 오차를 벗어났는지 확인
            if abs(power_difference) > DC_TOLERANCE_POWER:
                # 미세 조정을 통해 파워를 다시 목표치로 맞춤
                adjustment = DC_MAINTAIN_STEP if power_difference > 0 else -DC_MAINTAIN_STEP
                self.current_dac_value += adjustment
                self.current_dac_value = max(0, min(self.current_dac_value, 4095))
                self.send_dc_power_value.emit(self.current_dac_value)
                self.status_message.emit("DCpower", f"파워 유지 보정... Power: {self.current_power:.2f}W, DAC: {self.current_dac_value}")
