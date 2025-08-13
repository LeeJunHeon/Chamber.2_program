# Faduino.py

import serial
import time
from PyQt6.QtCore import QObject, QTimer, QMutex, QThread, pyqtSignal as Signal, pyqtSlot as Slot
from lib.config import (
    FADUINO_PORT, FADUINO_BAUD, BUTTON_TO_PIN,
    RF_PARAM_ADC_TO_WATT, RF_OFFSET_ADC_TO_WATT,
    DC_PARAM_ADC_TO_VOLT, DC_OFFSET_ADC_TO_VOLT,
    DC_PARAM_ADC_TO_AMP, DC_OFFSET_ADC_TO_AMP,
    ADC_FULL_SCALE, ADC_INPUT_VOLT, RF_WATT_PER_VOLT,
    DAC_FULL_SCALE
)

class FaduinoController(QObject):
    # UI/상위 컨트롤러용 시그널
    status_message = Signal(str, str)
    rf_power_updated = Signal(float, float)
    dc_power_updated = Signal(float, float, float)
    command_confirmed = Signal(str)
    command_failed = Signal(str, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.serial_faduino = None
        self.serial_lock = QMutex()

        self.expected_relay_mask = 0 # 릴레이의 예상 상태를 정수 마스크로 저장
        self._is_first_poll = True

        # (추가) 각 파워 컨트롤러의 활성 상태를 저장할 플래그
        self.is_rf_active = False
        self.is_dc_active = False

        self.rf_forward = 0.0
        self.rf_reflected = 0.0
        self.dc_voltage = 0.0
        self.dc_current = 0.0

        # 폴링 주기를 1초로 변경
        self.polling_timer = QTimer(self)
        self.polling_timer.setInterval(1000)
        self.polling_timer.timeout.connect(self.force_status_read)

    def connect_faduino(self):
        """프로그램 시작 시 시리얼 포트 연결 시도"""
        try:
            self.serial_faduino = serial.Serial(FADUINO_PORT, FADUINO_BAUD, timeout=0.1)
            self.status_message.emit("Faduino", f"{FADUINO_PORT} 연결 성공")
            QThread.msleep(100) # 짧은 대기
            if self.serial_faduino.in_waiting > 0: self.serial_faduino.read_all()
            self.serial_faduino.reset_input_buffer()
            self.serial_faduino.reset_output_buffer()
            self.status_message.emit("Faduino", "입출력 버퍼를 리셋했습니다.")
            return True
        except Exception as e:
            self.status_message.emit("Faduino", f"{FADUINO_PORT} 연결 실패: {e}")
            return False
        
    @Slot(bool)
    def set_process_status(self, should_poll: bool):
        """'ProcessController'의 방송을 수신하여 자신의 스레드에서 폴링을 제어"""
        if should_poll:
            self.start_polling()
        else:
            self.stop_polling()

    # [신규] 폴링을 시작하는 헬퍼 메소드
    def start_polling(self):
        if not self.polling_timer.isActive():
            self.status_message.emit("Faduino", "공정 감시 폴링 시작")
            self.polling_timer.start()

    # [신규] 폴링을 중지하는 헬퍼 메소드
    def stop_polling(self):
        if self.polling_timer.isActive():
            self.polling_timer.stop()
            self.status_message.emit("Faduino", "공정 감시 폴링 중지")

    # '상태 읽기' 요청을 처리하는 슬롯
    @Slot()
    def force_status_read(self):
        """'S' 명령으로 상태를 요청하고 응답을 파싱 (DCPowerController 또는 폴링 타이머에 의해 호출)"""
        self.send_command("S\n", verify=False)
        self._read_and_parse_status()

    def send_command(self, cmd, verify=True):
        """
        명령어를 전송하고, verify=True일 경우 'OK' 응답을 기다림.
        성공 시 True, 실패(타임아웃) 시 False 반환.
        """
        if not self.serial_faduino:
            self.status_message.emit("Faduino", "시리얼 연결 안 됨, 명령 전송 실패")
            return False
        
        self.serial_lock.lock()
        is_polling_active = self.polling_timer.isActive()
        if verify and is_polling_active:  # 검증이 필요한 명령('D', 'W')이면 폴링 잠시 중지
            self.polling_timer.stop()
            
        try:
            self.serial_faduino.reset_input_buffer()
            self.serial_faduino.write(cmd.encode('ascii'))

            if not verify:
                return True # 검증이 필요 없는 명령(S)은 바로 성공 처리

            # 'OK' 응답 대기 (최대 0.5초)
            start_time = time.time()
            while time.time() - start_time < 0.5:
                if self.serial_faduino.in_waiting > 0:
                    response = self.serial_faduino.readline().decode('ascii', errors='ignore').strip()
                    if response == "OK":
                        self.status_message.emit("Faduino", f"명령 성공: {cmd.strip()}")
                        return True
            
            self.status_message.emit("Faduino", f"응답 시간 초과: {cmd.strip()}")
            return False

        except Exception as e:
            self.status_message.emit("Faduino", f"명령 전송 실패: {e}")
            return False
        finally:
            if verify and is_polling_active:
                self.polling_timer.start()
            self.serial_lock.unlock()
            time.sleep(1) # 명령 전송 후 1초 대기

    @Slot(int, bool)
    def set_relay(self, pin, state):
        cmd = f"R,{pin},{1 if state else 0}\n"

        # 명령 성공시 예상 상태 업데이트
        if self.send_command(cmd):
            if state:
                self.expected_relay_mask |= (1 << pin)
            else:
                self.expected_relay_mask &= ~(1 << pin)

            self.command_confirmed.emit(cmd.strip())
        else:
            self.command_failed.emit("Faduino", f"Relay 제어 명령({cmd.strip()}) 실패")

    @Slot(str, bool)
    def handle_named_command(self, name, state):
        if name not in BUTTON_TO_PIN:
            self.status_message.emit("Faduino", f"알 수 없는 버튼명: {name}")
            return
        pin = BUTTON_TO_PIN[name]
        self.set_relay(pin, state)

    @Slot(int)
    def set_rf_power(self, value):
        v = self._clamp_dac(value)
        cmd = f"W,{v}\n"
        self.send_command(cmd)

    @Slot(int)
    def set_dc_power(self, value):
        v = self._clamp_dac(value)
        cmd = f"D,{v}\n"
        self.send_command(cmd)

    # power off 는 검증 없이
    @Slot(int)
    def set_dc_power_unverified(self, value):
        """'OK' 응답을 기다리지 않고 DC 파워를 설정합니다."""
        v = self._clamp_dac(value)
        cmd = f"D,{v}\n"
        self.send_command(cmd, verify=False)

    @Slot(int)
    def set_rf_power_unverified(self, value):
        """'OK' 응답을 기다리지 않고 RF 파워를 설정합니다."""
        v = self._clamp_dac(value)
        cmd = f"W,{v}\n"
        self.send_command(cmd, verify=False)

    def _clamp_dac(self, value: int) -> int:
        try:
            v = int(round(value))
        except Exception:
            v = 0
        if v < 0: v = 0
        if v > DAC_FULL_SCALE: v = DAC_FULL_SCALE
        return v
    
    # === pin 상태 읽기 ===
    @Slot()
    def force_pin_read(self):
        """'P' 명령으로 릴레이 마스크만 읽어옵니다."""
        self.send_command("P\n", verify=False)
        self._read_and_parse_pin()

    def _read_and_parse_pin(self):
        if not self.serial_faduino:
            return
        try:
            start = time.time()
            while time.time() - start < 0.5:
                if self.serial_faduino.in_waiting > 0:
                    line = self.serial_faduino.readline().decode('ascii', errors='ignore').strip()
                    if not line.startswith("OK"):
                        continue
                    parts = line.split(',')   # ["OK", relay_mask]
                    if len(parts) != 2:
                        continue

                    relay_mask = int(parts[1])

                    # 기존 기대 마스크 검증 로직 유지
                    if self._is_first_poll:
                        self.expected_relay_mask = relay_mask
                        self._is_first_poll = False
                        self.status_message.emit("Faduino", f"초기 릴레이 상태 동기화 완료: {relay_mask}")
                    elif relay_mask != self.expected_relay_mask:
                        msg = f"릴레이 상태 불일치! 예상: {self.expected_relay_mask}, 실제: {relay_mask}"
                        self.status_message.emit("Faduino(경고)", msg)
                        self.command_failed.emit("Faduino", f"Relay 상태 확인 {msg}")
                    return
        except Exception as e:
            self.status_message.emit("Faduino", f"핀 상태 파싱 오류: {e}")
    # === pin 상태 읽기 ===

    # === RF power 읽기 ===
    @Slot()
    def force_rf_read(self):
        """'r' 명령으로 RF(Forward/Reflected)만 빠르게 읽습니다."""
        self.send_command("r\n", verify=False)
        self._read_and_parse_rf()

    def _read_and_parse_rf(self):
        if not self.serial_faduino:
            return
        try:
            start = time.time()
            while time.time() - start < 0.5:
                if self.serial_faduino.in_waiting > 0:
                    line = self.serial_faduino.readline().decode('ascii', errors='ignore').strip()
                    if not line.startswith("OK"):
                        continue
                    parts = line.split(',')   # ["OK", rf_for_raw, rf_ref_raw]
                    if len(parts) != 3:
                        continue

                    if self.is_rf_active:
                        rf_for_raw = float(parts[1])
                        rf_ref_raw = float(parts[2])

                        # ADC → Watt 변환 (기존 스케일식 그대로)
                        self.rf_forward = max(0.0, (RF_PARAM_ADC_TO_WATT * rf_for_raw) + RF_OFFSET_ADC_TO_WATT)
                        rf_ref_voltage_at_adc = (rf_ref_raw / ADC_FULL_SCALE) * ADC_INPUT_VOLT
                        self.rf_reflected = max(0.0, rf_ref_voltage_at_adc * RF_WATT_PER_VOLT)

                        self.rf_power_updated.emit(self.rf_forward, self.rf_reflected)
                    return
        except Exception as e:
            self.status_message.emit("Faduino", f"RF 상태 파싱 오류: {e}")
    # === RF power 읽기 ===

    # === DC power 읽기 ===
    @Slot()
    def force_dc_read(self):
        """'d' 명령으로 DC(Voltage/Current)만 빠르게 읽습니다."""
        self.send_command("d\n", verify=False)
        self._read_and_parse_dc()

    def _read_and_parse_dc(self):
        if not self.serial_faduino:
            return
        try:
            start = time.time()
            while time.time() - start < 0.5:
                if self.serial_faduino.in_waiting > 0:
                    line = self.serial_faduino.readline().decode('ascii', errors='ignore').strip()
                    if not line.startswith("OK"):
                        continue
                    parts = line.split(',')   # ["OK", dc_v_raw, dc_c_raw]
                    if len(parts) != 3:
                        continue

                    if self.is_dc_active:
                        dc_v_raw = float(parts[1])
                        dc_c_raw = float(parts[2])

                        # ADC → 물리값 변환 (기존 스케일식 그대로)
                        self.dc_voltage = max(0.0, (DC_PARAM_ADC_TO_VOLT * dc_v_raw) + DC_OFFSET_ADC_TO_VOLT)
                        self.dc_current = max(0.0, (DC_PARAM_ADC_TO_AMP * dc_c_raw) + DC_OFFSET_ADC_TO_AMP)
                        self.dc_power   = self.dc_voltage * self.dc_current

                        self.dc_power_updated.emit(self.dc_power, self.dc_voltage, self.dc_current)
                    return
        except Exception as e:
            self.status_message.emit("Faduino", f"DC 상태 파싱 오류: {e}")
    # === DC power 읽기 ===

    def _read_and_parse_status(self):
        """S(상태) 명령에 대한 응답을 파싱"""
        if not self.serial_faduino:
            return
        
        try:
            start_time = time.time()
            # 버퍼에 있는 모든 라인을 읽어 처리
            while time.time() - start_time < 0.5:
                if self.serial_faduino.in_waiting > 0:
                    line = self.serial_faduino.readline().decode('ascii', errors='ignore').strip()
                    if not line.startswith("OK"): # 'OK'로 시작하는 상태 응답만 처리
                        continue
                    
                    parts = line.split(',') # 예: "OK,마스크,RF정,RF반,DC전압,DC전류"
                    if len(parts) == 6:
                        relay_mask = int(parts[1])
                        if self._is_first_poll:
                            self.expected_relay_mask = relay_mask
                            self._is_first_poll = False
                            self.status_message.emit("Faduino", f"초기 릴레이 상태 동기화 완료: {relay_mask}")
                        elif relay_mask != self.expected_relay_mask:
                            msg = f"릴레이 상태 불일치! 예상: {self.expected_relay_mask}, 실제: {relay_mask}"
                            self.status_message.emit("Faduino(경고)", msg)
                            self.command_failed.emit("Faduino", f"Relay 상태 확인 {msg}")
                            return
                        #self.status_message.emit("Faduino", "릴레이 모두 일치")

                        if self.is_rf_active:
                            rf_for_raw = float(parts[2])
                            rf_ref_raw = float(parts[3])

                            # --- RFpower 스케일 변환 ---
                            # 수식: Watts = param * ADC_raw + offset
                            # Forward: ADC → W
                            self.rf_forward = max(0.0, (RF_PARAM_ADC_TO_WATT * rf_for_raw) + RF_OFFSET_ADC_TO_WATT)

                            # Reflected: (ADC → Volt) → W
                            rf_ref_voltage_at_adc = (rf_ref_raw / ADC_FULL_SCALE) * ADC_INPUT_VOLT
                            self.rf_reflected = max(0.0, rf_ref_voltage_at_adc * RF_WATT_PER_VOLT)

                            # 변환된 값을 신호로 보냄
                            self.rf_power_updated.emit(self.rf_forward, self.rf_reflected)

                        if self.is_dc_active:
                            dc_v_raw = float(parts[4])
                            dc_c_raw = float(parts[5])

                            # --- DCpower 스케일 변환 ---
                            # 1. DC Voltage 계산: 데이터 기반 보정 수식 적용
                            # 수식: Voltage = param * ADC_v_raw + offset
                            self.dc_voltage = max(0.0, (DC_PARAM_ADC_TO_VOLT * dc_v_raw) + DC_OFFSET_ADC_TO_VOLT)

                            # 2. DC Current 계산: 데이터 기반 보정 수식 적용 (사용자 아이디어 반영)
                            # 수식: Current = param * ADC_c_raw + offset
                            self.dc_current = max(0.0, (DC_PARAM_ADC_TO_AMP * dc_c_raw) + DC_OFFSET_ADC_TO_AMP)
                            
                            # 3. 최종 DC Power는 보정된 전압과 전류를 곱하여 계산
                            self.dc_power = self.dc_voltage * self.dc_current

                            # dc_power_updated 시그널에 새로 계산된 파워, 전압, 전류를 함께 보냄
                            self.dc_power_updated.emit(self.dc_power, self.dc_voltage, self.dc_current)

                        return # 한줄 처리후 종료

        except Exception as e:
            self.status_message.emit("Faduino", f"응답 파싱 오류: {e}")

    # (추가) RF 컨트롤러의 상태 변경을 수신할 슬롯
    @Slot(bool)
    def on_rf_state_changed(self, is_active: bool):
        self.is_rf_active = is_active
        self.status_message.emit("Faduino", f"RF 컨트롤러 상태 감지: {'활성' if is_active else '비활성'}")

    # (추가) DC 컨트롤러의 상태 변경을 수신할 슬롯
    @Slot(bool)
    def on_dc_state_changed(self, is_active: bool):
        self.is_dc_active = is_active
        self.status_message.emit("Faduino", f"DC 컨트롤러 상태 감지: {'활성' if is_active else '비활성'}")

    @Slot()
    def cleanup(self):
        # 폴링 중지 및 내부 플래그
        self.stop_polling()  # 로그 포함해서 타이머 정지

        if not (self.serial_faduino and self.serial_faduino.is_open):
            return
        
        try:
            # --- 1) 모든 릴레이 OFF (검증하며 끄기) ---
            failed = []
            for pin in range(8):  # 0~7
                ok = self.send_command(f"R,{pin},0\n", verify=True)
                if not ok:
                    failed.append(pin)

            # --- 2) 최종 상태 확인 (S 읽어서 마스크 0인지 확인) ---
            self.send_command("S\n", verify=False)
            self._read_and_parse_status()  # 여기서 마스크를 읽고, 필요하면 로그로 경고가 떠요.

            # (선택) 남은 핀 재시도
            for pin in failed:
                self.send_command(f"R,{pin},0\n", verify=True)


            # --- 2) 아날로그 출력도 0으로 ---
            self.send_command("W,0\n", verify=False)  # RF 0
            self.send_command("D,0\n", verify=False)  # DC 0

            # --- 3) 내부 상태 동기화 초기화 ---
            self.expected_relay_mask = 0
            self._is_first_poll = True  # 다음 연결 때 하드웨어 마스크 재동기화

            # --- 4) 잠깐 대기 후 버퍼 정리(선택) ---
            QThread.msleep(50)
            try:
                self.serial_faduino.reset_input_buffer()
                self.serial_faduino.reset_output_buffer()
            except Exception:
                pass

        finally:
            try:
                self.serial_faduino.close()
            finally:
                self.serial_faduino = None
                self.status_message.emit("Faduino", "연결 종료")

