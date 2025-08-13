# IG.py (MFC 스타일 단순 폴링 버전)

import time
import serial
from lib.config import IG_PORT, IG_BAUD, IG_WAIT_TIMEOUT
from PyQt6.QtCore import QTimer, QObject, pyqtSignal as Signal

class IGController(QObject):
    status_message = Signal(str, str)
    pressure_update = Signal(float)      # 현재 압력 UI/로거 업데이트
    base_pressure_reached = Signal()     # 목표 압력 도달
    base_pressure_failed = Signal(str, str)   # 실패 사유

    def __init__(self, parent=None):
        super().__init__(parent)
        self.serial_ig: serial.Serial | None = None

        # MFC와 동일한 형태의 반복 폴링 타이머 (기본 10초 주기)
        self.polling_timer = QTimer(self)
        self.polling_timer.setInterval(10_000)
        self.polling_timer.timeout.connect(self._tick)

        self._target_pressure: float = 0.0
        self._wait_start_time: float = 0.0
        self._busy: bool = False

    # ---------------- 연결/정리 ----------------
    def connect_device(self) -> bool:
        try:
            # 간단하게 블로킹 읽기 1초 (워커스레드에서 돌므로 UI는 안 막힘)
            self.serial_ig = serial.Serial(IG_PORT, IG_BAUD, timeout=1, write_timeout=1)
            self.status_message.emit("IG", f"{IG_PORT} 연결 성공")

            # 잔여 버퍼 정리
            try:
                if self.serial_ig.in_waiting:
                    self.serial_ig.read(self.serial_ig.in_waiting)
                self.serial_ig.reset_input_buffer()
                self.serial_ig.reset_output_buffer()
            except Exception:
                pass

            return True
        except Exception as e:
            self.status_message.emit("IG", f"{IG_PORT} 연결 실패: {e}")
            self.serial_ig = None
            return False

    def cleanup(self):
        try:
            self.polling_timer.stop()
        except Exception:
            pass

        if self.serial_ig and self.serial_ig.is_open:
            try:
                self.set_ig_state(False)  # IG OFF
            except Exception:
                pass
            try:
                self.serial_ig.close()
            except Exception:
                pass
            self.serial_ig = None
            self.status_message.emit("IG", "연결 종료됨")

    # ---------------- 외부 API ----------------
    def start_wait_for_pressure(self, base_pressure: float, interval_ms: int = 10_000):
        """총 대기시간 내 목표 압력 도달할 때까지 주기 폴링 시작."""
        if self.polling_timer.isActive():
            self.status_message.emit("IG", "이미 압력 대기 프로세스가 실행 중입니다.")
            return

        # 포트가 안 열려 있으면 자동 연결
        if not self.serial_ig or not self.serial_ig.is_open:
            if not self.connect_device():
                self.base_pressure_failed.emit("IG", "포트 연결 실패")
                return

        self.status_message.emit("IG", "Base Pressure 대기를 시작합니다.")
        self._target_pressure = float(base_pressure)
        self._wait_start_time = time.time()

        # IG ON 시도 (ACK 실패해도 다음 주기에 계속 시도하지 않음 — 단순화)
        if not self.set_ig_state(True):
            self.status_message.emit("IG", "장비 켜기 실패")
            self.base_pressure_failed.emit("IG", "장비 켜기 실패")
            return

        # 주기 설정 및 시작 (첫 읽기는 interval 후)
        self.polling_timer.setInterval(int(interval_ms))
        self.polling_timer.start()

    # ---------------- 내부 로직 ----------------
    def _tick(self):
        """주기마다 한 번 읽고, 성공/타임아웃만 판단."""
        if self.serial_ig is None or self._busy:
            return
        self._busy = True
        try:
            # 1) 총 대기시간 초과?
            if time.time() - self._wait_start_time > IG_WAIT_TIMEOUT:
                self.status_message.emit("IG", f"시간 초과({IG_WAIT_TIMEOUT}초): 목표 압력 미도달")
                self.base_pressure_failed.emit("IG", "Timeout")
                self.polling_timer.stop()
                self.cleanup()
                return

            # 2) 한 번 읽어서 판단
            pressure = self.read_pressure()
            if pressure is not None:
                self.pressure_update.emit(pressure)
                self.status_message.emit("IG",
                    f"현재 압력: {pressure:.3e} Torr (목표: {self._target_pressure:.3e} Torr)")
                if pressure <= self._target_pressure:
                    self.status_message.emit("IG", "목표 압력 도달")
                    self.base_pressure_reached.emit()
                    self.polling_timer.stop()
                    self.cleanup()
                    return
            # 읽기 실패/미달이면 다음 주기에 자동 재시도 (아무것도 안 함)
        finally:
            self._busy = False

    # ---------------- 시리얼 명령 ----------------
    def read_pressure(self) -> float | None:
        """RDI 한 번 요청 후 한 줄 읽어 float 변환. 실패 시 None."""
        if not self.serial_ig:
            return None
        try:
            self.serial_ig.write(b"RDI\r")
            line = self.serial_ig.readline().decode(errors="ignore").strip()
            if not line:
                return None
            # 비표준 표기 보정
            cleaned = line.lower().replace("x10e", "e")
            return float(cleaned)
        except Exception as e:
            self.status_message.emit("IG", f"압력 읽기 실패: {e}")
            return None

    def set_ig_state(self, state: bool) -> bool:
        """SIG 1/0 간단 제어. OK 응답이면 True."""
        if not self.serial_ig:
            return False
        try:
            cmd = b"SIG 1\r" if state else b"SIG 0\r"
            self.serial_ig.write(cmd)
            ack = self.serial_ig.readline().decode(errors="ignore").strip()
            return ack.upper() == "OK"
        except Exception:
            return False
