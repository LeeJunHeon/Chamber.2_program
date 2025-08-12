# IG.py

import time
import serial
from PyQt6.QtCore import QTimer, QObject, QThread, pyqtSignal as Signal, pyqtSlot as Slot
from lib.config import (
    IG_PORT,
    IG_BAUD,
    IG_WAIT_TIMEOUT,      # 전체 대기 제한(초) - 이 시간이 넘으면 무조건 실패로 종료
    MAX_READ_RETRIES,     # read_pressure() 내부에서의 재시도 횟수(한 사이클 내)
    RESPONSE_TIMEOUT_SEC, # 한 번의 응답 대기 데드라인(초)
)

class IGController(QObject):
    """
    Ion Gauge(IG) 압력 대기/읽기를 비동기적으로 수행하는 컨트롤러.
    - 10초 대기 → 압력 읽기 → (목표 미달이면) 다시 10초 대기 …를 반복
    - 응답이 아예 없을 때는 '사이클 단위'로 최대 N회(기본 3사이클)까지 재시도 후 종료
    - 전체 대기 시간(IG_WAIT_TIMEOUT)을 초과하면 즉시 실패 처리
    - 읽기는 논블로킹 방식(Serial timeout=0, in_waiting 기반)으로 구현해 UI 프리즈 방지
    """

    # --- 시그널 ---
    status_message = Signal(str, str)        # (source, message)
    pressure_update = Signal(float)          # 현재 압력 값을 Data_logger 등에 전달
    base_pressure_reached = Signal()         # 목표 압력 도달 시 성공 신호
    base_pressure_failed = Signal(str, str)  # (source, reason) 실패 신호

    def __init__(self, parent=None):
        super().__init__(parent)
        self.serial_ig: serial.Serial | None = None

        # [중요] '10초 대기 → 읽기' 한 사이클을 관리하는 1회성 타이머
        self.cycle_timer = QTimer(self)
        self.cycle_timer.setSingleShot(True)
        self.cycle_timer.timeout.connect(self._read_and_evaluate)

        self._target_pressure: float = 0.0
        self._wait_start_time: float = 0.0    # 전체 타이머 시작시각(IG_WAIT_TIMEOUT 체크용)
        self._busy: bool = False              # _read_and_evaluate 중복 진입 가드

        # [추가] '응답 없음' 사이클 누적 카운터와 한계치(기본 3사이클까지)
        self._no_response_cycles: int = 0
        self._no_response_cycle_limit: int = 3

    # -------------------------------------------------------------------------
    # 연결/정리
    # -------------------------------------------------------------------------
    def connect_device(self) -> bool:
        """
        장비에 연결을 시도. 성공 시 True.
        - timeout=0(논블로킹)으로 열고, 응답 대기는 우리가 직접 데드라인 관리
        - reset_input_buffer()는 일부 환경에서 블로킹 이슈가 있어 가급적 회피하고
          in_waiting 기반으로 남은 데이터를 수동 플러시
        """
        try:
            self.serial_ig = serial.Serial(
                IG_PORT,
                IG_BAUD,
                timeout=0,        # 논블로킹
                write_timeout=1,  # 쓰기 타임아웃(보호)
            )
            self.status_message.emit("IG", f"{IG_PORT} 연결 성공 (non-blocking)")

            # 잔여 수신 버퍼 수동 플러시
            try:
                if self.serial_ig.in_waiting:
                    self.serial_ig.read(self.serial_ig.in_waiting)
            except Exception:
                pass
            return True
        except Exception as e:
            self.status_message.emit("IG", f"{IG_PORT} 연결 실패: {e}")
            self.serial_ig = None
            return False

    def cleanup(self):
        """
        연결 종료 및 상태 리셋.
        - 사이클 타이머 정지
        - '응답 없음' 사이클 카운터 리셋
        - IG OFF 시도 후 포트 닫기
        """
        self.cycle_timer.stop()
        self._no_response_cycles = 0  # 사이클 카운터 리셋

        if self.serial_ig and self.serial_ig.is_open:
            # 장비 OFF (ACK 없어도 무시 가능)
            try:
                self.set_ig_state(False)
            except Exception:
                pass
            try:
                self.serial_ig.close()
            except Exception:
                pass
            self.serial_ig = None
            self.status_message.emit("IG", "연결 종료됨")

    # -------------------------------------------------------------------------
    # 외부 API
    # -------------------------------------------------------------------------
    @Slot(float)
    def start_wait_for_pressure(self, base_pressure: float):
        """
        비동기적으로 Base Pressure 도달 대기 시작. 즉시 리턴되며 결과는 시그널로 통지.
        - 사이클 타이머가 이미 도는 중이면 무시
        - 장비 켜기 실패 시 즉시 종료
        """
        if self.cycle_timer.isActive():
            self.status_message.emit("IG", "이미 압력 대기 프로세스가 실행 중입니다.")
            return

        self.status_message.emit("IG", "Base Pressure 대기를 시작합니다.")
        self._target_pressure = float(base_pressure)
        self._wait_start_time = time.time()
        self._no_response_cycles = 0  # 사이클 카운터 리셋

        if not self.set_ig_state(True):
            self.status_message.emit("IG", "장비를 켜는 데 실패했습니다.")
            self.base_pressure_failed.emit("IG", "장비 켜기 실패")
            return

        self._execute_wait_read_cycle()

    # -------------------------------------------------------------------------
    # 내부: 사이클 제어
    # -------------------------------------------------------------------------
    def _execute_wait_read_cycle(self):
        """압력을 다시 읽기 전 10초 대기를 시작."""
        self.status_message.emit("IG", "10초 대기 후 압력을 읽습니다...")
        self.cycle_timer.start(10_000)

    def _read_and_evaluate(self):
        """
        10초 대기 후 실제 압력을 읽고 목표와 비교.
        - 정상 응답이면 비교 후 통과/재시도 결정
        - '응답 없음'이면 사이클 카운트 증가 → 3사이클까지 재시도, 이후 종료
        - 전체 대기 시간(IG_WAIT_TIMEOUT) 초과 시 즉시 종료
        """
        # 중복 진입 방지(이벤트 루프/타이머 이슈 방지)
        if self._busy:
            return
        self._busy = True
        try:
            # 전체 대기 제한 체크
            if time.time() - self._wait_start_time > IG_WAIT_TIMEOUT:
                self.status_message.emit("IG", f"시간 초과({IG_WAIT_TIMEOUT}초): 목표 압력에 도달하지 못했습니다.")
                self.base_pressure_failed.emit("IG", "Timeout")
                self.cleanup()
                return

            pressure, status = self.read_pressure()

            # 사이클 단위 '응답 없음' 처리(최대 3사이클)
            if status == 'no_response':
                self._no_response_cycles += 1
                if self._no_response_cycles >= self._no_response_cycle_limit:
                    self.status_message.emit(
                        "IG",
                        f"응답 없음이 {self._no_response_cycles}회 연속 발생하여 프로세스를 종료합니다."
                    )
                    self.base_pressure_failed.emit("IG", "No response (cycle limit)")
                    self.cleanup()
                    return
                else:
                    self.status_message.emit(
                        "IG",
                        f"응답 없음: {self._no_response_cycles}/{self._no_response_cycle_limit}회. 10초 후 재시도합니다."
                    )
                    self._execute_wait_read_cycle()
                    return

            # IG OFF → 다음 사이클에서 재시도(응답은 받은 것이므로 카운터 리셋)
            if status == 'ig_off':
                self._no_response_cycles = 0
                self._execute_wait_read_cycle()
                return

            # 정상 응답(숫자)
            if pressure is not None:
                self._no_response_cycles = 0  # 정상 응답이므로 리셋
                self.status_message.emit(
                    "IG",
                    f"현재 압력: {pressure:.3e} Torr (목표: {self._target_pressure:.3e} Torr)"
                )
                if pressure <= self._target_pressure:
                    self.status_message.emit("IG", "목표 압력 도달")
                    self.pressure_update.emit(pressure)
                    self.base_pressure_reached.emit()
                    self.cleanup()
                    return
                else:
                    # 목표 미달 → 다음 사이클
                    self._execute_wait_read_cycle()
                    return

            # 그 외(파싱 실패 등) → 다음 사이클
            self.status_message.emit("IG", "압력 값을 읽지 못했습니다. 다음 사이클에서 재시도합니다.")
            self._execute_wait_read_cycle()

        finally:
            self._busy = False

    # -------------------------------------------------------------------------
    # 내부: 시리얼 IO (논블로킹 읽기)
    # -------------------------------------------------------------------------
    def _readline_nonblocking(self, deadline_monotonic: float) -> str:
        """
        논블로킹으로 '\r' 또는 '\n'까지 한 줄을 읽어 반환.
        - serial timeout=0 환경에서 in_waiting 기반으로 읽고, 데드라인을 엄격히 준수
        - 데드라인 내 라인 엔드가 없으면 지금까지 수집한 문자열(부분) 반환
        """
        buf = bytearray()
        port = self.serial_ig
        if not port:
            return ""

        while time.monotonic() < deadline_monotonic:
            try:
                n = port.in_waiting
                if n:
                    chunk = port.read(n)  # 즉시 반환
                    if chunk:
                        buf.extend(chunk)
                        if b'\r' in buf or b'\n' in buf:
                            # CR 우선, 없으면 LF 기준 분리
                            if b'\r' in buf:
                                line, _, _ = buf.partition(b'\r')
                            else:
                                line, _, _ = buf.partition(b'\n')
                            return line.decode(errors='ignore').strip()
                # 루프 양보(바쁘지 않게)
                QThread.msleep(5)
            except Exception:
                break

        return buf.decode(errors='ignore').strip()

    def read_pressure(self) -> tuple[float | None, str]:
        """
        IG에서 현재 압력 읽기(논블로킹).
        반환: (pressure, status)
          - status: 'ok' | 'ig_off' | 'no_response' | 'no_port'
        - 이 함수 내부에서도 MAX_READ_RETRIES 만큼 재시도(한 사이클 내 재시도)
        - 완전 무응답이면 'no_response' 반환 → 바깥에서 사이클 단위 재시도 제어
        """
        if not self.serial_ig:
            return None, 'no_port'

        # 이전 사이클 잔여 수신 버퍼 수동 플러시
        try:
            if self.serial_ig.in_waiting:
                self.serial_ig.read(self.serial_ig.in_waiting)
        except Exception:
            pass

        for attempt in range(1, MAX_READ_RETRIES + 1):
            try:
                # 명령 송신
                self.serial_ig.write(b"RDI\r")

                # 데드라인까지 논블로킹으로 라인 수집
                deadline = time.monotonic() + RESPONSE_TIMEOUT_SEC
                response = self._readline_nonblocking(deadline)

                if not response:
                    self.status_message.emit(
                        "IG",
                        f"응답 없음 (시도 {attempt}/{MAX_READ_RETRIES}, {RESPONSE_TIMEOUT_SEC:.0f}s 대기)"
                    )
                    continue

                # IG OFF 처리(대소문자/공백 관대)
                if response.strip().upper() == "IG OFF":
                    self.status_message.emit(
                        "IG", "장비가 꺼져있어 자동으로 켭니다. 다음 사이클에서 재시도합니다."
                    )
                    self.set_ig_state(True)
                    return None, 'ig_off'

                # 비표준 표기 보정 후 float 변환
                cleaned = response.lower().replace('x10e', 'e')
                value = float(cleaned)
                return value, 'ok'

            except ValueError as e:
                self.status_message.emit("IG", f"압력 값 변환 실패: '{response}' -> {e}")
                # 다음 재시도
            except Exception as e:
                self.status_message.emit("IG", f"읽기 오류: {e}")
                # 다음 재시도

        # 모든 내부 재시도 실패 → 바깥 사이클 재시도 판단용
        return None, 'no_response'

    def set_ig_state(self, state: bool) -> bool:
        """
        IG ON/OFF 제어.
        - LabVIEW VI에서 SIG 1 = open / SIG 0 = close
        - 읽기 루프와 충돌하지 않도록 간단히 데드라인을 두고 ACK('OK') 확인
        """
        if not self.serial_ig:
            return False
        try:
            cmd = b"SIG 1\r" if state else b"SIG 0\r"
            self.serial_ig.write(cmd)
            # 짧은 데드라인으로 응답 확인(없어도 False 반환)
            ack = self._readline_nonblocking(time.monotonic() + 0.5)
            return ack.strip().upper() == "OK"
        except Exception:
            return False
