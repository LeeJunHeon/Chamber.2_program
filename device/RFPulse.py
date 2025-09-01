# device/RFPulse.py
from __future__ import annotations

from typing import Optional, Tuple
from PyQt6.QtCore import QObject, QTimer, QCoreApplication, QElapsedTimer, pyqtSignal as Signal, pyqtSlot as Slot
from PyQt6.QtSerialPort import QSerialPort, QSerialPort as QSP
from lib.config import RFPULSE_PORT, RFPULSE_BAUD, RFPULSE_ADDR

# ===== 타이밍/타임아웃 상수 =====
ACK_TIMEOUT_MS         = 2000   # 쓰기(설정) 명령 후 ACK/프레임 대기 시간
QUERY_TIMEOUT_MS       = 2500   # 읽기(리드백) 명령 후 데이터 프레임 대기 시간
RECV_FRAME_TIMEOUT_MS  = 2000   # _recv_frame 기본 타임아웃
CMD_GAP_MS             = 1500   # 명령 간 최소 간격(밀리초)
POST_WRITE_DELAY_MS    = 1500    # 각 쓰기 명령 후 여유 대기(밀리초)

# ===== AE Bus command numbers (기존 테스트 코드와 동일) =====
CMD_RF_OFF              = 1
CMD_RF_ON               = 2
CMD_SET_CTRL_MODE       = 3      # data: 6=FWD, 7=LOAD, 8=EXT (1 byte)
CMD_SET_SETPOINT        = 8      # data: 16-bit Watts, LSB first
CMD_SET_ACTIVE_CTRL     = 14     # data: 0x02 -> Host

# Reads (report)
CMD_REPORT_STATUS        = 162
CMD_REPORT_SETPOINT_MODE = 164   # 2B setpoint (W, LSB first) + 1B mode
CMD_REPORT_FORWARD       = 165   # 2B W (LSB first)
CMD_REPORT_REFLECTED     = 166   # 2B W
CMD_REPORT_DELIVERED     = 167   # 2B W

# Pulsing
CMD_SET_PULSING         = 27     # 0=off, 1=int, 2=ext, 3=ext_inv, 4=int_by_ext
CMD_SET_PULSE_FREQ      = 93     # 3 bytes (Hz, LSB first)
CMD_SET_PULSE_DUTY      = 96     # 2 bytes (percent, LSB first)

# Pulsing 리드백(보고) 명령 추가
CMD_REPORT_PULSING      = 177   # 현재 펄싱 모드 readback (선택 사용)
CMD_REPORT_PULSE_FREQ   = 193   # 펄스 주파수 readback (3B LSB-first, 일부 FW는 2B)
CMD_REPORT_PULSE_DUTY   = 196   # 듀티(%) readback (2B 또는 1B)

CSR_CODES = {
    0: "OK",
    1: "Command Not Recognized",
    2: "Not in Host Mode",
    3: "Not Implemented",
    4: "Bad Data Value",
    5: "Busy",
}

MODE_SET  = {"fwd": 6, "load": 7, "ext": 8}
MODE_NAME = {46: "FWD", 47: "LOAD", 48: "EXT", 6: "FWD", 7: "LOAD", 8: "EXT"}

def _u16le(buf: bytes, i: int = 0) -> int:
    return buf[i] | (buf[i+1] << 8)

def _u24le(buf: bytes, i: int = 0) -> int:
    return buf[i] | (buf[i+1] << 8) | (buf[i+2] << 16)

def _build_packet(addr: int, cmd: int, data: bytes=b"") -> bytes:
    if not (0 <= addr <= 31):
        raise ValueError("addr 0..31")
    L = len(data)
    if L <= 6:
        header = ((addr & 0x1F) << 3) | L
        body = bytes([header, cmd]) + data
    else:
        header = ((addr & 0x1F) << 3) | 0x07
        body = bytes([header, cmd, L & 0xFF]) + data
    cs = 0
    for b in body: cs ^= b
    return body + bytes([cs & 0xFF])

# ====== (추가) 간단 리드백 헬퍼들 ======
def _bit(v: int, n: int) -> int:
    return (v >> n) & 1

class RFPulseController(QObject):
    """CESAR AE RS-232(RS-232C) Pulse 제어 (QSerialPort 사용, 워커 스레드에서 동작)"""

    # 로그 / 상태
    status_message = Signal(str, str)            # (src, msg)
    # 공정 컨트롤러 연동
    target_reached = Signal()                    # RF ON 완료 (정상)
    target_failed  = Signal(str)                 # 실패 사유
    power_off_finished = Signal()                # RF OFF 완료

    def __init__(self, parent=None):
        super().__init__(parent)
        self.port: QSerialPort = QSerialPort(self)
        self.addr: int = RFPULSE_ADDR
        # 명령 간 대기 / 기본 지연
        self._min_cmd_gap_ms = CMD_GAP_MS
        self.default_delay_ms = POST_WRITE_DELAY_MS

        self._configured_port: Optional[str] = None
        self._default_port = RFPULSE_PORT
        self._default_baud = RFPULSE_BAUD

        # --- 워치독 ---
        self._wd_timer = QTimer(self)
        self._wd_timer.setInterval(2000)   # 2초마다 상태점검
        self._wd_timer.timeout.connect(self._on_watchdog_tick)
        self._wd_miss = 0
        self._wd_miss_limit = 2           # 2회 연속 실패 시 재연결
        self._max_retries = 5             # 설정/검증 실패시 재시도 상한

        self._port_had_error = False
        self.port.errorOccurred.connect(self._on_port_error)  # 포트 에러 감지

        # --- 전송 스로틀(명령 사이 최소 간격) ---
        self._tx_epoch = QElapsedTimer()
        self._tx_epoch.start()
        self._last_sent_at_ms = -10**9

    # ---------- 포트 ----------
    @Slot(QSP.SerialPortError)
    def _on_port_error(self, err):
        critical = {
            QSP.SerialPortError.ResourceError,
            QSP.SerialPortError.DeviceNotFoundError,
            QSP.SerialPortError.PermissionError,
            QSP.SerialPortError.NotOpenError,
        }
        if err in critical:
            self._port_had_error = True

    def _tx_throttle(self):
        """마지막 전송 시각으로부터 _min_cmd_gap_ms 만큼 대기."""
        if self._min_cmd_gap_ms <= 0:
            return
        now = self._tx_epoch.elapsed()
        remain = self._min_cmd_gap_ms - (now - self._last_sent_at_ms)
        if remain > 0:
            self._delay_ms(remain)

    @Slot()
    def connect_rfpulse_device(self) -> bool:
        self.set_address(RFPULSE_ADDR)
        ok = self.open_port(self._default_port, self._default_baud)
        if ok:
            self.status_message.emit("RFPulse", f"연결 성공: {self._default_port}@{self._default_baud}")
            self._wd_miss = 0
            if not self._wd_timer.isActive():
                self._wd_timer.start()
        else:
            self.status_message.emit("RFPulse", f"연결 실패: {self._default_port}@{self._default_baud}")
        return ok

    @Slot(int)
    def set_address(self, addr: int):
        self.addr = int(addr) & 0x1F

    @Slot(str, int)
    def open_port(self, port_name: str, baud: int = 9600) -> bool:
        """지연 연결 패턴: 이미 열려있다면 같은 포트면 패스, 다르면 교체."""
        if self.port.isOpen() and self._configured_port == port_name:
            return True

        if self.port.isOpen():
            try: self.port.close()
            except: pass

        self.port.setPortName(port_name)
        self.port.setBaudRate(baud)
        self.port.setDataBits(QSP.DataBits.Data8)
        self.port.setParity(QSP.Parity.OddParity)          # 8O1
        self.port.setStopBits(QSP.StopBits.OneStop)
        self.port.setFlowControl(QSP.FlowControl.NoFlowControl)

        ok = self.port.open(QSP.OpenModeFlag.ReadWrite)
        if not ok:
            self.status_message.emit("RFPulse", f"포트 열기 실패: {port_name}")
            return False

        # DTR 필요(장비에 따라 필수)
        try:
            self.port.setDataTerminalReady(True)
        except Exception:
            pass

        self._configured_port = port_name
        self.status_message.emit("RFPulse", f"포트 연결 완료: {port_name}")
        return True

    @Slot()
    def close_port(self):
        if self.port.isOpen():
            try:
                self.port.close()
                self.status_message.emit("RFPulse", "포트 닫힘")
            except Exception:
                pass
        if self._wd_timer.isActive():
            self._wd_timer.stop()

    # ---------- 워치독 ----------
    def _on_watchdog_tick(self):
        if not self.port or not self.port.isOpen() or self._port_had_error:
            self._port_had_error = False
            self.status_message.emit("RFPulse", "[WD] 포트 이상 감지 → 재연결 시도")
            self.reconnect()
            return

    def reconnect(self) -> bool:
        try:
            self.close_port()
        except Exception:
            pass
        ok = self.open_port(self._default_port, self._default_baud)
        if ok:
            self._wd_miss = 0
            if not self._wd_timer.isActive():
                self._wd_timer.start()
            self.status_message.emit("RFPulse", "[WD] 재연결 성공")
        else:
            self.status_message.emit("RFPulse", "[WD] 재연결 실패")
        return ok

    def _wd_pause(self):
        if self._wd_timer.isActive():
            self._wd_timer.stop()

    def _wd_resume(self):
        if self.is_connected() and not self._wd_timer.isActive():
            self._wd_timer.start()

    # ---------- 저수준 통신 ----------
    def _delay_ms(self, ms: int):
        t = QElapsedTimer(); t.start()
        while t.elapsed() < ms:
            QCoreApplication.processEvents()

    def _read_bytes(self, need: int, deadline_ms: int) -> Optional[bytes]:
        buf = bytearray()
        t = QElapsedTimer(); t.start()
        while t.elapsed() < deadline_ms:
            if self.port.waitForReadyRead(max(1, deadline_ms - t.elapsed())):
                chunk: bytes = self.port.read(need - len(buf))
                if chunk:
                    buf.extend(chunk)
                    if len(buf) >= need:
                        return bytes(buf)
        return None

    def _recv_frame(self, timeout_ms: int = RECV_FRAME_TIMEOUT_MS, allow_ack_only: bool = False):
        """ACK(0x06) 스킵, NAK(0x15) 에러, 유효 프레임 리턴"""
        seen_ack = False
        t = QElapsedTimer(); t.start()
        while t.elapsed() < timeout_ms:
            if not self.port.waitForReadyRead(max(1, timeout_ms - t.elapsed())):
                continue
            b = bytes(self.port.read(1))
            if not b:
                continue
            if b == b"\x15":
                self.status_message.emit("RFPulse", "RX: NAK")
                raise RuntimeError("NAK(0x15) received")
            if b == b"\x06":
                self.status_message.emit("RFPulse", "RX: ACK")
                seen_ack = True
                continue

            hdr = b[0]
            length_bits = hdr & 0x07

            cmd_b = self._read_bytes(1, timeout_ms - t.elapsed())
            if not cmd_b:
                continue

            if length_bits == 7:
                opt_len = self._read_bytes(1, timeout_ms - t.elapsed())
                if not opt_len:
                    continue
                data_len = opt_len[0]
                data = self._read_bytes(data_len, timeout_ms - t.elapsed())
                if data is None or len(data) != data_len:
                    continue
                cs = self._read_bytes(1, timeout_ms - t.elapsed())
                if not cs:
                    continue
                pkt = bytes([hdr]) + cmd_b + opt_len + data + cs
            else:
                data_len = length_bits
                data = self._read_bytes(data_len, timeout_ms - t.elapsed())
                if data is None or len(data) != data_len:
                    continue
                cs = self._read_bytes(1, timeout_ms - t.elapsed())
                if not cs:
                    continue
                pkt = bytes([hdr]) + cmd_b + data + cs

            # checksum
            calc = 0
            for x in pkt[:-1]: calc ^= x
            if (calc ^ pkt[-1]) != 0:
                continue
            return ("FRAME", pkt)

        if allow_ack_only and seen_ack:
            return ("ACK_ONLY", None)
        self.status_message.emit("RFPulse", "RX: timeout waiting frame")
        raise TimeoutError("No response header")

    # ---------- 쓰기: ACK만 확인하고 진행 ----------
    def _send_exec_ack(self, cmd: int, data: bytes=b"", timeout_ms: int=ACK_TIMEOUT_MS) -> None:
        """
        쓰기(설정) 명령: ACK만 받으면 성공으로 간주하고 다음 단계로 진행.
        CSR/리드백 검증은 수행하지 않음.
        """
        self.port.clear(QSP.Direction.AllDirections)
        pkt = _build_packet(self.addr, cmd, data)
        self.status_message.emit(
            "RFPulse",
            f"TX exec(ack-only): addr={self.addr} cmd=0x{cmd:02X} data={data.hex(' ')} pkt={pkt.hex(' ')}"
        )
        self._tx_throttle()
        self._last_sent_at_ms = self._tx_epoch.elapsed()
        self.port.write(pkt)
        self.port.waitForBytesWritten(timeout_ms)
        kind, _payload = self._recv_frame(timeout_ms=timeout_ms, allow_ack_only=True)  # ACK or FRAME
        if kind not in ("ACK_ONLY", "FRAME"):
            raise TimeoutError("Expected ACK/FRAME not received")

    def _send_query(self, cmd: int, data: bytes=b"", timeout_ms: int=QUERY_TIMEOUT_MS) -> bytes:
        self.port.clear(QSP.Direction.AllDirections)
        pkt = _build_packet(self.addr, cmd, data)
        self.status_message.emit(
            "RFPulse",
            f"TX query: addr={self.addr} cmd=0x{cmd:02X} data={data.hex(' ')} pkt={pkt.hex(' ')}"
        )
        self._tx_throttle()
        self._last_sent_at_ms = self._tx_epoch.elapsed()
        self.port.write(pkt)
        self.port.waitForBytesWritten(timeout_ms)
        kind, payload = self._recv_frame(timeout_ms=timeout_ms, allow_ack_only=False)
        if kind != "FRAME":
            raise TimeoutError("No data frame")
        p = payload
        hdr = p[0]
        rx_cmd = p[1]
        length_bits = hdr & 0x07
        idx = 2
        if length_bits == 7:
            dlen = p[idx]; idx += 1
        else:
            dlen = length_bits
        data_bytes = p[idx:idx+dlen]
        self.status_message.emit(
            "RFPulse",
            f"RX query FRAME: hdr=0x{hdr:02X} cmd=0x{rx_cmd:02X} dlen={dlen} data={data_bytes.hex(' ')} pkt={p.hex(' ')}"
        )
        return data_bytes

    # ---------- 고수준 동작 (검증 제거: ACK만 확인) ----------
    def _host_mode(self):
        self._send_exec_ack(CMD_SET_ACTIVE_CTRL, b"\x02", ACK_TIMEOUT_MS)
        self.status_message.emit("RFPulse", "[HOST] sent (ack-only)")
        self._delay_ms(self.default_delay_ms)

    def _set_mode(self, mode: str = "fwd"):
        m = MODE_SET.get(mode.lower())
        if m is None:
            raise ValueError("mode should be fwd/load/ext")
        self._send_exec_ack(CMD_SET_CTRL_MODE, bytes([m]), ACK_TIMEOUT_MS)
        self.status_message.emit("RFPulse", f"[MODE {mode.upper()}] sent (ack-only)")
        self._delay_ms(self.default_delay_ms)

    def _set_power(self, watts: int):
        if watts < 0: watts = 0
        if watts > 65535: watts = 65535
        data = bytes([watts & 0xFF, (watts >> 8) & 0xFF])
        self._send_exec_ack(CMD_SET_SETPOINT, data, ACK_TIMEOUT_MS)
        self.status_message.emit("RFPulse", f"[SETP {watts}W] sent (ack-only)")
        self._delay_ms(self.default_delay_ms)

    def _set_pulsing(self, mode_code: int):
        self._send_exec_ack(CMD_SET_PULSING, bytes([mode_code & 0xFF]), ACK_TIMEOUT_MS)
        self.status_message.emit("RFPulse", f"[PULSING {mode_code}] sent (ack-only)")
        self._delay_ms(self.default_delay_ms)

    def _set_pulse_freq(self, hz: int):
        if hz < 1 or hz > 100000:
            raise ValueError("freq 1..100000Hz")
        data = bytes([hz & 0xFF, (hz >> 8) & 0xFF, (hz >> 16) & 0xFF])
        self._send_exec_ack(CMD_SET_PULSE_FREQ, data, ACK_TIMEOUT_MS)
        self.status_message.emit("RFPulse", f"[PULSE FREQ {hz}] sent (ack-only)")
        self._delay_ms(self.default_delay_ms)

    def _set_pulse_duty(self, duty_percent: int):
        if duty_percent < 1 or duty_percent > 99:
            raise ValueError("duty 1..99%")
        v = int(duty_percent) & 0xFFFF
        data = bytes([v & 0xFF, (v >> 8) & 0xFF])
        self._send_exec_ack(CMD_SET_PULSE_DUTY, data, ACK_TIMEOUT_MS)
        self.status_message.emit("RFPulse", f"[PULSE DUTY {duty_percent}%] sent (ack-only)")
        self._delay_ms(self.default_delay_ms)

    def _rf_on(self):
        self._send_exec_ack(CMD_RF_ON, b"", timeout_ms=max(ACK_TIMEOUT_MS, 2500))
        self.status_message.emit("RFPulse", "[RF ON] sent (ack-only)")

    def _rf_off(self):
        try:
            self._send_exec_ack(CMD_RF_OFF, b"", timeout_ms=max(ACK_TIMEOUT_MS, 2500))
            self.status_message.emit("RFPulse", "[RF OFF] sent (ack-only)")
        except Exception as e:
            self.status_message.emit("RFPulse", f"[RF OFF] 예외(무시): {e}")

    # ---------- (선택) 리드백 유틸 (그대로 보존, 상단 타임아웃 상수 반영) ----------
    def _report_process_status(self) -> tuple[int, int, bytes]:
        data = self._send_query(CMD_REPORT_STATUS, b"", QUERY_TIMEOUT_MS)
        if len(data) < 2:
            raise RuntimeError("report status payload too short")
        return data[0], data[1], data

    def _read_setpoint_mode(self) -> tuple[int, int]:
        data = self._send_query(CMD_REPORT_SETPOINT_MODE, b"", QUERY_TIMEOUT_MS)
        sp = _u16le(data, 0) if len(data) >= 2 else 0
        md = data[2] if len(data) >= 3 else 0
        return sp, md

    def _read_forward_power(self) -> int:
        data = self._send_query(CMD_REPORT_FORWARD, b"", QUERY_TIMEOUT_MS)
        return _u16le(data, 0) if len(data) >= 2 else 0

    def _read_pulsing_mode(self) -> Optional[int]:
        try:
            data = self._send_query(CMD_REPORT_PULSING, b"", QUERY_TIMEOUT_MS)
            return data[0] if len(data) >= 1 else None
        except Exception:
            return None

    def _read_pulse_freq(self) -> Optional[int]:
        try:
            data = self._send_query(CMD_REPORT_PULSE_FREQ, b"", QUERY_TIMEOUT_MS)
            if len(data) >= 3:
                return _u24le(data, 0)
            elif len(data) >= 2:
                return _u16le(data, 0)
            elif len(data) == 1:
                return data[0]
            return None
        except Exception:
            return None

    def _read_pulse_duty(self) -> Optional[int]:
        try:
            data = self._send_query(CMD_REPORT_PULSE_DUTY, b"", QUERY_TIMEOUT_MS)
            if len(data) >= 2:
                return _u16le(data, 0)
            elif len(data) == 1:
                return data[0]
            return None
        except Exception:
            return None

    # ---------- 외부에서 호출 (검증 제거 버전) ----------
    @Slot(float, object, object)
    def start_pulse_process(self, target_w: float, freq_hz: Optional[int] = None, duty_percent: Optional[int] = None):
        """
        순서: HOST(14) → FWD(3) → SETP(8) → (옵션: 93/96) → RF ON(2)
        - 각 단계는 'ACK 수신'만 확인하고 다음 단계로 진행 (검증/리드백 제거)
        - 실패/예외(ACK 미수신 등) 시 재연결 후 재시도(self._max_retries)
        """
        self._wd_pause()
        try:
            last_err = None
            for attempt in range(1, self._max_retries + 1):
                try:
                    if not self.is_connected():
                        if not self.connect_rfpulse_device():
                            raise RuntimeError(f"포트 '{self._default_port}' 연결 실패")

                    # 1) HOST
                    self._host_mode()

                    # 2) FWD 모드
                    self._set_mode("fwd")

                    # 3) 세트포인트
                    sp = int(round(float(target_w)))
                    self._set_power(sp)

                    # 4) (옵션) 펄스 파라미터
                    if freq_hz is not None:
                        self._set_pulse_freq(int(freq_hz))
                    if duty_percent is not None:
                        self._set_pulse_duty(int(duty_percent))

                    # 5) RF ON
                    self._rf_on()

                    # 성공(검증 없음)
                    self.target_reached.emit()
                    last_err = None
                    break

                except Exception as e:
                    last_err = e
                    self.status_message.emit("RFPulse", f"[시도 {attempt}/{self._max_retries}] 실패: {e}")
                    try:
                        self._rf_off()
                    except Exception:
                        pass
                    if attempt < self._max_retries:
                        self.reconnect()
                    else:
                        raise

        except Exception as final_e:
            try:
                self._rf_off()
            finally:
                self.power_off_finished.emit()
            self.target_failed.emit(str(final_e))
        finally:
            self._wd_resume()

    def is_connected(self) -> bool:
        return bool(self.port and self.port.isOpen())

    @Slot()
    def stop_process(self, also_turn_pulsing_off: bool = True):
        """STOP/에러 시 즉시 RF OFF"""
        try:
            if also_turn_pulsing_off:
                try:
                    self._set_pulsing(0)  # off
                except Exception:
                    pass
            self._rf_off()
        finally:
            self.power_off_finished.emit()

    @Slot()
    def cleanup(self):
        """워커 스레드에서 안전 종료: stop → close"""
        try:
            self.stop_process()
        except Exception:
            pass
        try:
            self.close_port()
        except Exception:
            pass
