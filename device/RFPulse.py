# device/RFPulse.py
from __future__ import annotations

from typing import Optional
from PyQt6.QtCore import QObject, QTimer, QCoreApplication, QElapsedTimer, pyqtSignal as Signal, pyqtSlot as Slot
from PyQt6.QtSerialPort import QSerialPort, QSerialPortInfo
from lib.config import RFPULSE_PORT, RFPULSE_BAUD, RFPULSE_ADDR

# ===== 타이밍/타임아웃 상수 =====
ACK_TIMEOUT_MS         = 2000   # 쓰기(설정) 명령 후 ACK/프레임 대기 시간
QUERY_TIMEOUT_MS       = 2500   # 읽기(리드백) 명령 후 데이터 프레임 대기 시간
RECV_FRAME_TIMEOUT_MS  = 2000   # _recv_frame 기본 타임아웃
CMD_GAP_MS             = 1500   # 명령 간 최소 간격(밀리초)
POST_WRITE_DELAY_MS    = 1500   # 각 쓰기 명령 후 여유 대기(밀리초)

# 워치독/재연결(지수 백오프)
RFPULSE_WATCHDOG_INTERVAL_MS        = 2000
RFPULSE_RECONNECT_BACKOFF_START_MS  = 2000
RFPULSE_RECONNECT_BACKOFF_MAX_MS    = 15000

# ===== AE Bus command numbers =====
CMD_RF_OFF              = 1
CMD_RF_ON               = 2
CMD_SET_CTRL_MODE       = 3
CMD_SET_SETPOINT        = 8
CMD_SET_ACTIVE_CTRL     = 14

# Reads (report)
CMD_REPORT_STATUS        = 162
CMD_REPORT_SETPOINT_MODE = 164
CMD_REPORT_FORWARD       = 165
CMD_REPORT_REFLECTED     = 166
CMD_REPORT_DELIVERED     = 167

# Pulsing
CMD_SET_PULSING         = 27     # 0=off, 1=int, 2=ext, 3=ext_inv, 4=int_by_ext
CMD_SET_PULSE_FREQ      = 93     # 3 bytes (Hz, LSB first)
CMD_SET_PULSE_DUTY      = 96     # 2 bytes (percent, LSB first)

# Pulsing 리드백(선택)
CMD_REPORT_PULSING      = 177
CMD_REPORT_PULSE_FREQ   = 193
CMD_REPORT_PULSE_DUTY   = 196

CSR_CODES = {
    0: "OK",
    1: "Command Not Recognized",
    2: "Not in Host Mode",
    3: "Not Implemented",
    4: "Bad Data Value",
    5: "Busy",
}

MODE_SET  = {"fwd": 6, "load": 7, "ext": 8}
MODE_NAME = {6: "FWD", 7: "LOAD", 8: "EXT"}

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

class RFPulseController(QObject):
    """CESAR AE RS-232 Pulse 제어 (QSerialPort 사용, 워커 스레드에서 동작)"""

    # 로그 / 상태
    status_message = Signal(str, str)            # (src, msg)
    # 공정 컨트롤러 연동
    target_reached = Signal()                    # RF ON 완료 (정상)
    target_failed  = Signal(str)                 # 실패 사유
    power_off_finished = Signal()                # RF OFF 완료
    update_rf_status_display = Signal(float, float)   # UI로 FWD/REF 송신

    def __init__(self, parent=None):
        super().__init__(parent)
        # ★★ 지연 생성(스레드 안전) — 포트/타이머는 None으로 두고 워커 스레드에서 생성
        self.port: Optional[QSerialPort] = None
        self._watchdog: Optional[QTimer] = None

        self.addr: int = RFPULSE_ADDR
        self._min_cmd_gap_ms = CMD_GAP_MS
        self.default_delay_ms = POST_WRITE_DELAY_MS

        self._configured_port: Optional[str] = None
        self._default_port = RFPULSE_PORT
        self._default_baud = RFPULSE_BAUD

        # 재연결/워치독 상태
        self._want_connected = False
        self._reconnect_backoff_ms = RFPULSE_RECONNECT_BACKOFF_START_MS
        self._reconnect_pending = False
        self._port_had_error = False
        self._stop_requested = False
        self._max_retries = 5

        # 전송 스로틀(명령 간격)
        self._tx_epoch = QElapsedTimer(); self._tx_epoch.start()
        self._last_sent_at_ms = -10**9

        # __init__() 안에 추가
        self._io_inflight = False      # 현재 I/O 진행 중 표시
        self._need_reopen = False      # 다음 재연결 시 포트 재오픈 필요
        self._per_cmd_retries = 3      # 명령 단위 재시도 횟수

        self._poll_timer = None                 # ← QTimer 대신 None
        self._last_forward_w = None
        self._last_reflected_w = None

    # ---------- 내부 생성기 ----------
    def _ensure_serial_created(self):
        if self.port is not None:
            return
        self.port = QSerialPort(self)
        self.port.setBaudRate(self._default_baud)
        self.port.setDataBits(QSerialPort.DataBits.Data8)
        self.port.setParity(QSerialPort.Parity.OddParity)    # 8O1
        self.port.setStopBits(QSerialPort.StopBits.OneStop)
        self.port.setFlowControl(QSerialPort.FlowControl.NoFlowControl)
        self.port.errorOccurred.connect(self._on_port_error)

    def _ensure_timers_created(self):
        if self._watchdog is None:
            self._watchdog = QTimer(self)
            self._watchdog.setInterval(RFPULSE_WATCHDOG_INTERVAL_MS)
            self._watchdog.timeout.connect(self._watch_connection)

    def _ensure_poll_timer_created(self):
        if self._poll_timer is None:
            from PyQt6.QtCore import QTimer
            self._poll_timer = QTimer(self)
            self._poll_timer.setInterval(3000)          # 3초
            self._poll_timer.timeout.connect(self._poll_power)

    def _start_power_polling(self):
        self._ensure_poll_timer_created()
        if not self._poll_timer.isActive():
            self._poll_timer.start()
            self.status_message.emit("RFPulse", "FWD/REF polling started (3초)")

    def _stop_power_polling(self):
        if self._poll_timer and self._poll_timer.isActive():
            self._poll_timer.stop()
            self.status_message.emit("RFPulse", "FWD/REF polling stopped")

    # ---------- 포트/에러 ----------
    @Slot(QSerialPort.SerialPortError)
    def _on_port_error(self, err: QSerialPort.SerialPortError):
        # NoError만 무시, 그 외(TimeoutError 포함)는 모두 재연결 루틴 진입
        if err == QSerialPort.SerialPortError.NoError:
            return 

        es = self.port.errorString() if self.port else ""
        err_name  = getattr(err, "name", str(err))
        err_value = getattr(err, "value", None)
        self.status_message.emit("RFPulse", f"시리얼 오류: {es} (err={err_name}/{err_value})")

        # 타임아웃은 장치 응답 지연일 수 있으니 별도 조치 없음(아래 재시도 로직에서 해결)
        if err == QSerialPort.SerialPortError.TimeoutError:
            return

        # ❌ 여기서 절대 close() 하지 않음. 워치독에게 맡기기.
        self._port_had_error = True
        self._need_reopen = True           # 다음 재연결 시 안전하게 닫고 다시 열도록 표시
        self._ensure_timers_created()
        self._want_connected = True
        if not self._watchdog.isActive():
            self._watchdog.start()
        QTimer.singleShot(0, self._watch_connection)

    def _tx_throttle(self):
        if self._min_cmd_gap_ms <= 0:
            return
        now = self._tx_epoch.elapsed()
        remain = self._min_cmd_gap_ms - (now - self._last_sent_at_ms)
        if remain > 0:
            self._delay_ms(remain)

    @Slot()
    def connect_rfpulse_device(self) -> bool:
        if self._stop_requested:
            self._stop_requested = False  # 재가동
        self._ensure_serial_created()
        self._ensure_timers_created()

        self._want_connected = True
        ok = self._open_port()
        if self._watchdog:
            self._watchdog.start()
        return ok

    def _open_port(self) -> bool:
        if self.port and self.port.isOpen():
            return True

        # 포트 존재 확인(선택)
        available = {p.portName() for p in QSerialPortInfo.availablePorts()}
        if self._default_port not in available:
            self.status_message.emit("RFPulse",
                f"{self._default_port} 존재하지 않음. 사용 가능 포트: {sorted(available)}")
            return False

        self.port.setPortName(self._default_port)
        if not self.port.open(QSerialPort.OpenModeFlag.ReadWrite):
            self.status_message.emit("RFPulse", f"{self._default_port} 연결 실패: {self.port.errorString()}")
            return False
        
        self._port_had_error = False

        try: self.port.setDataTerminalReady(True)
        except Exception: pass
        try: self.port.setRequestToSend(False)
        except Exception: pass

        self._configured_port = self._default_port
        self._reconnect_backoff_ms = RFPULSE_RECONNECT_BACKOFF_START_MS
        self._reconnect_pending = False

        self.status_message.emit("RFPulse", f"{self._default_port} 연결 성공 (PyQt6 QSerialPort)")
        return True

    @Slot()
    def close_port(self):
        if self.port and self.port.isOpen():
            try:
                self.port.close()
                self.status_message.emit("RFPulse", "포트 닫힘")
            except Exception:
                pass
        if self._watchdog and self._watchdog.isActive():
            self._watchdog.stop()

    # ---------- 워치독/재연결(지수 백오프) ----------
    def _watch_connection(self):
        if (not self._want_connected) or self._stop_requested:
            return
        if self._reconnect_pending:
            return
        # ⬇️ 포트가 열려 있어도 재오픈 플래그가 있으면 '재연결 필요'로 간주
        if self.port and self.port.isOpen() and not self._need_reopen:
            return
        self._reconnect_pending = True
        self.status_message.emit("RFPulse", f"재연결 시도... ({self._reconnect_backoff_ms} ms)")
        QTimer.singleShot(self._reconnect_backoff_ms, self._try_reconnect)

    def _try_reconnect(self):
        self._reconnect_pending = False
        if (not self._want_connected) or self._stop_requested:
            return
        # 이미 열려 있으면 재연결 불필요
        if self.port and self.port.isOpen() and not self._need_reopen:
            return

        # 🔐 포트를 열기 전에, 재오픈이 필요하다면 여기서만 닫는다(이 시점엔 I/O 없음).
        if self._need_reopen and self.port and self.port.isOpen():
            try: self.port.close()
            except Exception: pass
            finally: self._need_reopen = False

        if self._open_port():
            self.status_message.emit("RFPulse", "재연결 성공")
            self._reconnect_backoff_ms = RFPULSE_RECONNECT_BACKOFF_START_MS
            return

        self._reconnect_backoff_ms = min(
            self._reconnect_backoff_ms * 2, RFPULSE_RECONNECT_BACKOFF_MAX_MS
        )

    def _wd_pause(self):
        if self._watchdog and self._watchdog.isActive():
            self._watchdog.stop()

    def _wd_resume(self):
        if self._watchdog and (not self._watchdog.isActive()) and self._want_connected:
            self._watchdog.start()

    # ---------- 시리얼 통신 ----------
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

    def _recv_frame(
            self, 
            timeout_ms: int = RECV_FRAME_TIMEOUT_MS, 
            allow_ack_only: bool = False,
            *,
            allow_when_stopping: bool = False,
        ):
        if (self._stop_requested and not allow_when_stopping) or not self.is_connected():
            raise RuntimeError("Stopped or port closed")

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
                if allow_ack_only:
                    # ✅ ACK-only면 바로 반환 (타임아웃까지 기다리지 않음)
                    return ("ACK_ONLY", None)
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

    # ---------- 쓰기/읽기 (ACK만 확인) ----------
    def _send_exec_ack(self, cmd: int, data: bytes=b"", timeout_ms: int=ACK_TIMEOUT_MS, *, force: bool=False) -> None:
        if self._stop_requested and not force:
            raise RuntimeError("Stop requested (skip I/O)")
        if not self.is_connected():
            raise RuntimeError("Port not open")

        pkt = _build_packet(self.addr, cmd, data)
        label = f"exec 0x{cmd:02X}"

        for attempt in range(1, self._per_cmd_retries + 1):
            self.status_message.emit("RFPulse", f"TX {label}: addr={self.addr} data={data.hex(' ')} pkt={pkt.hex(' ')} (try {attempt}/{self._per_cmd_retries})")
            self._tx_throttle()
            self._last_sent_at_ms = self._tx_epoch.elapsed()

            self._io_inflight = True
            try:
                # ❌ clear(AllDirections) 제거 — I/O abort 방지
                self.port.write(pkt)
                self.port.waitForBytesWritten(timeout_ms)
                kind, _ = self._recv_frame(
                    timeout_ms=timeout_ms,
                    allow_ack_only=True,
                    allow_when_stopping=force,
                )
                if kind in ("ACK_ONLY", "FRAME"):
                    return
                raise TimeoutError("Expected ACK/FRAME not received")

            except Exception as e:
                # 실패 → 워치독 재연결 예약 후 백오프만큼 기다렸다가 동일 명령 재시도
                self._need_reopen = True         # 다음 재연결에서 안전 재오픈
                self._watch_connection()         # 워치독 트리거
                self._delay_ms(self._reconnect_backoff_ms)
                # 재연결 시도 후 포트가 살아났는지 점검
                if not self.is_connected():
                    # 재연결 타이머가 다시 돌 수 있게 한 번 더 툭 쳐줌
                    self._watch_connection()
                if attempt >= self._per_cmd_retries:
                    raise
                # 루프 계속

            finally:
                self._io_inflight = False

    def _send_query(self, cmd: int, data: bytes=b"", timeout_ms: int=QUERY_TIMEOUT_MS) -> bytes:
        if self._stop_requested:
            raise RuntimeError("Stop requested (skip I/O)")
        if not self.is_connected():
            raise RuntimeError("Port not open")

        pkt = _build_packet(self.addr, cmd, data)
        label = f"query 0x{cmd:02X}"

        for attempt in range(1, self._per_cmd_retries + 1):
            self.status_message.emit("RFPulse", f"TX {label}: addr={self.addr} data={data.hex(' ')} pkt={pkt.hex(' ')} (try {attempt}/{self._per_cmd_retries})")
            self._tx_throttle()
            self._last_sent_at_ms = self._tx_epoch.elapsed()

            self._io_inflight = True
            try:
                # ❌ clear(AllDirections) 제거
                self.port.write(pkt)
                self.port.waitForBytesWritten(timeout_ms)
                kind, payload = self._recv_frame(timeout_ms=timeout_ms, allow_ack_only=False)
                if kind != "FRAME":
                    raise TimeoutError("No data frame")

                p = payload
                hdr = p[0]
                length_bits = hdr & 0x07
                idx = 2
                if length_bits == 7:
                    dlen = p[idx]; idx += 1
                else:
                    dlen = length_bits
                data_bytes = p[idx:idx+dlen]
                self.status_message.emit("RFPulse", f"RX {label}: hdr=0x{hdr:02X} dlen={dlen} data={data_bytes.hex(' ')} pkt={p.hex(' ')}")
                return data_bytes

            except Exception as e:
                self._need_reopen = True
                self._watch_connection()
                self._delay_ms(self._reconnect_backoff_ms)
                if not self.is_connected():
                    self._watch_connection()
                if attempt >= self._per_cmd_retries:
                    raise

            finally:
                self._io_inflight = False

    # ---------- 장비 제어 동작 ----------
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

    def _set_pulsing(self, mode_code: int, *, force: bool=False):
        self._send_exec_ack(CMD_SET_PULSING, bytes([mode_code & 0xFF]), ACK_TIMEOUT_MS, force=force)
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

    def _rf_off(self, *, force: bool=False):
        self._send_exec_ack(CMD_RF_OFF, b"", timeout_ms=max(ACK_TIMEOUT_MS, 2500), force=force)
        self._stop_power_polling()
        tag = " (force)" if force else ""
        self.status_message.emit("RFPulse", f"[RF OFF] sent (ack-only){tag}")

    @Slot()
    def _poll_power(self):
        # 다른 명령 I/O 중이면 이번 틱은 건너뛴다
        if self._io_inflight or self._stop_requested:
            return
        if not self.is_connected():
            self._watch_connection()
            return
    
        if not self.is_connected():
            self._watch_connection()
            return

        fwd = None; ref = None
        try:
            fwd = float(self._read_forward_power())   # CMD 165
            self._last_forward_w = fwd
        except Exception as e:
            self.status_message.emit("RFPulse", f"[POLL] forward read failed: {e}")

        try:
            # reflected는 helper 없으면 아래처럼 직접:
            data = self._send_query(CMD_REPORT_REFLECTED, b"", QUERY_TIMEOUT_MS)  # CMD 166
            ref = float(_u16le(data, 0) if len(data) >= 2 else 0)
            self._last_reflected_w = ref
        except Exception as e:
            self.status_message.emit("RFPulse", f"[POLL] reflected read failed: {e}")

        out_f = self._last_forward_w if fwd is None else fwd
        out_r = self._last_reflected_w if ref is None else ref
        if (out_f is not None) and (out_r is not None):
            self.update_rf_status_display.emit(out_f, out_r)

    # ---------- (선택) 리드백 ----------
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

    # ---------- 외부 호출 ----------
    @Slot(float, object, object)
    def start_pulse_process(self, target_w: float, freq_hz: Optional[int] = None, duty_percent: Optional[int] = None):
        """
        HOST(14) → FWD(3) → SETP(8) → (옵션: 93/96) → RF ON(2)
        - 검증/리드백 제거(ACK만 확인)
        - 실패 시 워치독 재연결(지수 백오프)에 맡기고 재시도
        - Stop 요청 시 즉시 중단
        """
        self._wd_pause()
        try:
            last_err = None
            for attempt in range(1, self._max_retries + 1):
                if self._stop_requested:
                    raise RuntimeError("User stopped")

                try:
                    if not self.is_connected():
                        if not self.connect_rfpulse_device():
                            raise RuntimeError(f"포트 '{self._default_port}' 연결 실패")

                    self._host_mode()
                    self._set_mode("fwd")

                    sp = int(round(float(target_w)))
                    self._set_power(sp)

                    if freq_hz is not None:
                        self._set_pulse_freq(int(freq_hz))
                    if duty_percent is not None:
                        self._set_pulse_duty(int(duty_percent))

                    self._set_pulsing(1)

                    self._rf_on()

                    self.target_reached.emit()
                    self._start_power_polling()
                    last_err = None
                    break

                except Exception as e:
                    last_err = e
                    self.status_message.emit("RFPulse", f"[시도 {attempt}/{self._max_retries}] 실패: {e}")
                    try:
                        self._stop_power_polling()
                        self._rf_off()
                    except Exception:
                        pass

                    if self._stop_requested:
                        raise  # Stop이면 즉시 탈출

                    if attempt < self._max_retries:
                        # ⬇︎ 워치독에게 맡기고, 현재 백오프 시간만큼 대기 후 다음 루프
                        self._watch_connection()
                        self._delay_ms(self._reconnect_backoff_ms)
                        continue
                    else:
                        raise

        except Exception as final_e:
            try:
                self._rf_off()
            except Exception as off_e:
                self.status_message.emit("RFPulse", f"[RF OFF during failure] 실패: {off_e}")
            # ❌ 더 이상 power_off_finished.emit() 하지 않음
            self.target_failed.emit(str(final_e))

    def is_connected(self) -> bool:
        return bool(self.port and self.port.isOpen())
    
    @Slot()
    def stop_process(self, also_turn_pulsing_off: bool = True):
        # ✅ 정지 중에도 재연결/재시도가 돌 수 있게 설정
        self._stop_power_polling()
        self._stop_requested = True
        self._want_connected = True          # ← was False (삭제)
        self._reconnect_pending = False
        self._wd_resume()                    # ← was _wd_pause() (반대로)

        last_err = None
        for attempt in range(1, self._max_retries + 1):
            try:
                # 포트가 닫혀 있으면 정상 루틴처럼 연결 시도
                if not self.is_connected():
                    if not self.connect_rfpulse_device():
                        raise RuntimeError(f"포트 '{self._default_port}' 연결 실패")

                # ⬇ force=True → stop 상태여도 I/O 허용(allow_when_stopping)
                self._rf_off(force=True)

                # 성공시에만 완료 신호
                self.power_off_finished.emit()
                return

            except Exception as e:
                last_err = e
                self.status_message.emit("RFPulse",
                    f"[RF OFF(stop)] 시도 {attempt}/{self._max_retries} 실패: {e}")

                # 정상 루틴과 동일하게 워치독 트리거 + 백오프 대기 후 재시도
                self._watch_connection()
                self._delay_ms(self._reconnect_backoff_ms)
                continue

        # 모든 시도 실패 → 실패 알림(컨트롤러가 실패 처리)
        self.target_failed.emit(str(last_err))

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
        # 타이머/포트 deleteLater는 여기선 불필요(컨트롤러 생명주기와 함께 파괴됨)
