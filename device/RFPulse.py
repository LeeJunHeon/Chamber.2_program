# device/RFPulse.py
from __future__ import annotations

from dataclasses import dataclass
from collections import deque
from typing import Optional, Callable, Deque

from PyQt6.QtCore import (
    QObject, QTimer, QCoreApplication, QElapsedTimer,
    pyqtSignal as Signal, pyqtSlot as Slot
)
from PyQt6.QtSerialPort import QSerialPort, QSerialPortInfo
from lib.config import RFPULSE_PORT, RFPULSE_BAUD, RFPULSE_ADDR

# ===== 타이밍/타임아웃 상수 =====
ACK_TIMEOUT_MS         = 2000   # 쓰기(설정) 명령 후 ACK/프레임 대기 시간
QUERY_TIMEOUT_MS       = 4500   # 읽기(리드백) 명령 후 전체 대기 시간
RECV_FRAME_TIMEOUT_MS  = 4000   # _recv_frame 기본 타임아웃
CMD_GAP_MS             = 1500   # 명령 간 최소 간격(밀리초)
POST_WRITE_DELAY_MS    = 1500   # 각 쓰기 명령 후 여유 대기(밀리초)

# ★ ACK 뒤 CSR/데이터 프레임을 잠깐 더 기다려서 흡수하는 그레이스 윈도우
ACK_FOLLOWUP_GRACE_MS  = 500    # 400~600ms 권장

# 폴링 전용
POLL_INTERVAL_MS       = 3000   # 폴링 주기
POLL_QUERY_TIMEOUT_MS  = 9000   # 폴링 시 쿼리 타임아웃(ACK 후 지연 데이터 프레임 대비)

# RF ON 직후 폴링 시작 지연(장비 안정화 대기)
POLL_START_DELAY_AFTER_RF_ON_MS = 800

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

CMD_NAMES = {
    CMD_RF_OFF: "RF_OFF",
    CMD_RF_ON: "RF_ON",
    CMD_SET_CTRL_MODE: "SET_CTRL_MODE",
    CMD_SET_SETPOINT: "SET_SETPOINT",
    CMD_SET_ACTIVE_CTRL: "SET_ACTIVE_CTRL",
    CMD_REPORT_STATUS: "REPORT_STATUS",
    CMD_REPORT_SETPOINT_MODE: "REPORT_SETPOINT_MODE",
    CMD_REPORT_FORWARD: "REPORT_FORWARD",
    CMD_REPORT_REFLECTED: "REPORT_REFLECTED",
    CMD_REPORT_DELIVERED: "REPORT_DELIVERED",
    CMD_SET_PULSING: "SET_PULSING",
    CMD_SET_PULSE_FREQ: "SET_PULSE_FREQ",
    CMD_SET_PULSE_DUTY: "SET_PULSE_DUTY",
    CMD_REPORT_PULSING: "REPORT_PULSING",
    CMD_REPORT_PULSE_FREQ: "REPORT_PULSE_FREQ",
    CMD_REPORT_PULSE_DUTY: "REPORT_PULSE_DUTY",
}

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

# ======== 큐 명령 구조 ========
@dataclass
class RfCommand:
    kind: str  # "exec" (ACK 기대) | "query" (데이터 프레임 기대)
    cmd: int
    data: bytes
    timeout_ms: int
    gap_ms: int
    tag: str
    retries_left: int
    allow_no_reply: bool
    allow_when_closing: bool
    callback: Callable[[Optional[bytes]], None]  # 성공: bytes(빈바이트 허용), 실패: None

class RFPulseController(QObject):
    """CESAR AE RS-232 Pulse 제어 (QSerialPort + 명령 큐 직렬 처리)"""

    # 최소 로그(명령 전송/성공/실패)
    status_message = Signal(str, str)

    # 공정 컨트롤러 연동
    target_reached = Signal()                    # RF ON 완료 (정상)
    target_failed  = Signal(str)                 # 실패 사유
    power_off_finished = Signal()                # RF OFF 완료
    update_rf_status_display = Signal(float, float)   # UI로 FWD/REF 송신

    def __init__(self, parent=None):
        super().__init__(parent)
        # 지연 생성
        self.port: Optional[QSerialPort] = None
        self._watchdog: Optional[QTimer] = None
        self._gap_timer: Optional[QTimer] = None

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
        self._closing = False
        self._stop_requested = False
        self._need_reopen = False

        # 큐/인플라이트
        self._cmd_q: Deque[RfCommand] = deque()
        self._inflight: Optional[RfCommand] = None
        self._sending_spin = False

        # 전송 스로틀(명령 간격)
        self._tx_epoch = QElapsedTimer(); self._tx_epoch.start()
        self._last_sent_at_ms = -10**9

        # 폴링
        self._poll_timer: Optional[QTimer] = None
        self._poll_busy: bool = False
        self._last_forward_w: Optional[float] = None
        self._last_reflected_w: Optional[float] = None

    # ---------- 내부 유틸 ----------
    def _cmd_label(self, cmd: int) -> str:
        name = CMD_NAMES.get(cmd)
        return f"{name}(0x{cmd:02X})" if name else f"0x{cmd:02X}"

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
        self.port.setReadBufferSize(0)                       # 무제한(명시)
        self.port.errorOccurred.connect(self._on_port_error)

    def _ensure_timers_created(self):
        if self._watchdog is None:
            self._watchdog = QTimer(self)
            self._watchdog.setInterval(RFPULSE_WATCHDOG_INTERVAL_MS)
            self._watchdog.timeout.connect(self._watch_connection)
        if self._gap_timer is None:
            self._gap_timer = QTimer(self)
            self._gap_timer.setSingleShot(True)
            self._gap_timer.timeout.connect(self._dequeue_and_send)

    def _ensure_poll_timer_created(self):
        if self._poll_timer is None:
            self._poll_timer = QTimer(self)
            self._poll_timer.setInterval(POLL_INTERVAL_MS)
            self._poll_timer.timeout.connect(self._enqueue_poll_cycle)

    def _start_power_polling(self):
        self._ensure_poll_timer_created()
        if not self._poll_timer.isActive():
            self._poll_timer.start()

    def _stop_power_polling(self):
        if self._poll_timer and self._poll_timer.isActive():
            self._poll_timer.stop()

    # ---------- 포트/에러 ----------
    @Slot(QSerialPort.SerialPortError)
    def _on_port_error(self, err: QSerialPort.SerialPortError):
        if err == QSerialPort.SerialPortError.NoError:
            return
        if err == QSerialPort.SerialPortError.TimeoutError:
            return

        es = self.port.errorString() if self.port else ""
        err_name  = getattr(err, "name", str(err))
        err_value = getattr(err, "value", None)
        self.status_message.emit("RFPulse", f"시리얼 오류: {es} (err={err_name}/{err_value})")

        self._port_had_error = True
        if self._closing:
            return

        # 인플라이트 되돌리기
        if self._inflight:
            cmd = self._inflight
            self._inflight = None
            if cmd.retries_left > 0:
                cmd.retries_left -= 1
                self._cmd_q.appendleft(cmd)
            else:
                try: cmd.callback(None)
                except Exception: pass

        self._ensure_timers_created()
        self._want_connected = True
        self._need_reopen = True
        if not self._watchdog or not self._watchdog.isActive():
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
        self._closing = False
        if self._stop_requested:
            self._stop_requested = False
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

        available = {p.portName() for p in QSerialPortInfo.availablePorts()}
        if self._default_port not in available:
            self.status_message.emit("RFPulse",
                f"{self._default_port} 존재하지 않음. 사용 가능 포트: {sorted(available)}")
            return False

        self.port.setPortName(self._default_port)
        if not self.port.open(QSerialPort.OpenModeFlag.ReadWrite):
            self.status_message.emit("RFPulse", f"{self._default_port} 연결 실패: {self.port.errorString()}")
            return False

        try: self.port.setDataTerminalReady(True)
        except Exception: pass
        try: self.port.setRequestToSend(True)
        except Exception: pass
        try:
            from PyQt6.QtSerialPort import QSerialPort as _QSP
            self.port.clear(_QSP.Direction.AllDirections)
        except Exception:
            pass

        self._port_had_error = False
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
        if (not self._want_connected) or self._closing:
            return
        if self._reconnect_pending:
            return
        if self.port and self.port.isOpen() and not getattr(self, "_need_reopen", False):
            return
        self._reconnect_pending = True
        self.status_message.emit("RFPulse", f"재연결 시도... ({self._reconnect_backoff_ms} ms)")
        QTimer.singleShot(self._reconnect_backoff_ms, self._try_reconnect)

    def _try_reconnect(self):
        self._reconnect_pending = False
        if (not self._want_connected) or self._closing:
            return
        if self.port and self.port.isOpen() and not getattr(self, "_need_reopen", False):
            return

        if getattr(self, "_need_reopen", False) and self.port and self.port.isOpen():
            try: self.port.close()
            except Exception: pass
            finally: self._need_reopen = False

        if self._open_port():
            self.status_message.emit("RFPulse", "재연결 성공")
            self._reconnect_backoff_ms = RFPULSE_RECONNECT_BACKOFF_START_MS
            QTimer.singleShot(0, self._dequeue_and_send)
            return

        self._reconnect_backoff_ms = min(
            self._reconnect_backoff_ms * 2, RFPULSE_RECONNECT_BACKOFF_MAX_MS
        )

    # ---------- 시리얼 통신 ----------
    def _delay_ms(self, ms: int):
        t = QElapsedTimer(); t.start()
        while t.elapsed() < ms:
            if self._closing:
                break
            QCoreApplication.processEvents()

    def _read_bytes(self, need: int, deadline_ms: int) -> Optional[bytes]:
        if need <= 0:
            return b""
        buf = bytearray()
        t = QElapsedTimer(); t.start()
        while t.elapsed() < deadline_ms:
            # ① 버퍼에 이미 들어온 바이트부터 소비
            if self.port.bytesAvailable() > 0:
                chunk = self.port.read(need - len(buf))
                if chunk:
                    buf.extend(chunk)
                    if len(buf) >= need:
                        return bytes(buf)
                continue  # 아직 모자라면 루프 계속

            # ② 없으면 새 데이터 대기
            wait_ms = max(1, deadline_ms - t.elapsed())
            self.port.waitForReadyRead(wait_ms)
        return None

    def _recv_frame(
            self,
            timeout_ms: int = RECV_FRAME_TIMEOUT_MS,
            allow_ack_only: bool = False,
        ):
        if self._closing or not self.is_connected():
            raise RuntimeError("Stopped or port closed")

        seen_ack = False
        ack_grace_deadline = None  # ★ ACK를 본 후 더 기다릴 마감 시각

        t = QElapsedTimer(); t.start()
        while t.elapsed() < timeout_ms:
            remain = max(1, timeout_ms - t.elapsed())

            if allow_ack_only and seen_ack and ack_grace_deadline is not None and t.elapsed() >= ack_grace_deadline:
                return ("ACK_ONLY", None)

            if self.port.bytesAvailable() <= 0:
                if not self.port.waitForReadyRead(remain):
                    continue

            b = bytes(self.port.read(1))
            if not b:
                continue
            if b == b"\x15":
                raise RuntimeError("NAK(0x15) received")
            if b == b"\x06":
                seen_ack = True
                if allow_ack_only and ack_grace_deadline is None:
                    ack_grace_deadline = min(timeout_ms, t.elapsed() + ACK_FOLLOWUP_GRACE_MS)
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

            calc = 0
            for x in pkt[:-1]: calc ^= x
            if (calc ^ pkt[-1]) != 0:
                continue
            return ("FRAME", pkt)

        if allow_ack_only and seen_ack:
            return ("ACK_ONLY", None)

        raise TimeoutError("No response header")

    def _try_log_other_frame(self, payload: bytes):
        """프레임/RAW 디버그 로그는 사용하지 않음(흡수만)."""
        return

    # 기대 cmd/주소 일치 프레임만 돌려주는 래퍼
    def _recv_matching_frame(self, expected_cmd: int, timeout_ms: int, tag: str = "") -> bytes:
        t = QElapsedTimer(); t.start()
        while t.elapsed() < timeout_ms:
            remain = max(1, timeout_ms - t.elapsed())
            try:
                kind, payload = self._recv_frame(timeout_ms=remain, allow_ack_only=False)
            except TimeoutError:
                break
            except Exception:
                self._need_reopen = True
                self._watch_connection()
                break

            if kind != "FRAME" or not payload:
                continue

            hdr = payload[0]
            cmd_b = payload[1]
            rx_addr = (hdr >> 3) & 0x1F
            if rx_addr == self.addr and cmd_b == expected_cmd:
                length_bits = hdr & 0x07
                idx = 2
                if length_bits == 7:
                    dlen = payload[idx]; idx += 1
                else:
                    dlen = length_bits
                data_bytes = payload[idx:idx+dlen]
                return bytes(data_bytes)

            self._try_log_other_frame(payload)

        raise TimeoutError("No matching frame")

    # 안전 drain (clear 대신)
    def _drain_input_soft(self, max_bytes: int = 4096, budget_ms: int = 250):
        """하드 clear(Input) 대신 부드럽게 잔여 프레임 흡수"""
        t = QElapsedTimer(); t.start()
        while t.elapsed() < budget_ms and self.port and self.port.bytesAvailable():
            self.port.readAll()
            QCoreApplication.processEvents()

    # ---------- 큐 API ----------
    def enqueue_exec(self, cmd: int, data: bytes=b"", *,
                     tag: str="", timeout_ms: int=ACK_TIMEOUT_MS, gap_ms: int=CMD_GAP_MS,
                     retries: int=3, allow_no_reply: bool=False, allow_when_closing: bool=False,
                     callback: Optional[Callable[[Optional[bytes]], None]] = None):
        if self._closing and not allow_when_closing:
            return
        self._ensure_timers_created()
        cb = callback or (lambda _b: None)
        self._cmd_q.append(RfCommand(
            kind="exec", cmd=cmd, data=data, timeout_ms=timeout_ms, gap_ms=gap_ms,
            tag=tag, retries_left=retries, allow_no_reply=allow_no_reply,
            allow_when_closing=allow_when_closing, callback=cb
        ))
        if (self._inflight is None) and not (self._gap_timer and self._gap_timer.isActive()):
            QTimer.singleShot(0, self._dequeue_and_send)

    def enqueue_query(self, cmd: int, data: bytes=b"", *,
                      tag: str="", timeout_ms: int=QUERY_TIMEOUT_MS, gap_ms: int=CMD_GAP_MS,
                      retries: int=3, allow_when_closing: bool=False,
                      callback: Optional[Callable[[Optional[bytes]], None]] = None):
        if self._closing and not allow_when_closing:
            return
        self._ensure_timers_created()
        cb = callback or (lambda _b: None)
        self._cmd_q.append(RfCommand(
            kind="query", cmd=cmd, data=data, timeout_ms=timeout_ms, gap_ms=gap_ms,
            tag=tag, retries_left=retries, allow_no_reply=False,
            allow_when_closing=allow_when_closing, callback=cb
        ))
        if (self._inflight is None) and not (self._gap_timer and self._gap_timer.isActive()):
            QTimer.singleShot(0, self._dequeue_and_send)

    def _dequeue_and_send(self):
        if self._inflight is not None or not self._cmd_q:
            return
        if self._gap_timer and self._gap_timer.isActive():
            return
        if self._sending_spin:
            return
        self._sending_spin = True

        try:
            # 연결 보장
            if not self.is_connected():
                if not self.connect_rfpulse_device():
                    return

            cmd = self._cmd_q.popleft()
            self._inflight = cmd

            # 간격 스로틀
            self._tx_throttle()
            self._last_sent_at_ms = self._tx_epoch.elapsed()

            # 충돌 방지: exec/query 전송 직전 소프트 드레인
            if cmd.kind == "exec":
                self._drain_input_soft(max_bytes=4096, budget_ms=250)

            # === 최소 TX 로그 ===
            tag_disp = cmd.tag or ("exec" if cmd.kind == "exec" else "query")
            self.status_message.emit("RFPulse",
                f"TX {tag_disp} {self._cmd_label(cmd.cmd)} len={len(cmd.data)}")

            pkt = _build_packet(self.addr, cmd.cmd, cmd.data)
            self.port.write(pkt)
            self.port.flush()

            ok = False
            result: Optional[bytes] = None
            fail_reason: Optional[str] = None

            try:
                if cmd.kind == "exec":
                    # === 쓰기(설정) 명령 처리 ===
                    if cmd.allow_no_reply:
                        ok = True
                        result = b""
                    else:
                        kind, _ = self._recv_frame(timeout_ms=cmd.timeout_ms, allow_ack_only=True)
                        ok = (kind in ("ACK_ONLY", "FRAME"))
                        result = b"\x06" if ok else None

                        if ok and cmd.cmd == CMD_RF_ON:
                            QTimer.singleShot(0, self.target_reached.emit)

                else:
                    # === 읽기(리드백) 명령 처리 ===
                    t = QElapsedTimer(); t.start()
                    ack_phase = min(ACK_TIMEOUT_MS, cmd.timeout_ms // 3)
                    got_data_in_ack_phase = False

                    try:
                        kind, payload = self._recv_frame(timeout_ms=ack_phase, allow_ack_only=True)
                        if kind == "FRAME" and payload:
                            hdr = payload[0]; cmd_b = payload[1]
                            rx_addr = (hdr >> 3) & 0x1F
                            if rx_addr == self.addr and cmd_b == cmd.cmd:
                                length_bits = hdr & 0x07
                                idx = 2
                                if length_bits == 7:
                                    dlen = payload[idx]; idx += 1
                                else:
                                    dlen = length_bits
                                data_bytes = payload[idx:idx+dlen]
                                ok = True
                                result = bytes(data_bytes)
                                got_data_in_ack_phase = True
                            else:
                                self._try_log_other_frame(payload)
                    except Exception:
                        pass

                    if not got_data_in_ack_phase:
                        remain = max(1, cmd.timeout_ms - t.elapsed())
                        data_bytes = self._recv_matching_frame(cmd.cmd, timeout_ms=remain, tag=(cmd.tag or ""))
                        ok = True
                        result = data_bytes

            except TimeoutError:
                ok = False
                fail_reason = "timeout"
                result = None
            except Exception as e:
                ok = False
                fail_reason = "error"
                result = None
                self._need_reopen = True
                self._watch_connection()

            # === 최소 결과 로그 + 콜백/재시도/다음 예약 ===
            try:
                if ok:
                    self.status_message.emit("RFPulse", f"OK  {tag_disp} {self._cmd_label(cmd.cmd)}")
                    try: cmd.callback(result)
                    finally:
                        self._inflight = None
                        if self._gap_timer: self._gap_timer.start(cmd.gap_ms)
                else:
                    self.status_message.emit("RFPulse", f"FAIL {tag_disp} {self._cmd_label(cmd.cmd)}"
                                              + (f" ({fail_reason})" if fail_reason else ""))
                    if cmd.retries_left > 0 and not self._closing:
                        cmd.retries_left -= 1
                        self.status_message.emit("RFPulse",
                            f"RETRY {tag_disp} {self._cmd_label(cmd.cmd)} ({cmd.retries_left} left)")
                        self._inflight = None
                        self._cmd_q.appendleft(cmd)
                        if self._gap_timer: self._gap_timer.start(max(150, cmd.gap_ms))
                    else:
                        try: cmd.callback(None)
                        finally:
                            self._inflight = None
                            if self._gap_timer: self._gap_timer.start(cmd.gap_ms)
            except Exception:
                self._inflight = None
                if self._gap_timer: self._gap_timer.start(cmd.gap_ms)

        finally:
            self._sending_spin = False

    # ---------- 폴링(큐에 읽기 enqueue) ----------
    def _enqueue_poll_cycle(self):
        if self._closing or not self.is_connected():
            return
        if self._poll_busy or self._inflight is not None or len(self._cmd_q) > 0:
            return

        self._poll_busy = True
        tmp: dict = {}

        def _finish():
            self._poll_busy = False

        def on_ref(b: Optional[bytes]):
            # REF 파싱
            r = None
            if b is not None and len(b) >= 2:
                r = float(_u16le(b, 0))
            tmp['r'] = r

            # FWD와 REF 모두 확보 시 UI 갱신
            f = tmp.get('f', self._last_forward_w)
            r_val = tmp.get('r', self._last_reflected_w)
            if f is not None and r_val is not None:
                self._last_forward_w = f
                self._last_reflected_w = r_val
                self.update_rf_status_display.emit(f, r_val)

            _finish()

        def on_fwd(b: Optional[bytes]):
            # FWD 파싱
            f = None
            if b is not None and len(b) >= 2:
                f = float(_u16le(b, 0))
            tmp['f'] = f

            # REF 이어서
            self.enqueue_query(CMD_REPORT_REFLECTED, b"", tag="[POLL REF]",
                               timeout_ms=POLL_QUERY_TIMEOUT_MS, callback=on_ref)

        def after_wake(_b: Optional[bytes]):
            # WAKE 성공/실패와 무관하게 FWD 시도
            self.enqueue_query(CMD_REPORT_FORWARD, b"", tag="[POLL FWD]",
                               timeout_ms=POLL_QUERY_TIMEOUT_MS, callback=on_fwd)

        # 1) STATUS로 깨우기(잔여 프레임/지연 응답 흡수용)
        self.enqueue_query(CMD_REPORT_STATUS, b"", tag="[POLL WAKE]",
                           timeout_ms=POLL_QUERY_TIMEOUT_MS, callback=after_wake)

    # ---------- (선택) 리드백(큐 버전) ----------
    def _read_forward_power_enq(self, cb: Callable[[Optional[int]], None]):
        def _on(b: Optional[bytes]):
            if b is None: cb(None); return
            cb(_u16le(b, 0) if len(b) >= 2 else 0)
        self.enqueue_query(CMD_REPORT_FORWARD, b"", tag="[READ FWD]", callback=_on)

    def _read_reflected_power_enq(self, cb: Callable[[Optional[int]], None]):
        def _on(b: Optional[bytes]):
            if b is None: cb(None); return
            cb(_u16le(b, 0) if len(b) >= 2 else 0)
        self.enqueue_query(CMD_REPORT_REFLECTED, b"", tag="[READ REF]", callback=_on)

    # ---------- 외부 호출 (시작/정지) ----------
    @Slot(float, object, object)
    def start_pulse_process(self, target_w: float, freq_hz: Optional[int] = None, duty_percent: Optional[int] = None):
        """
        HOST(14) → FWD(3) → SETP(8) → (옵션: 93/96) → PULSING(27,1) → RF ON(2)
        모두 큐에 넣어 직렬 처리. 마지막 RF ON 성공 시 폴링 시작 + target_reached.
        """
        self._stop_requested = False
        self.set_process_status(False)

        def fail(msg: str):
            self.target_failed.emit(msg)

        def step6_rf_on(_b: Optional[bytes]):
            if _b is None:
                return fail("RF ON 실패")
            QTimer.singleShot(POLL_START_DELAY_AFTER_RF_ON_MS, lambda: self.set_process_status(True))

        def step5_pulsing_on(_b: Optional[bytes]):
            if _b is None:
                return fail("PULSING=1 실패")
            self.enqueue_exec(CMD_RF_ON, b"", tag="[START RF ON]",
                              timeout_ms=max(ACK_TIMEOUT_MS, 2500),
                              callback=step6_rf_on)

        def step4_set_duty(_b: Optional[bytes]):
            if duty_percent is not None and _b is None:
                return fail("PULSE DUTY 실패")
            self.enqueue_exec(CMD_SET_PULSING, bytes([1]), tag="[START PULSING 1]",
                              callback=step5_pulsing_on)

        def step3_set_freq(_b: Optional[bytes]):
            if freq_hz is not None and _b is None:
                return fail("PULSE FREQ 실패")
            if duty_percent is not None:
                v = int(duty_percent) & 0xFFFF
                data = bytes([v & 0xFF, (v >> 8) & 0xFF])
                self.enqueue_exec(CMD_SET_PULSE_DUTY, data, tag="[START DUTY]",
                                  callback=step4_set_duty)
            else:
                step4_set_duty(b"\x06")

        def step2_setp(_b: Optional[bytes]):
            if _b is None:
                return fail("SETP 실패")
            if freq_hz is not None:
                hz = int(freq_hz)
                data = bytes([hz & 0xFF, (hz >> 8) & 0xFF, (hz >> 16) & 0xFF])
                self.enqueue_exec(CMD_SET_PULSE_FREQ, data, tag="[START FREQ]",
                                  callback=step3_set_freq)
            else:
                step3_set_freq(b"\x06")

        def step1_mode(_b: Optional[bytes]):
            if _b is None:
                return fail("MODE=FWD 실패")
            sp = int(round(float(target_w)))
            data = bytes([sp & 0xFF, (sp >> 8) & 0xFF])
            self.enqueue_exec(CMD_SET_SETPOINT, data, tag=f"[START SETP {sp}W]",
                              callback=step2_setp)

        def step0_host(_b: Optional[bytes]):
            if _b is None:
                return fail("HOST 실패")
            self.enqueue_exec(CMD_SET_CTRL_MODE, bytes([MODE_SET["fwd"]]), tag="[START MODE FWD]",
                              callback=step1_mode)

        self.enqueue_exec(CMD_SET_ACTIVE_CTRL, b"\x02", tag="[START HOST]", callback=step0_host)

    def is_connected(self) -> bool:
        return bool(self.port and self.port.isOpen())

    # ---------- 폴링 on/off ----------
    @Slot(bool)
    def set_process_status(self, should_poll: bool):
        """
        True  -> 폴링 시작
        False -> 폴링 중지 후: 현재 큐를 완전히 비우고, pulsing=0 / RF OFF 실행(큐 넣기)
        """
        if should_poll:
            self._start_power_polling()
            return

        # 폴링 중지
        self._stop_power_polling()
        # 큐 비우기
        self.purge_pending("polling off")

        # 안전 초기화 시퀀스 (응답 미보장 허용)
        self.enqueue_exec(CMD_SET_ACTIVE_CTRL, b"\x02", tag="[SAFE HOST]",
                          allow_no_reply=True, allow_when_closing=True,
                          timeout_ms=ACK_TIMEOUT_MS, callback=lambda _b: None)

        self.enqueue_exec(CMD_SET_PULSING, bytes([0]), tag="[SAFE PULSING 0]",
                          allow_no_reply=True, allow_when_closing=True,
                          timeout_ms=ACK_TIMEOUT_MS, callback=lambda _b: None)

        def _safe_off_cb(_b: Optional[bytes]):
            if self._stop_requested or self._closing:
                self.power_off_finished.emit()

        self.enqueue_exec(CMD_RF_OFF, b"", tag="[SAFE RF OFF]",
                          allow_no_reply=True, allow_when_closing=True,
                          timeout_ms=max(ACK_TIMEOUT_MS, 2500), callback=_safe_off_cb)

    @Slot()
    def stop_process(self):
        """외부 stop: 폴링 off → 큐 비움 → pulsing 0 & RF off enqueue"""
        self._stop_requested = True
        self._want_connected = False
        self.set_process_status(False)

    @Slot()
    def cleanup(self):
        """안전 종료: 폴링 off → 큐 비움 → pulsing 0 & RF off enqueue → 잠시 후 포트 닫기"""
        self._closing = True
        self._want_connected = False
        self.set_process_status(False)
        QTimer.singleShot(200, self.close_port)

    # ---------- 큐 정리 ----------
    @Slot()
    def purge_pending(self, reason: str = "manual purge") -> int:
        """대기/진행 중인 명령을 취소하고 큐를 비웁니다. (인플라이트는 즉시 중단 불가)"""
        purged = 0
        if self._gap_timer: self._gap_timer.stop()

        if self._inflight is not None:
            try:
                self._inflight.callback(None)
            except Exception:
                pass
            self._inflight = None
            purged += 1

        while self._cmd_q:
            c = self._cmd_q.popleft()
            purged += 1
            try: c.callback(None)
            except Exception: pass

        self.status_message.emit("RFPulse", f"명령 {purged}개 폐기 ({reason})")
        return purged

    # ---------- 유틸: 즉시 1회 리드(큐 사용) ----------
    @Slot()
    def poll_once(self):
        """원할 때 즉시 한 번 FWD/REF를 큐로 읽고 UI 갱신"""
        tmp: dict = {}

        def _f(b: Optional[bytes]):
            if b is None: return
            tmp['f'] = float(_u16le(b,0) if len(b)>=2 else 0)

        def _r(b: Optional[bytes]):
            if b is None: return
            tmp['r'] = float(_u16le(b,0) if len(b)>=2 else 0)
            f = tmp.get('f', self._last_forward_w)
            r = tmp.get('r', self._last_reflected_w)
            if f is not None and r is not None:
                self._last_forward_w = f
                self._last_reflected_w = r
                self.update_rf_status_display.emit(f, r)

        # WAKE → FWD → REF
        self.enqueue_query(CMD_REPORT_STATUS, b"", tag="[ONCE WAKE]",
                           timeout_ms=POLL_QUERY_TIMEOUT_MS,
                           callback=lambda _b: self.enqueue_query(
                               CMD_REPORT_FORWARD, b"", tag="[ONCE FWD]",
                               timeout_ms=POLL_QUERY_TIMEOUT_MS, callback=_f))
        self.enqueue_query(CMD_REPORT_REFLECTED, b"", tag="[ONCE REF]",
                           timeout_ms=POLL_QUERY_TIMEOUT_MS, callback=_r)
