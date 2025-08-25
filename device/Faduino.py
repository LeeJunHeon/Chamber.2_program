# -*- coding: utf-8 -*-
"""
Faduino.py — PyQt6 QSerialPort(완전 비동기) 기반 Faduino 컨트롤러 (스레드 안전 리팩터링)

변경 핵심:
  - __init__ 에서 QSerialPort/QTimer 생성하지 않음(모두 None으로). ➜ 타겟 스레드에서 생성.
  - connect_faduino() 진입 시 _ensure_serial_created/_ensure_timers_created 호출.
  - 모든 타이머/시리얼 접근은 None-가드.
  - cleanup() 은 stop 후 deleteLater(), 시리얼도 close + deleteLater().
  - 재연결 예약 중복 방지(_reconnect_pending).
"""

from __future__ import annotations
import time
import traceback
from collections import deque
from dataclasses import dataclass
from typing import Callable, Optional, Deque

from PyQt6.QtCore import QObject, QTimer, QIODeviceBase, pyqtSignal as Signal, pyqtSlot as Slot
from PyQt6.QtSerialPort import QSerialPort, QSerialPortInfo

from lib.config import (
    FADUINO_PORT, FADUINO_BAUD, BUTTON_TO_PIN,
    RF_PARAM_ADC_TO_WATT, RF_OFFSET_ADC_TO_WATT,
    DC_PARAM_ADC_TO_VOLT, DC_OFFSET_ADC_TO_VOLT,
    DC_PARAM_ADC_TO_AMP,  DC_OFFSET_ADC_TO_AMP,
    ADC_FULL_SCALE, ADC_INPUT_VOLT, RF_WATT_PER_VOLT,
    DAC_FULL_SCALE, FADUINO_POLLING_INTERVAL_MS,
    FADUINO_WATCHDOG_INTERVAL_MS, FADUINO_TIMEOUT_MS,
    FADUINO_GAP_MS, FADUINO_RECONNECT_BACKOFF_START_MS,
    FADUINO_RECONNECT_BACKOFF_MAX_MS, DEBUG_PRINT,
    CLEAN_TIMEOUT
)

@dataclass
class Command:
    cmd_str: str
    callback: Callable[[Optional[str]], None]
    timeout_ms: int
    gap_ms: int
    tag: str
    retries_left: int
    allow_no_reply: bool

class FaduinoController(QObject):
    status_message = Signal(str, str)
    rf_power_updated = Signal(float, float)
    dc_power_updated = Signal(float, float, float)
    command_confirmed = Signal(str)
    command_failed = Signal(str, str)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.debug_print = DEBUG_PRINT
        self._closing = False
        self._send_spin = False

        self._last_error_time = 0.0
        self._error_debounce_s = 1.0

        # ❗ 지연 생성: 전부 None으로 시작
        self.serial_faduino: Optional[QSerialPort] = None

        self._cmd_timer: Optional[QTimer] = None
        self._gap_timer: Optional[QTimer] = None
        self.polling_timer: Optional[QTimer] = None
        self._watchdog: Optional[QTimer] = None

        self._rx = bytearray()
        self._RX_MAX = 16 * 1024
        self._LINE_MAX = 512
        self._overflow_count = 0

        self._cmd_q: Deque[Command] = deque()
        self._inflight: Optional[Command] = None

        self._want_connected = False
        self._reconnect_backoff_ms = FADUINO_RECONNECT_BACKOFF_START_MS
        self._reconnect_pending = False

        self.expected_relay_mask = 0
        self._is_first_poll = True
        self.is_rf_active = False
        self.is_dc_active = False
        self.rf_forward = 0.0
        self.rf_reflected = 0.0
        self.dc_voltage = 0.0
        self.dc_current = 0.0

    # ---------- 내부 헬퍼(지연 생성) ----------
    def _ensure_serial_created(self):
        if self.serial_faduino is not None:
            return
        self.serial_faduino = QSerialPort(self)                 # 반드시 Faduino 스레드에서 생성
        self.serial_faduino.setBaudRate(FADUINO_BAUD)
        self.serial_faduino.setDataBits(QSerialPort.DataBits.Data8)
        self.serial_faduino.setParity(QSerialPort.Parity.NoParity)
        self.serial_faduino.setStopBits(QSerialPort.StopBits.OneStop)
        self.serial_faduino.setFlowControl(QSerialPort.FlowControl.NoFlowControl)
        self.serial_faduino.readyRead.connect(self._on_ready_read)
        self.serial_faduino.errorOccurred.connect(self._on_serial_error)

    def _ensure_timers_created(self):
        if self._cmd_timer is None:
            self._cmd_timer = QTimer(self)
            self._cmd_timer.setSingleShot(True)
            self._cmd_timer.timeout.connect(self._on_cmd_timeout)
        if self._gap_timer is None:
            self._gap_timer = QTimer(self)
            self._gap_timer.setSingleShot(True)
            self._gap_timer.timeout.connect(self._dequeue_and_send)
        if self.polling_timer is None:
            self.polling_timer = QTimer(self)
            self.polling_timer.setInterval(FADUINO_POLLING_INTERVAL_MS)
            self.polling_timer.timeout.connect(self._enqueue_poll_cycle)
        if self._watchdog is None:
            self._watchdog = QTimer(self)
            self._watchdog.setInterval(FADUINO_WATCHDOG_INTERVAL_MS)
            self._watchdog.timeout.connect(self._watch_connection)

    # ---------- 디버그 ----------
    def _dprint(self, *args):
        if self.debug_print:
            try:
                print(*args, flush=True)
            except Exception:
                pass

    # ---------- 연결/해제 & 워치독 ----------
    @Slot()
    def connect_faduino(self) -> bool:
        self._ensure_serial_created()
        self._ensure_timers_created()

        self._want_connected = True
        ok = self._open_port()
        if self._watchdog:
            self._watchdog.start()
        return ok

    def _open_port(self) -> bool:
        if self.serial_faduino and self.serial_faduino.isOpen():
            return True

        # 포트 존재 확인
        available = {p.portName() for p in QSerialPortInfo.availablePorts()}
        if FADUINO_PORT not in available:
            msg = f"{FADUINO_PORT} 존재하지 않음. 사용 가능 포트: {sorted(available)}"
            self.status_message.emit("Faduino", msg); self._dprint(f"[FAD] {msg}")
            return False

        self.serial_faduino.setPortName(FADUINO_PORT)  # type: ignore
        if not self.serial_faduino.open(QIODeviceBase.OpenModeFlag.ReadWrite):  # type: ignore
            msg = f"{FADUINO_PORT} 연결 실패: {self.serial_faduino.errorString()}"  # type: ignore
            self.status_message.emit("Faduino", msg); self._dprint(f"[FAD] {msg}")
            return False

        # 라인 제어/버퍼 초기화
        self.serial_faduino.setDataTerminalReady(True)   # type: ignore
        self.serial_faduino.setRequestToSend(False)      # type: ignore
        self.serial_faduino.clear(QSerialPort.Direction.AllDirections)  # type: ignore
        self._rx.clear()

        # 재연결 상태 초기화
        self._reconnect_backoff_ms = FADUINO_RECONNECT_BACKOFF_START_MS
        self._reconnect_pending = False

        msg = f"{FADUINO_PORT} 연결 성공 (QSerialPort)"
        self.status_message.emit("Faduino", msg); self._dprint(f"[FAD] {msg}")
        return True

    def _watch_connection(self):
        if not self._want_connected:
            return
        if self.serial_faduino and self.serial_faduino.isOpen():
            return
        if self._reconnect_pending:
            self._dprint("[RECON] already scheduled")
            return

        self._reconnect_pending = True
        self.status_message.emit("Faduino", f"재연결 시도... ({self._reconnect_backoff_ms} ms)")
        self._dprint(f"[FAD] reconnect in {self._reconnect_backoff_ms}ms")
        QTimer.singleShot(self._reconnect_backoff_ms, self._try_reconnect)

    def _try_reconnect(self):
        self._reconnect_pending = False
        if not self._want_connected:
            return
        if self.serial_faduino and self.serial_faduino.isOpen():
            return

        if self._open_port():
            self.status_message.emit("Faduino", "재연결 성공. 대기 중 명령 재개.")
            self._dprint("[FAD] reconnected")
            QTimer.singleShot(0, self._dequeue_and_send)
            self._reconnect_backoff_ms = FADUINO_RECONNECT_BACKOFF_START_MS
            return

        self._reconnect_backoff_ms = min(self._reconnect_backoff_ms * 2, FADUINO_RECONNECT_BACKOFF_MAX_MS)

    @Slot()
    def cleanup(self):
        """안전 종료: 타이머/큐/포트 정리 + (선택) 출력 OFF 시도"""
        if self._closing:
            self._dprint("[CLOSE] cleanup already in progress")
            return
        self._closing = True
        self._want_connected = False
        self._reconnect_pending = False

        # inflight 취소 통지
        if self._inflight is not None:
            try:
                if self._cmd_timer: self._cmd_timer.stop()
                cmd = self._inflight
                self._inflight = None
                self._safe_callback(cmd.callback, None)
            except Exception:
                pass

        # 대기 큐 취소 통지
        try:
            while self._cmd_q:
                cmd = self._cmd_q.popleft()
                self._safe_callback(cmd.callback, None)
        except Exception:
            pass

        # 출력 OFF(무응답 허용). 타이머가 있어야 enqueue-전송이 가능
        self._ensure_timers_created()
        for pin in range(8):
            self.enqueue(f"R,{pin},0", lambda _l: None,
                         timeout_ms=CLEAN_TIMEOUT, gap_ms=FADUINO_GAP_MS,
                         tag=f"[CLEAN R {pin}]", retries_left=0, allow_no_reply=True,
                         allow_when_closing=True)
        self.enqueue("W,0", lambda _l: None,
                     timeout_ms=CLEAN_TIMEOUT, gap_ms=FADUINO_GAP_MS,
                     tag="[CLEAN W]", retries_left=0, allow_no_reply=True,
                     allow_when_closing=True)
        self.enqueue("D,0", lambda _l: None,
                     timeout_ms=CLEAN_TIMEOUT, gap_ms=FADUINO_GAP_MS,
                     tag="[CLEAN D]", retries_left=0, allow_no_reply=True,
                     allow_when_closing=True)

        # 잠깐 기다렸다가 포트 닫고 객체 파기
        QTimer.singleShot(200, self._close_now)

    def _close_now(self):
        # 타이머 정지/파기
        for tattr in ("_cmd_timer", "_gap_timer", "polling_timer", "_watchdog"):
            t = getattr(self, tattr)
            if t:
                try:
                    t.stop()
                except Exception:
                    pass
                t.deleteLater()
                setattr(self, tattr, None)

        # 시리얼 닫고 파기
        if self.serial_faduino:
            try:
                if self.serial_faduino.isOpen():
                    self.serial_faduino.close()
            except Exception:
                pass
            self.serial_faduino.deleteLater()
            self.serial_faduino = None

        self._rx.clear()
        self.status_message.emit("Faduino", "연결 종료")

    # ---------- 시리얼 이벤트 ----------
    def _on_serial_error(self, err: QSerialPort.SerialPortError):
        if err == QSerialPort.SerialPortError.NoError or self._closing:
            return

        now = time.monotonic()
        if now - self._last_error_time < self._error_debounce_s:
            self._dprint("[ERR] debounced serial error")
            return
        self._last_error_time = now

        err_name = getattr(err, "name", str(err))
        err_code = getattr(err, "value", "?")
        serr = self.serial_faduino.errorString() if self.serial_faduino else ""
        msg = f"시리얼 오류: {serr} (err={err_name}/{err_code})"
        self.status_message.emit("Faduino", msg); self._dprint(f"[ERR] {msg}")

        # 진행 중 명령 되돌리기
        if self._inflight is not None:
            cmd = self._inflight
            if self._cmd_timer: self._cmd_timer.stop()
            self._inflight = None
            if cmd.retries_left > 0:
                cmd.retries_left -= 1
                self._cmd_q.appendleft(cmd)
            else:
                self._safe_callback(cmd.callback, None)

        # 포트 닫기
        if self.serial_faduino and self.serial_faduino.isOpen():
            try:
                self.serial_faduino.close()
            except Exception:
                pass

        # 타이머/버퍼 정리
        if self._gap_timer: self._gap_timer.stop()
        self._rx.clear()

        # 다음 틱에 워치독이 재연결 시도
        QTimer.singleShot(0, self._watch_connection)

    def _on_ready_read(self):
        if not (self.serial_faduino and self.serial_faduino.isOpen()):
            return
        ba = self.serial_faduino.readAll()
        if ba.isEmpty():
            return

        # 1) 누적
        self._rx.extend(bytes(ba))

        # 2) RX 오버플로우 보호
        if len(self._rx) > self._RX_MAX:
            del self._rx[:-self._RX_MAX]
            self._overflow_count += 1
            if self._overflow_count % 5 == 1:
                self.status_message.emit("Faduino", f"수신 버퍼 과다(RX>{self._RX_MAX}); 최근 {self._RX_MAX}B만 보존.")
            self._dprint(f"[WARN] RX overflow: keep tail {len(self._rx)}B")

        # 3) 줄 단위 파싱
        while True:
            i_cr = self._rx.find(b'\r')
            i_lf = self._rx.find(b'\n')
            if i_cr == -1 and i_lf == -1:
                break

            idx = i_cr if i_lf == -1 else (i_lf if i_cr == -1 else min(i_cr, i_lf))
            line_bytes = self._rx[:idx]

            drop = idx + 1
            if drop < len(self._rx):
                ch = self._rx[idx]; nxt = self._rx[idx+1]
                if (ch == 13 and nxt == 10) or (ch == 10 and nxt == 13):
                    drop += 1
            del self._rx[:drop]

            if len(line_bytes) > self._LINE_MAX:
                self._dprint(f"[WARN] RX line too long (+{len(line_bytes)-self._LINE_MAX}B), truncating")
                line_bytes = line_bytes[:self._LINE_MAX]

            try:
                line = line_bytes.decode('ascii', errors='ignore').strip()
            except Exception:
                line = None

            if not line:
                continue

            # 에코 스킵
            if self._inflight:
                sent = (self._inflight.cmd_str or "").strip()
                if line == sent:
                    self._dprint(f"[RECV] echo skipped: {repr(line)}")
                    continue

            self._dprint(f"[RECV] {repr(line)}")
            self._finish_command(line)
            break

        while self._rx[:1] in (b'\r', b'\n'):
            del self._rx[0:1]

    # ---------- 명령 큐 ----------
    def enqueue(self, cmd_str: str, on_reply: Callable[[Optional[str]], None],
                timeout_ms: int = FADUINO_TIMEOUT_MS, gap_ms: int = FADUINO_GAP_MS,
                tag: str = "", retries_left: int = 5,
                allow_no_reply: bool = False, allow_when_closing: bool = False):

        # 종료 중이면 외부 enqueue 차단(단, cleanup 내부 허용)
        if self._closing and not allow_when_closing:
            self._dprint("[CLOSE] enqueue blocked")
            return

        # 타이머가 아직 없다면 만들어 둠
        self._ensure_timers_created()

        if not cmd_str.endswith('\r'):
            cmd_str += '\r'
        self._cmd_q.append(Command(cmd_str, on_reply, timeout_ms, gap_ms, tag, retries_left, allow_no_reply))

        # 인플라이트 없고, 갭타이머가 없거나 비활성일 때만 송신 예약
        if (self._inflight is None) and (not (self._gap_timer and self._gap_timer.isActive())):
            QTimer.singleShot(0, self._dequeue_and_send)

    def _dequeue_and_send(self):
        if self._inflight is not None or not self._cmd_q:
            return
        if not (self.serial_faduino and self.serial_faduino.isOpen()):
            return
        if self._gap_timer and self._gap_timer.isActive():
            return
        if self._send_spin:
            self._dprint("[GUARD] _dequeue_and_send re-enter blocked")
            return
        self._send_spin = True

        try:
            cmd = self._cmd_q.popleft()
            self._inflight = cmd
            self._rx.clear()

            self._dprint(f"[SEND] {cmd.cmd_str.strip()} (tag={cmd.tag})")
            self.status_message.emit("Faduino > 전송", f"{cmd.tag or ''} {cmd.cmd_str.strip()}".strip())

            payload = cmd.cmd_str.encode('ascii')
            n = self.serial_faduino.write(payload)  # type: ignore
            n = int(n) if isinstance(n, int) or hasattr(n, "__int__") else -1
            if n <= 0:
                raise IOError(f"serial write returned {n}")

            total = n
            if total != len(payload):
                remain = payload[total:]
                m = self.serial_faduino.write(remain)  # type: ignore
                m = int(m) if isinstance(m, int) or hasattr(m, "__int__") else -1
                if m > 0:
                    total += m
                if total != len(payload):
                    raise IOError(f"partial write: queued {total}/{len(payload)} bytes")

            self.serial_faduino.flush()  # type: ignore

            if self._cmd_timer:
                self._cmd_timer.stop()
                self._cmd_timer.start(cmd.timeout_ms)

        except Exception as e:
            self._dprint(f"[ERROR] Send failed: {e}")
            failed = self._inflight
            self._inflight = None
            if self._cmd_timer: self._cmd_timer.stop()
            if failed:
                self._cmd_q.appendleft(failed)
            try:
                if not (self.serial_faduino and self.serial_faduino.isOpen()):
                    QTimer.singleShot(0, self._try_reconnect)
                else:
                    gap_ms = failed.gap_ms if failed else 100
                    if self._gap_timer: self._gap_timer.start(gap_ms)
                    QTimer.singleShot(gap_ms + 1, self._dequeue_and_send)
            except Exception as ee:
                self._dprint(f"[WARN] reconnect/retry schedule failed: {ee}")
            self.status_message.emit("Faduino", f"전송 오류: {e}")
            return
        finally:
            self._send_spin = False

    def _on_cmd_timeout(self):
        if self._inflight and self._inflight.allow_no_reply:
            self._dprint("[NOTE] no-reply command; proceed after write")
        else:
            self._dprint("[TIMEOUT] command response timed out")
        self._finish_command(None)

    def _finish_command(self, line: Optional[str]):
        if self._inflight is None:
            return
        cmd = self._inflight
        if self._cmd_timer: self._cmd_timer.stop()
        self._inflight = None

        if line is None:
            if cmd.allow_no_reply:
                self._safe_callback(cmd.callback, None)
                if self._gap_timer: self._gap_timer.start(cmd.gap_ms)
                return
            if cmd.retries_left > 0:
                cmd.retries_left -= 1
                self._cmd_q.appendleft(cmd)
                if self.serial_faduino and self.serial_faduino.isOpen():
                    self.serial_faduino.close()
                QTimer.singleShot(0, self._try_reconnect)
                return
            self._safe_callback(cmd.callback, None)
            if self._gap_timer: self._gap_timer.start(cmd.gap_ms)
            return

        self._safe_callback(cmd.callback, (line or '').strip())
        if self._gap_timer: self._gap_timer.start(cmd.gap_ms)

    # ---------- 폴링 ----------
    @Slot(bool)
    def set_process_status(self, should_poll: bool):
        self._ensure_timers_created()
        if should_poll:
            if self.polling_timer and not self.polling_timer.isActive():
                self.status_message.emit("Faduino", "공정 감시 폴링 시작"); self._dprint("[RUN] POLL START")
                self.polling_timer.start()
        else:
            if self.polling_timer and self.polling_timer.isActive():
                self.polling_timer.stop()
                self.status_message.emit("Faduino", "공정 감시 폴링 중지"); self._dprint("[RUN] POLL STOP")

    def _enqueue_poll_cycle(self):
        def on_s(line: Optional[str]):
            p = self._parse_ok_and_compute(line or "")
            if p and p.get("type") == "ERROR":
                self.command_failed.emit("Faduino", p.get("msg", "ERROR"))
                return
            if not p or p.get("type") != "OK_S":
                return
            try:
                relay_mask = p["relay_mask"]
                if self._is_first_poll:
                    self.expected_relay_mask = relay_mask
                    self._is_first_poll = False
                    self.status_message.emit("Faduino", f"초기 릴레이 상태 동기화 완료: {relay_mask}")
                elif relay_mask != self.expected_relay_mask:
                    msg = f"릴레이 상태 불일치! 예상: {self.expected_relay_mask}, 실제: {relay_mask}"
                    self.status_message.emit("Faduino(경고)", msg)
                    self.command_failed.emit("Faduino", f"Relay 상태 확인 {msg}")
                if self.is_rf_active and "rf" in p:
                    self._update_rf(*p["rf"])
                if self.is_dc_active and "dc" in p:
                    self._update_dc(*p["dc"])
            except Exception:
                pass
        self.enqueue('S', on_s, timeout_ms=FADUINO_TIMEOUT_MS, gap_ms=FADUINO_GAP_MS, tag='[POLL S]')

    # ---------- 공개 API ----------
    @Slot(str, bool)
    def handle_named_command(self, name: str, state: bool):
        if name not in BUTTON_TO_PIN:
            self.status_message.emit("Faduino", f"알 수 없는 버튼명: {name}")
            return
        self.set_relay(BUTTON_TO_PIN[name], state)

    @Slot(int, bool)
    def set_relay(self, pin: int, state: bool):
        cmd = f"R,{pin},{1 if state else 0}"
        def on_reply(line: Optional[str], pin=pin, state=state):
            if (line or '').strip() == 'ACK_R':
                if state: self.expected_relay_mask |= (1 << pin)
                else:     self.expected_relay_mask &= ~(1 << pin)
                self.command_confirmed.emit(f"R,{pin},{1 if state else 0}")
                self.status_message.emit("Faduino", f"Relay({pin}) → {'ON' if state else 'OFF'}")
            else:
                self.command_failed.emit("R", f"Relay({pin}) 응답 불일치: {repr(line)}")
        self.enqueue(cmd, on_reply, timeout_ms=FADUINO_TIMEOUT_MS, gap_ms=FADUINO_GAP_MS, tag=f'[R {pin}]')

    @Slot(int)
    def set_rf_power(self, value: int):
        v = self._clamp_dac(value)
        cmd = f"W,{v}"
        def on_reply(line: Optional[str], v=v):
            if (line or '').strip() == 'ACK_W':
                self.command_confirmed.emit("W")
                self.status_message.emit("Faduino", f"RF DAC = {v}")
            else:
                self.command_failed.emit("W", f"응답 불일치: {repr(line)}")
        self.enqueue(cmd, on_reply, timeout_ms=FADUINO_TIMEOUT_MS, gap_ms=FADUINO_GAP_MS, tag='[W]')

    @Slot(int)
    def set_dc_power(self, value: int):
        v = self._clamp_dac(value)
        cmd = f"D,{v}"
        def on_reply(line: Optional[str], v=v):
            if (line or '').strip() == 'ACK_D':
                self.command_confirmed.emit("D")
                self.status_message.emit("Faduino", f"DC DAC = {v}")
            else:
                self.command_failed.emit("D", f"응답 불일치: {repr(line)}")
        self.enqueue(cmd, on_reply, timeout_ms=FADUINO_TIMEOUT_MS, gap_ms=FADUINO_GAP_MS, tag='[D]')

    @Slot(int)
    def set_dc_power_unverified(self, value: int):
        v = self._clamp_dac(value)
        self.enqueue(f"D,{v}", lambda _l: None, timeout_ms=FADUINO_TIMEOUT_MS, gap_ms=FADUINO_GAP_MS,
                     tag='[Du]', allow_no_reply=True)

    @Slot(int)
    def set_rf_power_unverified(self, value: int):
        v = self._clamp_dac(value)
        self.enqueue(f"W,{v}", lambda _l: None, timeout_ms=FADUINO_TIMEOUT_MS, gap_ms=FADUINO_GAP_MS,
                     tag='[Wu]', allow_no_reply=True)

    @Slot()
    def force_status_read(self):
        def on_s(line: Optional[str]):
            p = self._parse_ok_and_compute(line or "")
            if p and p.get("type") == "ERROR":
                self.command_failed.emit("Faduino", p.get("msg", "ERROR")); return
            if not p or p.get("type") != "OK_S":
                return
            try:
                relay_mask = p["relay_mask"]
                if self._is_first_poll:
                    self.expected_relay_mask = relay_mask; self._is_first_poll = False
                    self.status_message.emit("Faduino", f"초기 릴레이 상태 동기화 완료: {relay_mask}")
                elif relay_mask != self.expected_relay_mask:
                    msg = f"릴레이 상태 불일치! 예상: {self.expected_relay_mask}, 실제: {relay_mask}"
                    self.status_message.emit("Faduino(경고)", msg)
                    self.command_failed.emit("Faduino", f"Relay 상태 확인 {msg}")
                if self.is_rf_active and "rf" in p:
                    self._update_rf(*p["rf"])
                if self.is_dc_active and "dc" in p:
                    self._update_dc(*p["dc"])
            except Exception:
                pass
        self.enqueue('S', on_s, timeout_ms=FADUINO_TIMEOUT_MS, gap_ms=FADUINO_GAP_MS, tag='[FORCE S]')

    @Slot()
    def force_rf_read(self):
        def on_r(line: Optional[str]):
            p = self._parse_ok_and_compute(line or "")
            if p and p.get("type") == "ERROR":
                self.command_failed.emit("Faduino", p.get("msg", "ERROR")); return
            if not p or p.get("type") != "OK_r":
                return
            try:
                if self.is_rf_active and "rf" in p:
                    self._update_rf(*p["rf"])
            except Exception:
                pass
        self.enqueue('r', on_r, timeout_ms=FADUINO_TIMEOUT_MS, gap_ms=FADUINO_GAP_MS, tag='[FORCE r]')

    @Slot()
    def force_dc_read(self):
        def on_d(line: Optional[str]):
            p = self._parse_ok_and_compute(line or "")
            if p and p.get("type") == "ERROR":
                self.command_failed.emit("Faduino", p.get("msg", "ERROR")); return
            if not p or p.get("type") != "OK_d":
                return
            try:
                if self.is_dc_active and "dc" in p:
                    self._update_dc(*p["dc"])
            except Exception:
                pass
        self.enqueue('d', on_d, timeout_ms=FADUINO_TIMEOUT_MS, gap_ms=FADUINO_GAP_MS, tag='[FORCE d]')

    @Slot()
    def force_pin_read(self):
        def on_p(line: Optional[str]):
            p = self._parse_ok_and_compute(line or "")
            if p and p.get("type") == "ERROR":
                self.command_failed.emit("Faduino", p.get("msg", "ERROR")); return
            if not p or p.get("type") != "OK_P":
                return
            try:
                relay_mask = p["relay_mask"]
                if self._is_first_poll:
                    self.expected_relay_mask = relay_mask; self._is_first_poll = False
                    self.status_message.emit("Faduino", f"초기 릴레이 상태 동기화 완료: {relay_mask}")
                elif relay_mask != self.expected_relay_mask:
                    msg = f"릴레이 상태 불일치! 예상: {self.expected_relay_mask}, 실제: {relay_mask}"
                    self.status_message.emit("Faduino(경고)", msg)
                    self.command_failed.emit("Faduino", f"Relay 상태 확인 {msg}")
            except Exception:
                pass
        self.enqueue('P', on_p, timeout_ms=FADUINO_TIMEOUT_MS, gap_ms=FADUINO_GAP_MS, tag='[FORCE P]')

    # ---------- 파싱/계산 ----------
    def _parse_ok_and_compute(self, response: str):
        s = (response or "").strip()
        if s.startswith("OK_S,"):
            parts = s.split(",")
            if len(parts) != 6: return None
            try:
                relay_mask = int(parts[1])
                rf_for, rf_ref = self._compute_rf(parts[2], parts[3])
                dc_p, dc_v, dc_c = self._compute_dc(parts[4], parts[5])
                return {"type":"OK_S","relay_mask":relay_mask,"rf":(rf_for,rf_ref),"dc":(dc_p,dc_v,dc_c)}
            except Exception:
                return None
        if s.startswith("OK_P,"):
            parts = s.split(",")
            if len(parts) != 2: return None
            try:
                return {"type":"OK_P","relay_mask":int(parts[1])}
            except Exception:
                return None
        if s.startswith("OK_r,"):
            parts = s.split(",")
            if len(parts) != 3: return None
            try:
                rf_for, rf_ref = self._compute_rf(parts[1], parts[2])
                return {"type":"OK_r","rf":(rf_for,rf_ref)}
            except Exception:
                return None
        if s.startswith("OK_d,"):
            parts = s.split(",")
            if len(parts) != 3: return None
            try:
                dc_p, dc_v, dc_c = self._compute_dc(parts[1], parts[2])
                return {"type":"OK_d","dc":(dc_p,dc_v,dc_c)}
            except Exception:
                return None
        if s in ("ACK_R","ACK_W","ACK_D"):
            return {"type":s}
        if s.startswith("ERROR"):
            return {"type":"ERROR","msg":s}
        return None

    def _compute_rf(self, rf_for_raw, rf_ref_raw):
        rf_for_raw = float(rf_for_raw); rf_ref_raw = float(rf_ref_raw)
        rf_for = max(0.0, (RF_PARAM_ADC_TO_WATT * rf_for_raw) + RF_OFFSET_ADC_TO_WATT)
        rf_ref_v = (rf_ref_raw / ADC_FULL_SCALE) * ADC_INPUT_VOLT
        rf_ref = max(0.0, rf_ref_v * RF_WATT_PER_VOLT)
        return rf_for, rf_ref

    def _compute_dc(self, dc_v_raw, dc_c_raw):
        dc_v_raw = float(dc_v_raw); dc_c_raw = float(dc_c_raw)
        dc_v = max(0.0, (DC_PARAM_ADC_TO_VOLT * dc_v_raw) + DC_OFFSET_ADC_TO_VOLT)
        dc_c = max(0.0, (DC_PARAM_ADC_TO_AMP  * dc_c_raw) + DC_OFFSET_ADC_TO_AMP)
        dc_p = dc_v * dc_c
        return dc_p, dc_v, dc_c

    # ---------- 상태 업데이트 ----------
    def _update_rf(self, rf_for, rf_ref):
        self.rf_forward, self.rf_reflected = rf_for, rf_ref
        self.rf_power_updated.emit(rf_for, rf_ref)

    def _update_dc(self, dc_p, dc_v, dc_c):
        self.dc_voltage, self.dc_current = dc_v, dc_c
        self.dc_power_updated.emit(dc_p, dc_v, dc_c)

    @Slot(bool)
    def on_rf_state_changed(self, is_active: bool):
        self.is_rf_active = is_active
        self.status_message.emit("Faduino", f"RF 컨트롤러 상태 감지: {'활성' if is_active else '비활성'}")

    @Slot(bool)
    def on_dc_state_changed(self, is_active: bool):
        self.is_dc_active = is_active
        self.status_message.emit("Faduino", f"DC 컨트롤러 상태 감지: {'활성' if is_active else '비활성'}")

    # ---------- 유틸 ----------
    def _clamp_dac(self, value: int) -> int:
        try: v = int(round(value))
        except Exception: v = 0
        if v < 0: v = 0
        if v > DAC_FULL_SCALE: v = DAC_FULL_SCALE
        return v

    def _safe_callback(self, callback: Callable, *args):
        try:
            callback(*args)
        except Exception as e:
            self._dprint(traceback.format_exc())
            self._dprint(f"[ERROR] Callback failed: {e}")
            self.status_message.emit("Faduino", f"콜백 오류: {e}")
