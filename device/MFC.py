# -*- coding: utf-8 -*-
"""
MFC.py — PyQt6 QSerialPort(완전 비동기) 기반 MFC 컨트롤러

핵심:
  - QSerialPort + readyRead 시그널 → UI/이벤트 루프 블로킹 없음
  - 단일 명령 큐(타임아웃/재시도/인터커맨드 간격) → 폴링·검증 충돌 제거
  - 연결 워치독 + 자동 재연결(지수 백오프) → 공정 중간 끊김에 강함
  - 스케일 일관화: 전송(곱), 표시(나눔), 안정화·비교는 장비 단위
  - 멀티 장비를 위해 인스턴스 독립(포트/큐/타이머 분리)
  - ⚠️ 중요: QSerialPort/타이머는 컨트롤러가 속한 스레드에서 지연 생성
"""

from __future__ import annotations
import traceback
from collections import deque
from dataclasses import dataclass
from typing import Deque, Callable, Optional
import re

from PyQt6.QtCore import QObject, QTimer, QIODeviceBase, pyqtSignal as Signal, pyqtSlot as Slot
from PyQt6.QtSerialPort import QSerialPort, QSerialPortInfo

@dataclass
class Command:
    cmd_str: str
    callback: Callable[[Optional[str]], None]
    timeout_ms: int
    gap_ms: int
    tag: str
    retries_left: int
    allow_no_reply: bool

from lib.config import (
    MFC_PORT,
    MFC_BAUD,
    MFC_COMMANDS,
    FLOW_ERROR_TOLERANCE,
    FLOW_ERROR_MAX_COUNT,
    MFC_SCALE_FACTORS,
    MFC_POLLING_INTERVAL_MS,
    MFC_STABILIZATION_INTERVAL_MS,
    MFC_WATCHDOG_INTERVAL_MS,
    MFC_RECONNECT_BACKOFF_START_MS,
    MFC_RECONNECT_BACKOFF_MAX_MS,
    MFC_TIMEOUT,
    MFC_GAP_MS,
    MFC_DELAY_MS,
    MFC_DELAY_MS_VALVE,
    DEBUG_PRINT
    )

class MFCController(QObject):
    # --- 시그널 (기존 시그널명 유지) ---
    status_message   = Signal(str, str)
    update_flow      = Signal(str, float)
    update_pressure  = Signal(str)
    command_failed   = Signal(str, str)
    command_confirmed= Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.debug_print = DEBUG_PRINT

        # (1) 시리얼/타이머는 '지연 생성' — 스레드 안에서만 생성되도록 전부 None
        self.serial_mfc: Optional[QSerialPort] = None

        self._cmd_timer: Optional[QTimer] = None
        self._gap_timer: Optional[QTimer] = None
        self.polling_timer: Optional[QTimer] = None
        self.stabilization_timer: Optional[QTimer] = None
        self._watchdog: Optional[QTimer] = None

        # (2) 수신/파싱 버퍼
        self._rx = bytearray()
        self._RX_MAX = 16 * 1024
        self._LINE_MAX = 512
        self._overflow_count = 0

        # (3) 명령 큐/상태
        self._cmd_q: Deque[Command] = deque()
        self._inflight: Optional[Command] = None
        self._send_spin = False  # 재진입 가드

        # (4) 연결/재연결 상태
        self._want_connected = False
        self._reconnect_backoff_ms = MFC_RECONNECT_BACKOFF_START_MS
        self._reconnect_pending = False

        # (5) 런타임 상태
        self._is_aborted = False
        self.current_cmd = ""
        self.current_params: dict = {}
        self.retry_attempts = 0

        # (6) 가스/스케일/모니터링
        self.gas_map = {1: "Ar", 2: "O2", 3: "N2"}
        self.last_setpoints = {1: 0.0, 2: 0.0, 3: 0.0}    # 장비 단위
        self.flow_error_counters = {1: 0, 2: 0, 3: 0}

        # (7) 안정화 상태
        self._stabilizing_channel: Optional[int] = None
        self._stabilizing_target: float = 0.0   # 장비 단위
        self._pending_cmd_for_timer: Optional[str] = None
        self.stabilization_attempts = 0

    # -------------------------------------------------
    # 내부 디버그 출력
    # -------------------------------------------------
    def _dprint(self, *args):
        if self.debug_print:
            try:
                print(*args, flush=True)
            except Exception:
                pass

    # -------------------------------------------------
    # 지연 생성 헬퍼(반드시 MFC 스레드에서 호출)
    # -------------------------------------------------
    def _ensure_serial_created(self):
        if self.serial_mfc is not None:
            return
        self.serial_mfc = QSerialPort(self)
        self.serial_mfc.setBaudRate(MFC_BAUD)
        self.serial_mfc.setDataBits(QSerialPort.DataBits.Data8)
        self.serial_mfc.setParity(QSerialPort.Parity.NoParity)
        self.serial_mfc.setStopBits(QSerialPort.StopBits.OneStop)
        self.serial_mfc.setFlowControl(QSerialPort.FlowControl.NoFlowControl)
        self.serial_mfc.readyRead.connect(self._on_ready_read)
        self.serial_mfc.errorOccurred.connect(self._on_serial_error)

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
            self.polling_timer.setInterval(MFC_POLLING_INTERVAL_MS)
            self.polling_timer.timeout.connect(self._enqueue_poll_cycle)

        if self.stabilization_timer is None:
            self.stabilization_timer = QTimer(self)
            self.stabilization_timer.setInterval(MFC_STABILIZATION_INTERVAL_MS)
            self.stabilization_timer.timeout.connect(self._check_flow_stabilization)

        if self._watchdog is None:
            self._watchdog = QTimer(self)
            self._watchdog.setInterval(MFC_WATCHDOG_INTERVAL_MS)
            self._watchdog.timeout.connect(self._watch_connection)

    # ---------- 연결/해제 & 워치독 ----------
    @Slot()
    def connect_mfc_device(self) -> bool:
        """포트를 열고 워치독 시작. 실패해도 워치독이 재연결 시도."""
        self._ensure_serial_created()
        self._ensure_timers_created()

        self._want_connected = True
        ok = self._open_port()
        if self._watchdog:
            self._watchdog.start()
        return ok

    def _open_port(self) -> bool:
        if self.serial_mfc and self.serial_mfc.isOpen():
            return True

        # 포트 존재 확인
        available = {p.portName() for p in QSerialPortInfo.availablePorts()}
        if MFC_PORT not in available:
            msg = f"{MFC_PORT} 존재하지 않음. 사용 가능 포트: {sorted(available)}"
            self.status_message.emit("MFC", msg); self._dprint(f"[MFC] {msg}")
            return False

        self.serial_mfc.setPortName(MFC_PORT)
        if not self.serial_mfc.open(QIODeviceBase.OpenModeFlag.ReadWrite):
            msg = f"{MFC_PORT} 연결 실패: {self.serial_mfc.errorString()}"
            self.status_message.emit("MFC", msg); self._dprint(f"[MFC] {msg}")
            return False

        # 라인 제어/초기화
        self.serial_mfc.setDataTerminalReady(True)
        self.serial_mfc.setRequestToSend(False)
        self.serial_mfc.clear(QSerialPort.Direction.AllDirections)
        self._rx.clear()

        # 재연결 백오프 초기화
        self._reconnect_backoff_ms = MFC_RECONNECT_BACKOFF_START_MS
        self._reconnect_pending = False

        msg = f"{MFC_PORT} 연결 성공 (PyQt6 QSerialPort)"
        self.status_message.emit("MFC", msg); self._dprint(f"[MFC] {msg}")
        return True

    def _watch_connection(self):
        """주기적으로 연결 상태 확인. 끊겨 있으면 백오프로 재연결."""
        if (not self._want_connected) or (self.serial_mfc and self.serial_mfc.isOpen()):
            return
        if self._reconnect_pending:
            return
        self._reconnect_pending = True
        msg = f"재연결 시도... ({self._reconnect_backoff_ms} ms)"
        self.status_message.emit("MFC", msg); self._dprint(f"[MFC] {msg}")
        QTimer.singleShot(self._reconnect_backoff_ms, self._try_reconnect)

    def _try_reconnect(self):
        self._reconnect_pending = False
        if (not self._want_connected) or (self.serial_mfc and self.serial_mfc.isOpen()):
            return

        if self._open_port():
            msg = "재연결 성공. 대기 중 명령 재개."
            self.status_message.emit("MFC", msg); self._dprint(f"[MFC] {msg}")
            QTimer.singleShot(0, self._dequeue_and_send)
            self._reconnect_backoff_ms = MFC_RECONNECT_BACKOFF_START_MS
            return

        self._reconnect_backoff_ms = min(self._reconnect_backoff_ms * 2, MFC_RECONNECT_BACKOFF_MAX_MS)

    @Slot()
    def cleanup(self):
        """안전 종료: 타이머/큐/포트 정리 (타 스레드 파괴 경고 방지)"""
        self._want_connected = False

        # 진행/대기 명령 콜백 취소 통지
        if self._inflight is not None:
            self._safe_callback(self._inflight.callback, None)
            self._inflight = None
        while self._cmd_q:
            pending = self._cmd_q.popleft()
            self._safe_callback(pending.callback, None)

        # 안정화 상태 리셋
        self._stabilizing_channel = None
        self._stabilizing_target = 0.0
        self._pending_cmd_for_timer = None

        # 타이머들 정지/파기
        for t_attr in ("_cmd_timer", "_gap_timer", "polling_timer", "stabilization_timer", "_watchdog"):
            t = getattr(self, t_attr)
            if t:
                try:
                    t.stop()
                except Exception:
                    pass
                t.deleteLater()
                setattr(self, t_attr, None)

        # 포트 정리
        if self.serial_mfc:
            try:
                if self.serial_mfc.isOpen():
                    self.serial_mfc.close()
                    self.status_message.emit("MFC", "시리얼 포트를 안전하게 닫았습니다.")
                    self._dprint("[MFC] 시리얼 포트를 안전하게 닫았습니다.")
            finally:
                self.serial_mfc.deleteLater()
                self.serial_mfc = None

        self._rx.clear()
        self._reconnect_pending = False

    # ---------- 시리얼 이벤트 ----------
    def _on_serial_error(self, err: QSerialPort.SerialPortError):
        if err == QSerialPort.SerialPortError.NoError:
            return

        err_name = getattr(err, "name", str(err))
        err_code = getattr(err, "value", "?")
        serr = self.serial_mfc.errorString() if self.serial_mfc else ""
        msg = f"시리얼 오류: {serr} (err={err_name}/{err_code})"
        self.status_message.emit("MFC", msg); self._dprint(f"[MFC] {msg}")

        # 진행 중 명령 되돌리기
        if self._inflight is not None:
            cmd = self._inflight
            if self._cmd_timer:
                self._cmd_timer.stop()
            self._inflight = None
            if cmd.retries_left > 0:
                cmd.retries_left -= 1
                self._cmd_q.appendleft(cmd)
            else:
                self._safe_callback(cmd.callback, None)

        # 포트/타이머/버퍼 정리
        if self.serial_mfc and self.serial_mfc.isOpen():
            self.serial_mfc.close()
        if self._gap_timer:
            self._gap_timer.stop()
        self._rx.clear()

        QTimer.singleShot(0, self._watch_connection)

    def _on_ready_read(self):
        """줄(\r/\n) 단위 수신 → 에코 라인은 건너뛰고, 실제 응답 한 줄을 전달"""
        if not (self.serial_mfc and self.serial_mfc.isOpen()):
            return

        ba = self.serial_mfc.readAll()
        if ba.isEmpty():
            return

        # 1) 버퍼 누적
        self._rx.extend(bytes(ba))

        # RX 오버플로우 방지
        if len(self._rx) > self._RX_MAX:
            del self._rx[:-self._RX_MAX]
            self._overflow_count += 1
            if self._overflow_count % 5 == 1:
                try:
                    self.status_message.emit("MFC", f"수신 버퍼 과다(RX>{self._RX_MAX}); 최근 {self._RX_MAX}B만 보존.")
                except Exception:
                    pass
            self._dprint(f"[WARN] RX overflow: keep tail {len(self._rx)}B")

        # 2) 줄 경계(\r/\n) 기준 파싱
        while True:
            i_cr = self._rx.find(b'\r')
            i_lf = self._rx.find(b'\n')
            if i_cr == -1 and i_lf == -1:
                break

            idx = i_cr if i_lf == -1 else (i_lf if i_cr == -1 else min(i_cr, i_lf))
            line_bytes = self._rx[:idx]

            # CRLF/LFCR 같이 제거
            drop = idx + 1
            if drop < len(self._rx):
                ch = self._rx[idx]
                nxt = self._rx[idx + 1]
                if (ch == 13 and nxt == 10) or (ch == 10 and nxt == 13):
                    drop += 1
            del self._rx[:drop]

            # 라인 길이 제한
            if len(line_bytes) > self._LINE_MAX:
                over = len(line_bytes) - self._LINE_MAX
                self._dprint(f"[WARN] RX line too long (+{over}B), truncating")
                line_bytes = line_bytes[:self._LINE_MAX]

            try:
                line = line_bytes.decode('ascii', errors='ignore').strip()
            except Exception:
                line = None

            if not line:
                continue

            # 에코 라인 스킵
            if self._inflight:
                sent_cmd_str = (self._inflight.cmd_str or "").strip()
                if line == sent_cmd_str:
                    self._dprint(f"[RECV] echo skipped: {repr(line)}")
                    continue

            # 유효 응답 1줄만 처리
            self._dprint(f"[RECV] {repr(line)}")
            self._finish_command(line)
            break

        # 앞쪽 연속 CR/LF 정리
        while self._rx[:1] in (b'\r', b'\n'):
            del self._rx[0:1]

    # ---------- 명령 큐 ----------
    def enqueue(self, cmd_str: str, on_reply: Callable[[Optional[str]], None],
                timeout_ms: int = MFC_TIMEOUT, gap_ms: int = MFC_GAP_MS,
                tag: str = "", retries_left: int = 5,
                allow_no_reply: bool = False):
        """명령을 큐에 넣고, inflight 없고 간격타이머가 비활성일 때 전송 예약"""
        if not cmd_str.endswith('\r'):
            cmd_str += '\r'

        self._cmd_q.append(Command(cmd_str, on_reply, timeout_ms, gap_ms, tag, retries_left, allow_no_reply))

        if (self._inflight is None) and (not (self._gap_timer and self._gap_timer.isActive())):
            QTimer.singleShot(0, self._dequeue_and_send)

    def _dequeue_and_send(self):
        if self._inflight is not None or not self._cmd_q:
            return
        if not (self.serial_mfc and self.serial_mfc.isOpen()):
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
            self.status_message.emit("MFC > 전송", f"{cmd.tag or ''} {cmd.cmd_str.strip()}".strip())

            payload = cmd.cmd_str.encode('ascii')
            n = self.serial_mfc.write(payload)
            n_int = int(n) if isinstance(n, int) or hasattr(n, "__int__") else -1
            if n_int <= 0:
                raise IOError(f"serial write returned {n_int}")

            total = n_int
            if total != len(payload):
                remain = payload[total:]
                m = self.serial_mfc.write(remain)
                m_int = int(m) if isinstance(m, int) or hasattr(m, "__int__") else -1
                if m_int > 0:
                    total += m_int
                if total != len(payload):
                    raise IOError(f"partial write: queued {total}/{len(payload)} bytes")

            self.serial_mfc.flush()

            if self._cmd_timer:
                self._cmd_timer.stop()
                self._cmd_timer.start(cmd.timeout_ms)

        except Exception as e:
            self._dprint(f"[ERROR] Send failed: {e}")
            failed = self._inflight
            self._inflight = None
            if self._cmd_timer:
                self._cmd_timer.stop()
            if failed:
                self._cmd_q.appendleft(failed)
            try:
                if not (self.serial_mfc and self.serial_mfc.isOpen()):
                    QTimer.singleShot(0, self._try_reconnect)
                else:
                    gap_ms = failed.gap_ms if failed else 100
                    if self._gap_timer:
                        self._gap_timer.start(gap_ms)
                    QTimer.singleShot(gap_ms + 1, self._dequeue_and_send)
            except Exception as ee:
                self._dprint(f"[WARN] reconnect/retry schedule failed: {ee}")
            self.status_message.emit("MFC", f"전송 오류: {e}")
            return
        finally:
            self._send_spin = False

    def _on_cmd_timeout(self):
        if not self._inflight:
            return
        cmd = self._inflight
        if cmd.allow_no_reply:
            self._dprint("[NOTE] no-reply command; proceeding after write")
        else:
            self._dprint("[TIMEOUT] command response timed out")
        self._finish_command(None)

    def _finish_command(self, line: Optional[str]):
        if self._inflight is None:
            return
        cmd = self._inflight
        if self._cmd_timer:
            self._cmd_timer.stop()
        self._inflight = None

        # 응답 없음
        if line is None:
            if cmd.allow_no_reply:
                self._safe_callback(cmd.callback, None)
                if self._gap_timer:
                    self._gap_timer.start(cmd.gap_ms)
                return
            if cmd.retries_left > 0:
                cmd.retries_left -= 1
                self._cmd_q.appendleft(cmd)
                if self.serial_mfc and self.serial_mfc.isOpen():
                    self.serial_mfc.close()
                self._try_reconnect()
                return
            self._safe_callback(cmd.callback, None)
            if self._gap_timer:
                self._gap_timer.start(cmd.gap_ms)
            return

        # 정상 응답
        self._safe_callback(cmd.callback, (line.strip() if isinstance(line, str) else line))
        if self._gap_timer:
            self._gap_timer.start(cmd.gap_ms)

    # ---------- 폴링 ----------
    @Slot(bool)
    def set_process_status(self, should_poll: bool):
        """공정 시작/중지 시 폴링 ON/OFF (타이머가 준비된 이후에만)"""
        if not self.polling_timer:
            return
        if should_poll:
            if not self.polling_timer.isActive():
                self.status_message.emit("MFC", "주기적 읽기(Polling) 시작"); self._dprint("[RUN] POLL START")
                self.polling_timer.start()
        else:
            if self.polling_timer.isActive():
                self.polling_timer.stop()
                self.status_message.emit("MFC", "주기적 읽기(Polling) 중지"); self._dprint("[RUN] POLL STOP")

    def _enqueue_poll_cycle(self):
        self._read_flow_all_async(tag="[POLL R60]")

        cmdp = MFC_COMMANDS['READ_PRESSURE']
        def on_p(line: Optional[str]):
            line = (line or "").strip()
            if line:
                self.update_pressure.emit(line)
        self.enqueue(cmdp, on_p, timeout_ms=MFC_TIMEOUT, gap_ms=MFC_GAP_MS, tag="[POLL PRESS]")

    # ---------- 상위에서 호출하는 공개 API ----------
    @Slot(str, dict)
    def handle_command(self, cmd: str, params: dict):
        self._is_aborted = False
        self.current_cmd = cmd
        self.current_params = params
        self.retry_attempts = 0

        # (A) FLOW_SET: 스케일 적용 후 SET → 검증
        if cmd == 'FLOW_SET':
            ch = params.get('channel')
            original_ui = float(params.get('value', 0.0))
            sf = MFC_SCALE_FACTORS.get(ch, 1.0)
            scaled = original_ui * sf
            params['value'] = scaled
            self.status_message.emit("MFC", f"Ch{ch} 유량 스케일: {original_ui:.2f}sccm → 장비 {scaled:.2f}")
            self._dprint(f"[INFO] SCALE ch{ch}: {original_ui} -> {scaled}")

            set_cmd = MFC_COMMANDS['FLOW_SET'](channel=ch, value=scaled)
            def after_set(_line):
                self._verify_flow_set_async(ch, scaled)
            self.enqueue(set_cmd, after_set, timeout_ms=MFC_TIMEOUT, gap_ms=MFC_GAP_MS, tag=f"[SET ch{ch}]", allow_no_reply=True)
            return

        # (B) FLOW_ON / (C) FLOW_OFF
        if cmd in ('FLOW_ON', 'FLOW_OFF'):
            ch = params.get('channel')
            self._apply_flow_onoff_with_L0(ch, cmd == 'FLOW_ON')
            return

        # (D) 밸브
        if cmd in ('VALVE_OPEN', 'VALVE_CLOSE'):
            vcmd = MFC_COMMANDS[cmd]
            def after_valve(_line, origin_cmd=cmd):
                self.status_message.emit("MFC", f"밸브 이동 대기 ({MFC_DELAY_MS_VALVE/1000:.0f}초)..."); self._dprint(f"[INFO] wait {MFC_DELAY_MS_VALVE/1000:.0f}s for valve move")
                QTimer.singleShot(MFC_DELAY_MS_VALVE, lambda: self._check_valve_position_async(origin_cmd))
            self.enqueue(vcmd, after_valve, timeout_ms=MFC_TIMEOUT, gap_ms=MFC_GAP_MS, tag=f"[{cmd}]", allow_no_reply=True)
            return

        # (E) SP1/4
        if cmd in ("SP1_SET", "SP1_ON", "SP4_ON"):
            scmd = MFC_COMMANDS[cmd](**params) if params else MFC_COMMANDS[cmd]
            def after_simple(_line, _cmd=cmd):
                self._verify_simple_async(_cmd, params)
            self.enqueue(scmd, after_simple, timeout_ms=MFC_TIMEOUT, gap_ms=MFC_GAP_MS, tag=f"[{cmd}]", allow_no_reply=True)
            return

        # (E0) ZEROING 류
        if cmd == "PS_ZEROING":
            scmd = MFC_COMMANDS["PS_ZEROING"]
            def after_ps(_line):
                self.command_confirmed.emit("PS_ZEROING")
            self.enqueue(scmd, after_ps, timeout_ms=MFC_TIMEOUT, gap_ms=MFC_GAP_MS, tag="[PS_ZEROING]", allow_no_reply=True)
            return

        if cmd == "MFC_ZEROING":
            ch = int(params.get("channel", 1))
            scmd = MFC_COMMANDS["MFC_ZEROING"](channel=ch)
            def after_mfc(_line):
                self.command_confirmed.emit("MFC_ZEROING")
            self.enqueue(scmd, after_mfc, timeout_ms=MFC_TIMEOUT, gap_ms=MFC_GAP_MS, tag=f"[MFC_ZEROING ch{ch}]", allow_no_reply=True)
            return

        # (F) READ
        if cmd in ("READ_FLOW", "READ_PRESSURE"):
            if cmd == "READ_FLOW":
                ch = params.get('channel', 1)
                def on_done(vals):
                    if vals and (1 <= ch <= len(vals)):
                        v_hw = vals[ch-1]
                        sf   = MFC_SCALE_FACTORS.get(ch, 1.0)
                        self.update_flow.emit(self.gas_map.get(ch, f"Ch{ch}"), v_hw / sf)
                        self.command_confirmed.emit("READ_FLOW")
                    else:
                        self.command_failed.emit("READ_FLOW", "R60 파싱 실패/채널 누락")
                self._read_flow_all_async(on_done=on_done, tag=f"[READ R60 ch{ch}]")
            else:
                self._read_pressure_async(tag="[READ_PRESSURE]")
            return

        self.command_failed.emit(cmd, "알 수 없는 명령"); self._dprint(f"[FAIL] unknown cmd: {cmd}")

    # ---------- 파서/유틸 ----------
    def _q_prefixes_for(self, cmd_key: str, ch: int) -> tuple[str, ...]:
        if cmd_key == 'READ_FLOW_SET':  # R65~R68
            return (f"Q{4 + int(ch)}",)
        if cmd_key == 'READ_FLOW_ALL':  # R60
            return ("Q0",)
        return tuple()

    def _parse_q_value_with_prefixes(self, line: str | None, expected_prefixes: tuple[str, ...]) -> float | None:
        s = (line or "").strip()
        for p in expected_prefixes:
            if s.startswith(p):
                m = re.match(r'^' + re.escape(p) + r'\s*([+\-]?\d+(?:\.\d+)?)$', s)
                if m:
                    try: return float(m.group(1))
                    except Exception: return None
                return None
        return None

    def _parse_r60_values(self, line: str | None) -> list[float] | None:
        s = (line or "").strip()
        if not s.startswith("Q0"):
            return None
        nums = re.findall(r'([+\-]?\d+(?:\.\d+)?)', s[2:])
        try: return [float(x) for x in nums]
        except Exception: return None

    # ---------- 비동기 검증/읽기 ----------
    def _verify_flow_set_async(self, ch: int, scaled_value: float,
                               attempt: int = 1, max_attempts: int = 5, delay_ms: int = MFC_DELAY_MS):
        cmd = MFC_COMMANDS['READ_FLOW_SET'](channel=ch)
        expected = self._q_prefixes_for('READ_FLOW_SET', ch)

        def on_reply(line: Optional[str], ch=ch, scaled_value=scaled_value, attempt=attempt):
            line = (line or "").strip()
            val = self._parse_q_value_with_prefixes(line, expected)

            if val is None and line.startswith('Q'):
                self.status_message.emit("DBG", f"[FLOW_SET 검증] 접두사 불일치: 기대 {expected}, 응답 {repr(line)}")
                QTimer.singleShot(MFC_DELAY_MS, lambda: self._verify_flow_set_async(ch, scaled_value, attempt, max_attempts, delay_ms))
                return

            ok = (val is not None) and (abs(val - scaled_value) < 0.1)

            if ok:
                self.last_setpoints[ch] = scaled_value
                sf = MFC_SCALE_FACTORS.get(ch, 1.0)
                msg = f"Ch{ch} 목표 {scaled_value/sf:.2f} sccm 설정 완료."
                self.status_message.emit("MFC < 확인", msg); self._dprint(f"[OK ] {msg}")
                self.command_confirmed.emit("FLOW_SET")
            else:
                if attempt < max_attempts:
                    resend_cmd = MFC_COMMANDS['FLOW_SET'](channel=ch, value=scaled_value)
                    self.enqueue(resend_cmd, lambda _line: None,
                                 timeout_ms=MFC_TIMEOUT, gap_ms=MFC_GAP_MS, tag=f"[RE-SET ch{ch}]", allow_no_reply=True)
                    self.status_message.emit("WARN",
                        f"[FLOW_SET 검증 재시도] ch{ch}: 기대={scaled_value:.2f}, 응답={repr(line)} (시도 {attempt}/{max_attempts})")
                    QTimer.singleShot(delay_ms,
                        lambda: self._verify_flow_set_async(ch, scaled_value, attempt+1, max_attempts, delay_ms))
                else:
                    self.command_failed.emit("FLOW_SET", f"Ch{ch} FLOW_SET 확인 실패"); self._dprint(f"[FAIL] FLOW_SET verify ch{ch}")

        self.enqueue(cmd, on_reply, timeout_ms=MFC_TIMEOUT, gap_ms=MFC_GAP_MS, tag=f"[VERIFY SET ch{ch}]")

    def _apply_flow_onoff_with_L0(self, ch: int, turn_on: bool):
        """R69로 현재 마스크 읽고, 해당 비트만 수정해 L0로 일괄 적용 → 검증/안정화"""
        def _after_r69(line: Optional[str], ch=ch, turn_on=turn_on):
            now_bits = self._parse_r69_bits((line or "").strip()) or "0000"
            bits = list(now_bits.ljust(4, '0'))
            if 1 <= ch <= len(bits):
                bits[ch-1] = '1' if turn_on else '0'
            target = ''.join(bits[:4])

            self._set_onoff_mask_and_verify(
                target,
                start_stab_for_ch=(ch if turn_on else None),
                confirm_cmd=("FLOW_ON" if turn_on else "FLOW_OFF")
            )

        self.enqueue(
            MFC_COMMANDS['READ_MFC_ON_OFF_STATUS'],
            _after_r69,
            timeout_ms=MFC_TIMEOUT,
            gap_ms=MFC_GAP_MS,
            tag="[READ R69]",
        )

    def _set_onoff_mask_and_verify(
        self,
        bits_target: str,
        attempt: int = 1,
        max_attempts: int = 2,
        delay_ms: int = MFC_DELAY_MS,
        start_stab_for_ch: int | None = None,
        confirm_cmd: str | None = None,
    ):
        # 1) L0 전송
        self.enqueue(
            MFC_COMMANDS['SET_ONOFF_MASK'](bits_target),
            lambda _l: None,
            timeout_ms=MFC_TIMEOUT,
            gap_ms=MFC_GAP_MS,
            tag=f"[L0 {bits_target}]",
            allow_no_reply=True,
        )

        # 2) 검증(필요 시 재시도)
        def make_verify(cur_attempt: int):
            def _verify(line: Optional[str]):
                now = self._parse_r69_bits((line or "").strip())
                if now == bits_target:
                    self.command_confirmed.emit("FLOW_ONOFF_BATCH")
                    if confirm_cmd:
                        self.command_confirmed.emit(confirm_cmd)
                    self.status_message.emit("MFC < 확인", f"L0 적용 확인: {bits_target}")

                    # 안정화 시작(필요 시)
                    if start_stab_for_ch:
                        ch = start_stab_for_ch
                        if 1 <= ch <= len(bits_target) and bits_target[ch - 1] == '1':
                            if self.stabilization_timer and self.stabilization_timer.isActive():
                                self.stabilization_timer.stop()
                            self._stabilizing_channel = None
                            self._stabilizing_target = 0.0
                            self._pending_cmd_for_timer = None

                            self._stabilizing_channel = ch
                            self._stabilizing_target = float(self.last_setpoints.get(ch, 0.0))

                            if (self._stabilizing_target > 0) and self.stabilization_timer:
                                self.stabilization_attempts = 0
                                self._pending_cmd_for_timer = "FLOW_ON"
                                self.stabilization_timer.start()
                    return

                if cur_attempt < max_attempts:
                    self.enqueue(
                        MFC_COMMANDS['SET_ONOFF_MASK'](bits_target),
                        lambda _l: None,
                        timeout_ms=MFC_TIMEOUT,
                        gap_ms=MFC_GAP_MS,
                        tag=f"[RE-L0 {bits_target}]",
                        allow_no_reply=True,
                    )
                    QTimer.singleShot(
                        delay_ms,
                        lambda: self.enqueue(
                            MFC_COMMANDS['READ_MFC_ON_OFF_STATUS'],
                            make_verify(cur_attempt + 1),
                            timeout_ms=MFC_TIMEOUT,
                            gap_ms=MFC_GAP_MS,
                            tag="[VERIFY R69]",
                        ),
                    )
                else:
                    self.command_failed.emit("FLOW_ONOFF_BATCH", f"L0 적용 불일치(now={now}, want={bits_target})")
            return _verify

        self.enqueue(
            MFC_COMMANDS['READ_MFC_ON_OFF_STATUS'],
            make_verify(attempt),
            timeout_ms=MFC_TIMEOUT,
            gap_ms=MFC_GAP_MS,
            tag="[VERIFY R69]",
        )

    def _check_valve_position_async(
        self,
        origin_cmd: str,
        attempt: int = 1,
        max_attempts: int = 5,
        delay_ms: int = MFC_DELAY_MS,
        resend_on_attempts: tuple = (2, 4),
        resend_wait_ms: int = MFC_DELAY_MS_VALVE
    ):
        cmd = MFC_COMMANDS['READ_VALVE_POSITION']

        def on_reply(line: Optional[str], origin_cmd=origin_cmd, attempt=attempt):
            line = (line or "").strip()
            ok = False
            try:
                s = (line or "").strip()
                m = re.match(r'^(?:V\s*)?\+?([+\-]?\d+(?:\.\d+)?)$', s)
                pos = float(m.group(1)) if m else None
                ok = (pos is not None) and (
                    (origin_cmd == "VALVE_CLOSE" and pos < 1.0) or
                    (origin_cmd == "VALVE_OPEN"  and pos > 99.0)
                )
            except Exception:
                ok = False

            if ok:
                msg = f"{origin_cmd} 완료."
                self.status_message.emit("MFC < 확인", msg); self._dprint(f"[OK ] {msg}")
                self.command_confirmed.emit(origin_cmd)
            else:
                if attempt < max_attempts:
                    if attempt in resend_on_attempts:
                        vcmd = MFC_COMMANDS[origin_cmd]
                        self.enqueue(vcmd, lambda _l: None, timeout_ms=MFC_TIMEOUT, gap_ms=MFC_GAP_MS,
                                     tag=f"[RE-{origin_cmd}]", allow_no_reply=True)
                        self.status_message.emit("MFC", f"{origin_cmd} 재전송 (시도 {attempt}/{max_attempts})")
                        wait = max(delay_ms, resend_wait_ms)
                    else:
                        wait = delay_ms

                    self.status_message.emit("WARN",
                        f"[{origin_cmd} 검증 재시도] 응답={repr(line)} (시도 {attempt}/{max_attempts})")
                    QTimer.singleShot(
                        wait,
                        lambda: self._check_valve_position_async(
                            origin_cmd, attempt + 1, max_attempts, delay_ms, resend_on_attempts, resend_wait_ms
                        )
                    )
                else:
                    self.command_failed.emit(origin_cmd, "밸브 위치 확인 실패"); self._dprint(f"[FAIL] valve verify {origin_cmd}")

        self.enqueue(cmd, on_reply, timeout_ms=MFC_TIMEOUT, gap_ms=MFC_GAP_MS, tag=f"[VERIFY VALVE {origin_cmd}]")

    def _verify_simple_async(self, cmd: str, params: dict,
                             attempt: int = 1, max_attempts: int = 5, delay_ms: int = MFC_DELAY_MS):
        if cmd == "SP1_SET":
            val = float(params['value'])
            rd = MFC_COMMANDS['READ_SP1_VALUE']
            def on_reply(line: Optional[str], attempt=attempt):
                line = (line or "").strip()
                ok = False
                try:
                    ok = (line and '+' in line and abs(float(line.split('+')[1]) - val) < 0.1)
                except Exception:
                    ok = False
                if ok:
                    msg = f"SP1 목표값 {val:.2f} 설정 완료."
                    self.status_message.emit("MFC < 확인", msg); self._dprint(f"[OK ] {msg}")
                    self.command_confirmed.emit("SP1_SET")
                else:
                    if attempt < max_attempts:
                        self.status_message.emit("WARN",
                            f"[SP1_SET 검증 재시도] 응답={repr(line)} (시도 {attempt}/{max_attempts})")
                        QTimer.singleShot(delay_ms,
                            lambda: self._verify_simple_async("SP1_SET", {"value": val}, attempt+1, max_attempts, delay_ms))
                    else:
                        self.command_failed.emit("SP1_SET", "SP1 설정 확인 실패"); self._dprint("[FAIL] SP1_SET verify")
            self.enqueue(rd, on_reply, timeout_ms=MFC_TIMEOUT, gap_ms=MFC_GAP_MS, tag="[VERIFY SP1_SET]")
            return

        if cmd in ("SP1_ON", "SP4_ON"):
            rd = MFC_COMMANDS['READ_SYSTEM_STATUS']
            def on_reply(line: Optional[str], attempt=attempt, cmd=cmd):
                line = (line or "").strip()
                ok = bool(line and line.startswith("M") and line[1] == ('1' if cmd == "SP1_ON" else '4'))
                if ok:
                    msg = f"{cmd} 활성화 확인."
                    self.status_message.emit("MFC < 확인", msg); self._dprint(f"[OK ] {msg}")
                    self.command_confirmed.emit(cmd)
                else:
                    if attempt < max_attempts:
                        self.status_message.emit("WARN",
                            f"[{cmd} 검증 재시도] 응답={repr(line)} (시도 {attempt}/{max_attempts})")
                        QTimer.singleShot(delay_ms,
                            lambda: self._verify_simple_async(cmd, params, attempt+1, max_attempts, delay_ms))
                    else:
                        self.command_failed.emit(cmd, f"{cmd} 상태 확인 실패"); self._dprint(f"[FAIL] {cmd} verify")
            self.enqueue(rd, on_reply, timeout_ms=MFC_TIMEOUT, gap_ms=MFC_GAP_MS, tag=f"[VERIFY {cmd}]")

    def _read_flow_all_async(self, on_done=None, tag: str = "[POLL R60]", attempt: int = 1):
        """R60 한 번으로 모든 채널 읽기"""
        def on_reply(line: Optional[str], attempt=attempt):
            line = (line or "").strip()
            vals = self._parse_r60_values(line)
            if not vals:
                if attempt < 2:
                    QTimer.singleShot(MFC_DELAY_MS, lambda: self._read_flow_all_async(on_done, tag, attempt+1))
                else:
                    if on_done:
                        self._safe_callback(on_done, None)
                return

            for ch, name in self.gas_map.items():
                idx = ch - 1
                if idx < len(vals):
                    try:
                        v_hw = vals[idx]
                        sf   = MFC_SCALE_FACTORS.get(ch, 1.0)
                        self.update_flow.emit(name, v_hw / sf)
                        self._monitor_flow(ch, v_hw)
                    except Exception:
                        pass

            if on_done:
                self._safe_callback(on_done, vals)

        self.enqueue(MFC_COMMANDS['READ_FLOW_ALL'], on_reply,
                     timeout_ms=MFC_TIMEOUT, gap_ms=MFC_GAP_MS, tag=tag)

    def _read_pressure_async(self, tag: str = ""):
        cmd = MFC_COMMANDS['READ_PRESSURE']
        def on_reply(line: Optional[str]):
            line = (line or "").strip()
            if line:
                self.update_pressure.emit(line)
                self.command_confirmed.emit("READ_PRESSURE")
            else:
                self.command_failed.emit("READ_PRESSURE", "응답 없음")
        self.enqueue(cmd, on_reply, timeout_ms=MFC_TIMEOUT, gap_ms=MFC_GAP_MS, tag=tag or "[READ_PRESSURE]")

    # ---------- 안정화/모니터링 ----------
    def _check_flow_stabilization(self):
        if self._is_aborted:
            return

        ch = self._stabilizing_channel
        target = float(self._stabilizing_target)
        if ch is None or target <= 0:
            if self.stabilization_timer:
                self.stabilization_timer.stop()
            self._stabilizing_channel = None
            self._stabilizing_target = 0.0
            self._pending_cmd_for_timer = None
            self.command_failed.emit("FLOW_ON", "안정화 대상 없음")
            return

        sf  = MFC_SCALE_FACTORS.get(ch, 1.0)
        tol = target * FLOW_ERROR_TOLERANCE

        def _finish(actual_hw: float | None):
            self.stabilization_attempts += 1
            self.status_message.emit(
                "MFC",
                f"유량 확인... (목표: {target:.2f}/{target/sf:.2f}sccm, "
                f"현재: {(-1 if actual_hw is None else actual_hw):.2f}/"
                f"{(-1 if actual_hw is None else actual_hw/sf):.2f}sccm)"
            )

            if (actual_hw is not None) and (abs(actual_hw - target) <= tol):
                if self.stabilization_timer:
                    self.stabilization_timer.stop()
                self.command_confirmed.emit("FLOW_ON")
                self._stabilizing_channel = None
                self._stabilizing_target = 0.0
                self._pending_cmd_for_timer = None
                return

            if self.stabilization_attempts >= 30:
                if self.stabilization_timer:
                    self.stabilization_timer.stop()
                self.command_failed.emit("FLOW_ON", "유량 안정화 시간 초과")
                self._stabilizing_channel = None
                self._stabilizing_target = 0.0
                self._pending_cmd_for_timer = None

        def _after_all(vals):
            if vals and (ch - 1) < len(vals):
                _finish(vals[ch - 1])
            else:
                _finish(None)

        self._read_flow_all_async(on_done=_after_all, tag=f"[STAB R60 ch{ch}]")

    def _monitor_flow(self, channel: int, actual_flow_hw: float):
        target_flow = self.last_setpoints.get(channel, 0.0)
        if target_flow < 0.1:
            self.flow_error_counters[channel] = 0
            return
        if abs(actual_flow_hw - target_flow) > (target_flow * FLOW_ERROR_TOLERANCE):
            self.flow_error_counters[channel] += 1
            if self.flow_error_counters[channel] >= FLOW_ERROR_MAX_COUNT:
                msg = f"Ch{channel} 유량 불안정! (목표: {target_flow:.2f}, 현재: {actual_flow_hw:.2f})"
                self.status_message.emit("MFC(경고)", msg); self._dprint(f"[WARN] {msg}")
                self.flow_error_counters[channel] = 0
        else:
            self.flow_error_counters[channel] = 0

    # ---------- 보조 ----------
    def _parse_r69_bits(self, resp: str) -> str:
        s = (resp or "").strip()
        if s.startswith("L0"):
            payload = s[2:]
        elif s.startswith("L"):
            payload = s[1:]
        else:
            payload = s
        bits = "".join(ch for ch in payload if ch in "01")
        return bits[:4]

    def _safe_callback(self, callback: Callable, *args):
        try:
            callback(*args)
        except Exception as e:
            self._dprint(traceback.format_exc())
            self._dprint(f"[ERROR] Callback failed: {e}")
            self.status_message.emit("MFC", f"콜백 오류: {e}")
