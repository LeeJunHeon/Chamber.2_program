# -*- coding: utf-8 -*-
"""
IG.py — PyQt6 QSerialPort(완전 비동기) 기반 IG(이온 게이지/진공 게이지) 컨트롤러

핵심 설계:
  - PyQt6 QSerialPort + readyRead 시그널 → UI/이벤트 루프 블로킹 없음
  - 단일 명령 큐: (타임아웃 / 재시도 / 인터커맨드 gap)로 모든 송수신 직렬화
  - 연결 워치독 + 지수 백오프 재연결 → 중간에 케이블/포트 끊겨도 회복
  - 일정: "IG ON → gap 1초 → RDI 1회 즉시 읽기 → 이후 주기 폴링"

요구 상수(lib.config):
  IG_PORT, IG_BAUD, IG_WAIT_TIMEOUT(초),
  IG_TIMEOUT_MS, IG_GAP_MS,
  IG_POLLING_INTERVAL_MS, IG_WATCHDOG_INTERVAL_MS,
  IG_RECONNECT_BACKOFF_START_MS, IG_RECONNECT_BACKOFF_MAX_MS,
  DEBUG_PRINT
"""

from __future__ import annotations
from dataclasses import dataclass
from collections import deque
from typing import Optional, Callable, Deque

import time

from PyQt6.QtCore import QObject, QTimer, QIODeviceBase, pyqtSignal as Signal, pyqtSlot as Slot
from PyQt6.QtSerialPort import QSerialPort, QSerialPortInfo

from lib.config import (
    IG_PORT, IG_BAUD, IG_WAIT_TIMEOUT,
    IG_TIMEOUT_MS, IG_GAP_MS,
    IG_POLLING_INTERVAL_MS, IG_WATCHDOG_INTERVAL_MS,
    IG_RECONNECT_BACKOFF_START_MS, IG_RECONNECT_BACKOFF_MAX_MS,
    DEBUG_PRINT
)

# =========================
#  명령 큐에 넣는 레코드
# =========================
@dataclass
class Command:
    cmd_str: str                                      # 실제 전송 문자열(끝에 '\r' 필수)
    callback: Callable[[Optional[str]], None]         # 응답 1줄 또는 None을 넘겨받는 콜백
    timeout_ms: int                                   # 응답 타임아웃(ms)
    gap_ms: int                                       # 다음 명령 전 인터커맨드 간격(ms)
    tag: str                                          # 로깅 태그
    retries_left: int                                 # 타임아웃/오류 시 재시도 횟수
    allow_no_reply: bool                              # 응답 없이 진행해도 되는 명령(예: 일부 SET/ON)

# =========================
#  IG 컨트롤러
# =========================
class IGController(QObject):
    # ---- 상위/UI로 내보내는 시그널 ----
    status_message = Signal(str, str)         # (섹션, 메시지) 예: ("IG", "연결 성공")
    pressure_update = Signal(float)           # 최신 압력 값 (Torr)
    base_pressure_reached = Signal()          # 목표 압력 도달
    base_pressure_failed = Signal(str, str)   # (섹션, 사유) 예: ("IG","Timeout")

    def __init__(self, parent=None):
        super().__init__(parent)
        self.debug_print = DEBUG_PRINT

        # ---- 1) QSerialPort 설정 ----
        self.serial_ig = QSerialPort(self)
        self.serial_ig.setBaudRate(IG_BAUD)
        self.serial_ig.setDataBits(QSerialPort.DataBits.Data8)
        self.serial_ig.setParity(QSerialPort.Parity.NoParity)
        self.serial_ig.setStopBits(QSerialPort.StopBits.OneStop)
        self.serial_ig.setFlowControl(QSerialPort.FlowControl.NoFlowControl)
        self.serial_ig.readyRead.connect(self._on_ready_read)
        self.serial_ig.errorOccurred.connect(self._on_serial_error)

        # ---- 2) 수신 버퍼/제한 ----
        self._rx = bytearray()
        self._RX_MAX = 16 * 1024      # 과도 수신 보호(최대 보존 바이트)
        self._LINE_MAX = 512          # 한 줄 최대 길이
        self._overflow_count = 0

        # ---- 3) 명령 큐/인플라이트 ----
        self._cmd_q: Deque[Command] = deque()
        self._inflight: Optional[Command] = None      # 현재 진행 중인 단일 명령
        self._send_spin = False                       # 재진입 가드

        # ---- 4) 타이머들 ----
        self._cmd_timer = QTimer(self)   # 명령 타임아웃
        self._cmd_timer.setSingleShot(True)
        self._cmd_timer.timeout.connect(self._on_cmd_timeout)

        self._gap_timer = QTimer(self)   # 인터커맨드 간격
        self._gap_timer.setSingleShot(True)
        self._gap_timer.timeout.connect(self._dequeue_and_send)

        self.polling_timer = QTimer(self)                           # 주기 폴링(RDI 큐잉)
        self.polling_timer.setInterval(IG_POLLING_INTERVAL_MS)
        self.polling_timer.timeout.connect(self._enqueue_read_once)

        self._watchdog = QTimer(self)                               # 연결 감시/재연결 스케줄러
        self._watchdog.setInterval(IG_WATCHDOG_INTERVAL_MS)
        self._watchdog.timeout.connect(self._watch_connection)

        # ---- 5) 연결/재연결 상태 ----
        self._want_connected = False
        self._reconnect_backoff_ms = IG_RECONNECT_BACKOFF_START_MS
        self._reconnect_pending = False

        # ---- 6) 대기 로직 상태 ----
        self._target_pressure: float = 0.0
        self._waiting_active: bool = False
        self._wait_start_s: float = 0.0   # time.monotonic() 기준 시작 시각

    # -------------------------------------------------
    # 내부 디버그 출력
    # -------------------------------------------------
    def _dprint(self, *args):
        if self.debug_print:
            try:
                print(*args, flush=True)
            except Exception:
                pass

    # =================================================
    # 연결/해제 & 워치독(자동 재연결)
    # =================================================
    def connect_ig_device(self) -> bool:
        """
        포트를 열고 워치독을 시작한다.
        - 실패해도 워치독이 지수 백오프로 재연결을 계속 시도한다.
        """
        self._want_connected = True
        ok = self._open_port()
        self._watchdog.start()
        return ok

    def _open_port(self) -> bool:
        """실제 포트 open. 성공 시 True, 실패 시 False."""
        if self.serial_ig.isOpen():
            return True

        # 사용 가능 포트 확인(편의용). 실제 운영에서 포트명이 리스트에 없을 수 있으면 이 검사 제거 가능.
        available = {p.portName() for p in QSerialPortInfo.availablePorts()}
        if IG_PORT not in available:
            msg = f"{IG_PORT} 존재하지 않음. 사용 가능 포트: {sorted(available)}"
            self.status_message.emit("IG", msg); self._dprint(f"[IG ] {msg}")
            return False

        self.serial_ig.setPortName(IG_PORT)
        if not self.serial_ig.open(QIODeviceBase.OpenModeFlag.ReadWrite):
            msg = f"{IG_PORT} 연결 실패: {self.serial_ig.errorString()}"
            self.status_message.emit("IG", msg); self._dprint(f"[IG ] {msg}")
            return False

        # 라인 제어/버퍼 초기화
        self.serial_ig.setDataTerminalReady(True)
        self.serial_ig.setRequestToSend(False)
        self.serial_ig.clear(QSerialPort.Direction.AllDirections)
        self._rx.clear()

        # 재연결 백오프 초기화
        self._reconnect_backoff_ms = IG_RECONNECT_BACKOFF_START_MS
        self._reconnect_pending = False

        self.status_message.emit("IG", f"{IG_PORT} 연결 성공 (QSerialPort)")
        self._dprint(f"[IG ] {IG_PORT} connected")
        return True

    def _watch_connection(self):
        """워치독: 포트가 닫혀 있고 연결 의도가 있으면 지수 백오프로 재연결을 예약."""
        if not self._want_connected or self.serial_ig.isOpen():
            return
        if self._reconnect_pending:
            return

        self._reconnect_pending = True
        self.status_message.emit("IG", f"재연결 시도 예약... ({self._reconnect_backoff_ms} ms)")
        self._dprint(f"[IG ] reconnect in {self._reconnect_backoff_ms}ms")
        QTimer.singleShot(self._reconnect_backoff_ms, self._try_reconnect)

    def _try_reconnect(self):
        """예약된 재연결 시도."""
        self._reconnect_pending = False
        if not self._want_connected or self.serial_ig.isOpen():
            return

        if self._open_port():
            self.status_message.emit("IG", "재연결 성공. 대기 중 명령 재개.")
            self._dprint("[IG ] reconnected")
            QTimer.singleShot(0, self._dequeue_and_send)
            self._reconnect_backoff_ms = IG_RECONNECT_BACKOFF_START_MS
            return

        # 실패 → 백오프 증가
        self._reconnect_backoff_ms = min(self._reconnect_backoff_ms * 2, IG_RECONNECT_BACKOFF_MAX_MS)

    @Slot()
    def cleanup(self):
        """
        안전 종료:
          - 모든 타이머 정지
          - 인플라이트/대기 중 명령에 콜백(None) 통지
          - 버퍼/포트 정리
        """
        self._want_connected = False
        self._waiting_active = False

        # 타이머 정지
        self._cmd_timer.stop()
        self._gap_timer.stop()
        self.polling_timer.stop()
        self._watchdog.stop()
        self._reconnect_pending = False

        # 진행 중/대기 중 콜백에 취소 통지
        if self._inflight is not None:
            self._safe_callback(self._inflight.callback, None)
            self._inflight = None
        while self._cmd_q:
            cmd = self._cmd_q.popleft()
            self._safe_callback(cmd.callback, None)

        # 버퍼/포트 정리
        self._rx.clear()
        if self.serial_ig.isOpen():
            self.serial_ig.close()
            self.status_message.emit("IG", "연결 종료됨")

    # =================================================
    # 시리얼 이벤트(오류/수신)
    # =================================================
    def _on_serial_error(self, err: QSerialPort.SerialPortError):
        """Qt 시리얼 오류 콜백: 진행 중 명령 롤백 + 포트 닫기 + 워치독 재연결."""
        if err == QSerialPort.SerialPortError.NoError:
            return

        err_name = getattr(err, "name", str(err))
        err_code = getattr(err, "value", "?")
        msg = f"시리얼 오류: {self.serial_ig.errorString()} (err={err_name}/{err_code})"
        self.status_message.emit("IG", msg); self._dprint(f"[ERR] {msg}")

        # 진행 중 명령을 재시도 기회 유지한 채로 되돌림
        if self._inflight is not None:
            cmd = self._inflight
            self._cmd_timer.stop()
            self._inflight = None
            if cmd.retries_left > 0:
                cmd.retries_left -= 1
                self._cmd_q.appendleft(cmd)
            else:
                self._safe_callback(cmd.callback, None)

        # 포트 닫고 버퍼/갭 타이머 정리 → 재연결은 워치독이 담당
        if self.serial_ig.isOpen():
            self.serial_ig.close()
        self._gap_timer.stop()
        self._rx.clear()

        # 다음 틱에 워치독이 동작하도록 트리거
        QTimer.singleShot(0, self._watch_connection)

    def _on_ready_read(self):
        """
        수신 핸들러:
          - 버퍼 누적 → CR/LF 기준 라인 추출
          - 보낸 명령 에코 라인 스킵
          - 유효 응답 1줄만 처리(_finish_command) → 나머지는 다음 틱
        """
        ba = self.serial_ig.readAll()
        if ba.isEmpty():
            return

        # 1) 버퍼 누적 + 과다 보호
        self._rx.extend(bytes(ba))
        if len(self._rx) > self._RX_MAX:
            del self._rx[:-self._RX_MAX]
            self._overflow_count += 1
            if self._overflow_count % 5 == 1:
                self.status_message.emit("IG", f"수신 버퍼 과다(RX>{self._RX_MAX}); 최근 {self._RX_MAX}B만 보존.")
            self._dprint(f"[WARN] RX overflow: keep tail {len(self._rx)}B")

        # 2) 줄 단위 파싱
        while True:
            i_cr = self._rx.find(b'\r')
            i_lf = self._rx.find(b'\n')
            if i_cr == -1 and i_lf == -1:
                break
            idx = i_cr if i_lf == -1 else (i_lf if i_cr == -1 else min(i_cr, i_lf))
            line_bytes = self._rx[:idx]

            # CRLF/LFCR 2바이트 처리
            drop = idx + 1
            if drop < len(self._rx):
                ch = self._rx[idx]
                nxt = self._rx[idx + 1]
                if (ch == 13 and nxt == 10) or (ch == 10 and nxt == 13):
                    drop += 1
            del self._rx[:drop]

            # 라인 길이 제한
            if len(line_bytes) > self._LINE_MAX:
                self._dprint(f"[WARN] RX line too long (+{len(line_bytes)-self._LINE_MAX}B), truncating")
                line_bytes = line_bytes[:self._LINE_MAX]

            try:
                line = line_bytes.decode('ascii', errors='ignore').strip()
            except Exception:
                line = None

            if not line:
                continue

            # 에코 라인은 스킵(보낸 명령과 정확히 동일한 한 줄)
            if self._inflight:
                sent = (self._inflight.cmd_str or "").strip()
                if line == sent:
                    self._dprint(f"[RECV] echo skipped: {repr(line)}")
                    continue

            # 유효 응답 1줄 처리 후 종료(한 틱에 1응답)
            self._dprint(f"[RECV] {repr(line)}")
            self._finish_command(line)
            break

        # 선행 CR/LF 정리(빈 줄 재처리 방지)
        while self._rx[:1] in (b'\r', b'\n'):
            del self._rx[0:1]

    # =================================================
    # 명령 큐
    # =================================================
    def enqueue(self, cmd_str: str, on_reply: Callable[[Optional[str]], None],
                timeout_ms: int = IG_TIMEOUT_MS, gap_ms: int = IG_GAP_MS,
                tag: str = "", retries_left: int = 5, allow_no_reply: bool = False):
        """
        명령을 큐에 추가한다.
          - cmd_str 끝의 '\r'은 자동 보정
          - on_reply는 라인(str) 또는 None을 받는다(타임아웃/취소/무응답 등)
        """
        if not cmd_str.endswith('\r'):
            cmd_str += '\r'
        self._cmd_q.append(Command(cmd_str, on_reply, timeout_ms, gap_ms, tag, retries_left, allow_no_reply))

        # 인플라이트가 없고 갭 타이머가 쉬는 중이면 즉시 송신 시도 예약
        if (self._inflight is None) and (not self._gap_timer.isActive()):
            QTimer.singleShot(0, self._dequeue_and_send)

    def _dequeue_and_send(self):
        """큐에서 1건을 꺼내 전송하고, 타임아웃 타이머를 시작한다."""
        if self._inflight is not None or not self._cmd_q:
            return
        if not self.serial_ig.isOpen():
            return
        if self._gap_timer.isActive():
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
            self.status_message.emit("IG > 전송", f"{cmd.tag or ''} {cmd.cmd_str.strip()}".strip())

            payload = cmd.cmd_str.encode('ascii')
            n = self.serial_ig.write(payload)
            n = int(n) if isinstance(n, int) or hasattr(n, "__int__") else -1
            if n <= 0:
                raise IOError(f"serial write returned {n}")

            # 부분 쓰기 보정(실제 QSerialPort는 보통 한 번에 다 보냄)
            total = n
            if total != len(payload):
                remain = payload[total:]
                m = self.serial_ig.write(remain)
                m = int(m) if isinstance(m, int) or hasattr(m, "__int__") else -1
                if m > 0:
                    total += m
                if total != len(payload):
                    raise IOError(f"partial write: queued {total}/{len(payload)} bytes")

            self.serial_ig.flush()
            self._cmd_timer.stop()
            self._cmd_timer.start(cmd.timeout_ms)

        except Exception as e:
            # 전송 실패 → 해당 명령 보존/재시도 예약 또는 재연결
            self._dprint(f"[ERROR] Send failed: {e}")
            failed = self._inflight
            self._inflight = None
            self._cmd_timer.stop()
            if failed:
                self._cmd_q.appendleft(failed)
            try:
                if not self.serial_ig.isOpen():
                    QTimer.singleShot(0, self._try_reconnect)
                else:
                    gap_ms = failed.gap_ms if failed else 100
                    self._gap_timer.start(gap_ms)
                    QTimer.singleShot(gap_ms + 1, self._dequeue_and_send)
            except Exception as ee:
                self._dprint(f"[WARN] reconnect/retry schedule failed: {ee}")
            self.status_message.emit("IG", f"전송 오류: {e}")
            return
        finally:
            self._send_spin = False

    def _on_cmd_timeout(self):
        """명령 응답 타임아웃."""
        if not self._inflight:
            return
        cmd = self._inflight
        if cmd.allow_no_reply:
            self._dprint("[NOTE] no-reply command; proceeding after write")
        else:
            self._dprint("[TIMEOUT] command response timed out")
        self._finish_command(None)

    def _finish_command(self, line: Optional[str]):
        """
        인플라이트 명령을 완료 처리한다.
          - line=None: 타임아웃/무응답/취소
          - line=str: 정상 응답(한 줄)
        """
        if self._inflight is None:
            return

        cmd = self._inflight
        self._cmd_timer.stop()
        self._inflight = None

        # 응답 없음 처리
        if line is None:
            if cmd.allow_no_reply:
                # 응답은 없어도 정상 진행
                self._safe_callback(cmd.callback, None)
                self._gap_timer.start(cmd.gap_ms)
                return

            # 재시도 기회가 있으면 되돌리고 재연결
            if cmd.retries_left > 0:
                cmd.retries_left -= 1
                self._cmd_q.appendleft(cmd)
                if self.serial_ig.isOpen():
                    self.serial_ig.close()
                self._try_reconnect()
                return

            # 더 이상 재시도 불가 → 콜백 통지 후 진행
            self._safe_callback(cmd.callback, None)
            self._gap_timer.start(cmd.gap_ms)
            return

        # 정상 응답 처리
        self._safe_callback(cmd.callback, line.strip())
        self._gap_timer.start(cmd.gap_ms)

    # =================================================
    # 외부 API
    # =================================================
    @Slot(float, int)
    def start_wait_for_pressure(self, base_pressure: float, interval_ms: int = IG_POLLING_INTERVAL_MS):
        """
        총 대기시간(IG_WAIT_TIMEOUT) 내 목표 압력(base_pressure)에 도달할 때까지
        큐 기반으로 폴링한다.

        시퀀스:
          (A) IG ON  → allow_no_reply=True, gap=1000ms
          (B) RDI 1회 → (A)의 gap이 끝나자마자 전송되어 '켜자마자 1초 뒤' 첫 읽기
          (C) 이후 polling_timer가 interval_ms마다 RDI 큐잉
        """
        if self._waiting_active:
            self.status_message.emit("IG", "이미 압력 대기 프로세스가 실행 중입니다.")
            return

        # 포트 연결 + 워치독 시작
        if not self.serial_ig.isOpen():
            if not self.connect_ig_device():
                self.base_pressure_failed.emit("IG", "포트 연결 실패")
                return

        self.status_message.emit("IG", "Base Pressure 대기를 시작합니다.")
        self._target_pressure = float(base_pressure)
        self._wait_start_s = time.monotonic()
        self._waiting_active = True

        # (A) IG ON — 응답이 없어도 진행(allow_no_reply=True), gap=1000ms
        def _after_on(_line: Optional[str]):
            # ON에 대한 응답은 무시(장비에 따라 OK가 오기도, 안 오기도 함)
            pass

        self.enqueue("SIG 1", _after_on,
                     timeout_ms=IG_TIMEOUT_MS, gap_ms=1000,
                     tag="[IG ON]", retries_left=1, allow_no_reply=True)

        # (B) 첫 읽기 — 큐 뒤에 붙인다 → 위 gap 1초 후 자동 전송됨
        self._enqueue_read_once(tag="[FIRST READ AFTER ON]")

        # (C) 이후 주기 폴링 시작
        self.polling_timer.setInterval(int(interval_ms))
        self.polling_timer.start()

    # =================================================
    # 내부: 읽기/파싱/판정
    # =================================================
    def _enqueue_read_once(self, tag: str = "[POLL RDI]"):
        """
        RDI 한 번을 큐에 올려 읽는다.
          - 응답 파싱 실패/빈 응답이면 아무것도 하지 않고(실패 시그널 없음) 다음 주기에 재시도
          - 성공하면 pressure_update 시그널 + 목표 도달 판단
        """
        def _on_rdi(line: Optional[str]):
            # 대기 중이 아니면 무시(취소/성공 후 호출될 수 있음)
            if not self._waiting_active:
                return

            # 총 대기 시간 초과 먼저 확인
            if (time.monotonic() - self._wait_start_s) > float(IG_WAIT_TIMEOUT):
                self.status_message.emit("IG", f"시간 초과({IG_WAIT_TIMEOUT}초): 목표 압력 미도달")
                self.base_pressure_failed.emit("IG", "Timeout")
                self.polling_timer.stop()
                self.cleanup()
                return

            # 응답 파싱
            s = (line or "").strip()
            if not s:
                return  # 빈 응답 → 다음 주기에 자동 재시도
            cleaned = s.lower().replace("x10e", "e")   # '1.2x10e-5' → '1.2e-5'
            try:
                pressure = float(cleaned)
            except Exception:
                self.status_message.emit("IG", f"압력 읽기 실패(파싱): {repr(s)}")
                return

            # 업데이트 + 목표 도달 판정
            self.pressure_update.emit(pressure)
            self.status_message.emit("IG",
                f"현재 압력: {pressure:.3e} Torr (목표: {self._target_pressure:.3e} Torr)")
            if pressure <= self._target_pressure:
                self.status_message.emit("IG", "목표 압력 도달")
                self.base_pressure_reached.emit()
                self.polling_timer.stop()
                self.cleanup()
                return

        # allow_no_reply=False: RDI는 반드시 한 줄을 기대
        self.enqueue("RDI", _on_rdi, timeout_ms=IG_TIMEOUT_MS, gap_ms=IG_GAP_MS,
                     tag=tag, retries_left=1, allow_no_reply=False)

    # =================================================
    # 보조
    # =================================================
    def _safe_callback(self, callback: Callable[[Optional[str]], None], arg: Optional[str]):
        """콜백 예외가 전체 루프를 깨지 않도록 보호."""
        try:
            callback(arg)
        except Exception as e:
            self._dprint(f"[ERROR] Callback failed: {e}")
            self.status_message.emit("IG", f"콜백 오류: {e}")
