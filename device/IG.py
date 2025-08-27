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

        # 1) 시리얼/타이머는 여기서 만들지 말고 전부 None으로
        self.serial_ig: Optional[QSerialPort] = None

        self._cmd_timer: Optional[QTimer] = None
        self._gap_timer: Optional[QTimer] = None
        self.polling_timer: Optional[QTimer] = None
        self._watchdog: Optional[QTimer] = None

        self._rx = bytearray(); self._RX_MAX = 16 * 1024
        self._LINE_MAX = 512;   self._overflow_count = 0

        self._cmd_q: Deque[Command] = deque()
        self._inflight: Optional[Command] = None
        self._send_spin = False

        self._want_connected = False
        self._reconnect_backoff_ms = IG_RECONNECT_BACKOFF_START_MS
        self._reconnect_pending = False

        self._target_pressure = 0.0
        self._waiting_active = False
        self._wait_start_s = 0.0

        self._first_read_delay_ms = 5000   # IG ON OK 후 첫 RDI까지 한 번만 지연

    # -------------------------------------------------
    # 객체 생성 헬퍼
    # -------------------------------------------------   

    def _ensure_serial_created(self):
        if self.serial_ig is not None:
            return
        self.serial_ig = QSerialPort(self)                    # IG 스레드에서 생성!
        self.serial_ig.setBaudRate(IG_BAUD)
        self.serial_ig.setDataBits(QSerialPort.DataBits.Data8)
        self.serial_ig.setParity(QSerialPort.Parity.NoParity)
        self.serial_ig.setStopBits(QSerialPort.StopBits.OneStop)
        self.serial_ig.setFlowControl(QSerialPort.FlowControl.NoFlowControl)
        self.serial_ig.readyRead.connect(self._on_ready_read)
        self.serial_ig.errorOccurred.connect(self._on_serial_error)

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
            self.polling_timer.setInterval(IG_POLLING_INTERVAL_MS)
            self.polling_timer.timeout.connect(self._enqueue_read_once)

        if self._watchdog is None:
            self._watchdog = QTimer(self)
            self._watchdog.setInterval(IG_WATCHDOG_INTERVAL_MS)
            self._watchdog.timeout.connect(self._watch_connection)    

    # =================================================
    # 연결/해제 & 워치독(자동 재연결)
    # =================================================
    @Slot()
    def connect_ig_device(self) -> bool:
        """
        포트를 열고 워치독을 시작한다.
        - 실패해도 워치독이 지수 백오프로 재연결을 계속 시도한다.
        """
        self._ensure_serial_created()   # ← 여기서 생성
        self._ensure_timers_created()

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
            self.status_message.emit("IG", msg); 
            return False

        self.serial_ig.setPortName(IG_PORT)
        if not self.serial_ig.open(QIODeviceBase.OpenModeFlag.ReadWrite):
            msg = f"{IG_PORT} 연결 실패: {self.serial_ig.errorString()}"
            self.status_message.emit("IG", msg); 
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
        return True

    def _watch_connection(self):
        """워치독: 포트가 닫혀 있고 연결 의도가 있으면 지수 백오프로 재연결을 예약."""
        if not self._want_connected or (self.serial_ig and self.serial_ig.isOpen()):
            return
        if self._reconnect_pending:
            return
        
        self._reconnect_pending = True
        self.status_message.emit("IG", f"재연결 시도 예약... ({self._reconnect_backoff_ms} ms)")
        QTimer.singleShot(self._reconnect_backoff_ms, self._try_reconnect)

    def _try_reconnect(self):
        """예약된 재연결 시도."""
        self._reconnect_pending = False
        if not self._want_connected or (self.serial_ig and self.serial_ig.isOpen()):
            return

        if self._open_port():
            self.status_message.emit("IG", "재연결 성공. 대기 중 명령 재개.")
            QTimer.singleShot(0, self._dequeue_and_send)
            self._reconnect_backoff_ms = IG_RECONNECT_BACKOFF_START_MS
            return

        # 실패 → 백오프 증가
        self._reconnect_backoff_ms = min(self._reconnect_backoff_ms * 2, IG_RECONNECT_BACKOFF_MAX_MS)

    @Slot()
    def cleanup(self):
        self._want_connected = False
        self._waiting_active = False

        # 진행 중 명령 취소 통지
        if self._inflight is not None:
            self._safe_callback(self._inflight.callback, None)
            self._inflight = None
        while self._cmd_q:
            cmd = self._cmd_q.popleft()
            self._safe_callback(cmd.callback, None)

        # 타이머들 정지/파기
        for t_attr in ("_cmd_timer", "_gap_timer", "polling_timer", "_watchdog"):
            t = getattr(self, t_attr)
            if t:
                t.stop()
                t.deleteLater()
                setattr(self, t_attr, None)

        # 시리얼 정리
        if self.serial_ig:
            try:
                if self.serial_ig.isOpen():
                    self.serial_ig.close()
            finally:
                self.serial_ig.deleteLater()
                self.serial_ig = None

        self._rx.clear()
        self._reconnect_pending = False
        self.status_message.emit("IG", "연결 종료됨")


    # =================================================
    # 시리얼 이벤트(오류/수신)
    # =================================================
    def _on_serial_error(self, err: QSerialPort.SerialPortError):
        if err == QSerialPort.SerialPortError.NoError:
            return

        err_name = getattr(err, "name", str(err))
        err_code = getattr(err, "value", "?")
        serr = self.serial_ig.errorString() if self.serial_ig else ""
        # 포괄 오류 로그
        self.status_message.emit("IG", f"시리얼 오류: {serr} (err={err_name}/{err_code})")

        cmd = self._inflight
        if cmd is not None:
            if self._cmd_timer:
                self._cmd_timer.stop()
            self._inflight = None
            # 인플라이트 명령 기준 오류 로그
            self.status_message.emit("IG < 응답", f"{cmd.tag} {cmd.cmd_str.strip()} → (시리얼 오류)")
            if cmd.retries_left > 0:
                cmd.retries_left -= 1
                self._cmd_q.appendleft(cmd)
            else:
                self._safe_callback(cmd.callback, None)

        if self.serial_ig and self.serial_ig.isOpen():
            self.serial_ig.close()
        if self._gap_timer:
            self._gap_timer.stop()
        self._rx.clear()

        QTimer.singleShot(0, self._watch_connection)

    def _on_ready_read(self):
        """
        수신 핸들러:
          - 버퍼 누적 → CR/LF 기준 라인 추출
          - 보낸 명령 에코 라인 스킵
          - 유효 응답 1줄만 처리(_finish_command) → 나머지는 다음 틱
        """
        if not (self.serial_ig and self.serial_ig.isOpen()):
            return
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
                self.status_message.emit("IG", f"Rx line too long (+{len(line_bytes)-self._LINE_MAX}B), truncating")
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
                    continue

            # 유효 응답 1줄 처리 후 종료(한 틱에 1응답)
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

        # ✅ 인플라이트 없고, 갭 타이머가 없거나 비활성일 때만 송신 예약 (이 블록만 유지)
        if (self._inflight is None) and (not (self._gap_timer and self._gap_timer.isActive())):
            QTimer.singleShot(0, self._dequeue_and_send)

    def _dequeue_and_send(self):
        if self._inflight is not None or not self._cmd_q:
            return
        if not (self.serial_ig and self.serial_ig.isOpen()):
            return
        if self._gap_timer and self._gap_timer.isActive():
            return

        if self._send_spin:
            self.status_message.emit("IG", "[GUARD] _dequeue_and_send re-enter blocked")
            return
        self._send_spin = True

        try:
            cmd = self._cmd_q.popleft()
            self._inflight = cmd
            self._rx.clear()

            self.status_message.emit("IG", f"[SEND] {cmd.cmd_str.strip()} (tag={cmd.tag})")

            payload = cmd.cmd_str.encode('ascii')
            n = self.serial_ig.write(payload)
            n = int(n) if isinstance(n, int) or hasattr(n, "__int__") else -1
            if n <= 0:
                raise IOError(f"serial write returned {n}")

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

            if self._cmd_timer:
                self._cmd_timer.stop()
                self._cmd_timer.start(cmd.timeout_ms)

        except Exception as e:
            failed = self._inflight
            self._inflight = None
            if self._cmd_timer:
                self._cmd_timer.stop()
            if failed:
                # 전송 실패 로그(보낸 명령 포함)
                self.status_message.emit("IG", f"{failed.tag} {failed.cmd_str.strip()} 전송 오류: {e}")
                self._cmd_q.appendleft(failed)
            try:
                # 워치독 기반 재연결로 통일
                if self.serial_ig and self.serial_ig.isOpen():
                    try:
                        self.serial_ig.close()
                    except Exception:
                        pass
                QTimer.singleShot(0, self._watch_connection)
            except Exception:
                pass
            return
        finally:
            self._send_spin = False

    def _on_cmd_timeout(self):
        """명령 응답 타임아웃."""
        if not self._inflight:
            return
        cmd = self._inflight
        if cmd.allow_no_reply:
            self.status_message.emit("IG", "[NOTE] no-reply command; proceeding after write")
        else:
            self.status_message.emit("IG", "[TIMEOUT] command response timed out")
        self._finish_command(None)

    def _finish_command(self, line: Optional[str]):
        """
        인플라이트 명령을 완료 처리한다.
          - line=None: 타임아웃/무응답/취소
          - line=str: 정상 응답(한 줄)
        """
        cmd = self._inflight
        if cmd is None:
            return

        if self._cmd_timer:
            self._cmd_timer.stop()
        self._inflight = None

        sent_txt = cmd.cmd_str.strip()
        tag_txt  = cmd.tag or ""

        if line is None:
            self.status_message.emit("IG < 응답", f"{tag_txt} {sent_txt} → (응답 없음/타임아웃)")

            if cmd.allow_no_reply:
                self._safe_callback(cmd.callback, None)
                if self._gap_timer:
                    self._gap_timer.start(cmd.gap_ms)
                return

            if cmd.retries_left > 0:
                cmd.retries_left -= 1
                # 재시도 로그
                self.status_message.emit("IG", f"{tag_txt} {sent_txt} 재시도 남은횟수={cmd.retries_left}")
                self._cmd_q.appendleft(cmd)
                if self.serial_ig and self.serial_ig.isOpen():
                    self.serial_ig.close()
                QTimer.singleShot(0, self._watch_connection)
                return

            self._safe_callback(cmd.callback, None)
            if self._gap_timer:
                self._gap_timer.start(cmd.gap_ms)
            return
        
        # 정상 수신 로그(보낸 명령 ↔ 받은 응답)
        recv_txt = (line or "").strip()
        self.status_message.emit("IG < 응답", f"{tag_txt} {sent_txt} ← {recv_txt}")


        self._safe_callback(cmd.callback, line.strip())
        if self._gap_timer:
            self._gap_timer.start(cmd.gap_ms)

    # =================================================
    # 외부 API
    # =================================================
    @Slot(float)        # Signal(float)과 호환
    @Slot(float, int)   # 필요 시 명시적 interval로도 호출 가능
    def start_wait_for_pressure(self, base_pressure: float, interval_ms: int = IG_POLLING_INTERVAL_MS):
        """
        총 대기시간(IG_WAIT_TIMEOUT) 내 목표 압력(base_pressure)에 도달할 때까지
        큐 기반으로 폴링한다.

        시퀀스:
          (A) IG ON  → allow_no_reply=False, gap=1000ms
          (B) RDI 1회 → (A)에서 'OK'응답을 받으면 첫 읽기
          (C) 이후 polling_timer가 interval_ms마다 RDI 큐잉
        """
        if self._waiting_active:
            self.status_message.emit("IG", "이미 압력 대기 프로세스가 실행 중입니다.")
            return

        # 포트 연결 + 워치독 시작
        if not (self.serial_ig and self.serial_ig.isOpen()):
            if not self.connect_ig_device():
                self.base_pressure_failed.emit("IG", "포트 연결 실패")
                return

        self.status_message.emit("IG", "Base Pressure 대기를 시작합니다.")
        self._target_pressure = float(base_pressure)
        self._wait_start_s = time.monotonic()
        self._waiting_active = True

        # (A) IG ON, gap=1000ms
        def _after_on(line: Optional[str]):
            if line is None:
                # 응답 없음 → 포트 닫지 말고 같은 포트에서 재시도
                self.status_message.emit("IG", "SIG 1 응답 없음 → 동일 포트 재시도")
                self.enqueue("SIG 1", _after_on,
                            timeout_ms=IG_TIMEOUT_MS, gap_ms=IG_GAP_MS,
                            tag="[IG ON - RETRY]", retries_left=5, allow_no_reply=False)
                return
            
            s = (line or "").strip()
            su = s.upper()

            if su.startswith("OK"):
                # ✅ OK 수신 → 첫 RDI는 5초 뒤 '한 번만' 실행, 이후에 polling 시작
                self.status_message.emit("IG",
                    f"IG ON OK → 첫 RDI를 {self._first_read_delay_ms}ms 후 수행")
                QTimer.singleShot(
                    self._first_read_delay_ms,
                    lambda: self._first_rdi_then_start_polling(int(interval_ms))
                )
                return
            
            # OK 미수신 → 포트 닫고 워치독으로 재연결 예약 후 SIG 1 재전송
            try:
                if self.serial_ig and self.serial_ig.isOpen():
                    self.serial_ig.close()
            except Exception:
                pass
            QTimer.singleShot(0, self._watch_connection)
            # 첫 시도 실패 이후
            self.enqueue(
                "SIG 1", _after_on,
                timeout_ms=IG_TIMEOUT_MS, gap_ms=IG_GAP_MS,
                tag="[IG ON - RETRY]", retries_left=5, allow_no_reply=False
            )

        # 첫 시도
        # SIG 1은 반드시 응답 요구(allow_no_reply=False)
        self.enqueue(
            "SIG 1", _after_on,
            timeout_ms=IG_TIMEOUT_MS, gap_ms=IG_GAP_MS,
            tag="[IG ON]", retries_left=5, allow_no_reply=False
        )

    def _first_rdi_then_start_polling(self, interval_ms: int):
        # 대기 상태가 이미 종료되었다면 아무 것도 하지 않음 (취소/타임아웃 대비)
        if not self._waiting_active:
            return

        # 첫 RDI 한 번만 실행
        self._enqueue_read_once(tag="[FIRST READ AFTER ON]")

        # 그리고 나서 polling 시작
        if self.polling_timer:
            self.polling_timer.setInterval(int(interval_ms))
            self.polling_timer.start()
    
    # =================================================
    # 내부: 읽기/파싱/판정
    # =================================================
    def _enqueue_read_once(self, tag: str = "[POLL RDI]"):
        """
        RDI 한 번을 큐에 올려 읽는다.
        - 응답 파싱 실패/빈 응답이면 다음 주기에 자동 재시도
        - 'IG OFF' 응답이면: 중단 대신 자동 재점등(SIG 1) 시도 후 폴링 재개
        - 목표 도달 시 SIG 0 전송 후 cleanup
        """
        def _on_rdi(line: Optional[str]):
            # 대기 상태가 아니면 무시(취소/성공 후 호출될 수 있음)
            if not self._waiting_active:
                return

            now_s = time.monotonic()

            # 1) 전체 대기 시간 초과: 자동 OFF 없이 종료 신호만
            if (now_s - self._wait_start_s) > float(IG_WAIT_TIMEOUT):
                self.status_message.emit("IG", f"시간 초과({IG_WAIT_TIMEOUT}초): 목표 압력 미도달")
                self.base_pressure_failed.emit("IG", "Timeout")
                self._waiting_active = False
                if self.polling_timer:
                    self.polling_timer.stop()
                
                def _after_sig0(_l: Optional[str]):
                    self.cleanup()

                self.enqueue(
                    "SIG 0", _after_sig0,
                    timeout_ms=IG_TIMEOUT_MS, gap_ms=150,
                    tag="[IG OFF] SIG 0", retries_left=3, allow_no_reply=False
                )

                return

            # 2) 응답 파싱
            s = (line or "").strip()
            if not s:
                return  # 빈 응답 → 다음 주기 자동 재시도

            # 3) 폴링 중 'IG OFF' 응답: 자동 재점등 시도
            if s.upper() == "IG OFF":
                self.status_message.emit("IG", "IG OFF 응답 감지 → 자동 재점등을 시도합니다.")
                if self.polling_timer:
                    self.polling_timer.stop()

                # 로컬(클로저) 재시도 카운터/백오프(ms): 2s, 5s, 10s
                attempt = {"n": 0}
                backoff = [2000, 5000, 10000]

                def re_on_cb(reply: Optional[str]):
                    ok = (reply or "").strip().upper() == "OK"
                    if ok:
                        self.status_message.emit("IG", "재점등 성공. 첫 RDI 후 폴링을 재개합니다.")
                        QTimer.singleShot(
                            self._first_read_delay_ms,
                            lambda: (
                                self._enqueue_read_once(tag="[AFTER RE-ON]"),
                                self.polling_timer and self.polling_timer.start()
                            )
                        )
                    else:
                        if attempt["n"] < len(backoff):
                            delay = backoff[attempt["n"]]
                            attempt["n"] += 1
                            self.status_message.emit("IG", f"재점등 실패 → {delay}ms 후 재시도")
                            QTimer.singleShot(
                                delay,
                                lambda: self.enqueue(
                                    "SIG 1", re_on_cb,
                                    timeout_ms=IG_TIMEOUT_MS, gap_ms=IG_GAP_MS,
                                    tag="[IG RE-ON]", retries_left=3, allow_no_reply=False
                                )
                            )
                        else:
                            self.status_message.emit("IG", "자동 재점등 실패(한도 도달). 폴링만 재개합니다.")
                            if self.polling_timer:
                                self.polling_timer.start()
                # 즉시 1차 재점등
                self.enqueue(
                    "SIG 1", re_on_cb,
                    timeout_ms=IG_TIMEOUT_MS, gap_ms=IG_GAP_MS,
                    tag="[IG RE-ON]", retries_left=3, allow_no_reply=False
                )
                return

            # 4) 숫자 파싱
            cleaned = s.lower().replace("x10e", "e")   # '1.2x10e-5' → '1.2e-5'
            try:
                pressure = float(cleaned)
            except Exception:
                self.status_message.emit("IG", f"압력 읽기 실패(파싱): {repr(s)}")
                return

            # 5) 업데이트 + 목표 도달 판단
            self.pressure_update.emit(pressure)
            self.status_message.emit("IG",
                f"현재 압력: {pressure:.3e} Torr (목표: {self._target_pressure:.3e} Torr)")

            if pressure <= self._target_pressure:
                # === 목표 도달: SIG 0 → cleanup → 다음 단계(상위는 base_pressure_reached 시그널로 진행) ===
                self.status_message.emit("IG", "목표 압력 도달")
                self.base_pressure_reached.emit()
                self._waiting_active = False
                if self.polling_timer:
                    self.polling_timer.stop()

                def _after_sig0(_l: Optional[str]):
                    self.cleanup()

                self.enqueue(
                    "SIG 0", _after_sig0,
                    timeout_ms=IG_TIMEOUT_MS, gap_ms=150,
                    tag="[IG OFF] SIG 0", retries_left=3, allow_no_reply=False
                )
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
            self.status_message.emit("IG", f"콜백 오류: {e}")

    @Slot()
    def start_polling(self):
        if self.polling_timer:
            self.polling_timer.start()

    @Slot()
    def stop_polling(self):
        if self.polling_timer:
            self.polling_timer.stop()

