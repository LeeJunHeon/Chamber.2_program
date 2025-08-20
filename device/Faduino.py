# -*- coding: utf-8 -*-
"""
Faduino.py — PyQt6 QSerialPort(완전 비동기) 기반 Faduino 컨트롤러 (리팩터링판)

핵심:
  - QSerialPort + readyRead 시그널 → UI/이벤트 루프 블로킹 없음
  - 단일 명령 큐(타임아웃/재시도/인터커맨드 간격) → 폴링·검증 충돌 제거
  - 연결 워치독 + 자동 재연결(지수 백오프) → 공정 중간 끊김에 강함
  - 기존 명령 프로토콜 유지: "R,pin,val", "W,val", "D,val", "S", "r", "d", "P"
  - 기존 시그널 이름/의미 유지: status_message/rf_power_updated/dc_power_updated/command_confirmed/command_failed

참고:
  - 기존 동기식(serial.Serial) 구현을 QSerialPort 이벤트 구동형 구조로 변경.
  - MFC 비동기 설계와 동일한 패턴(큐, 콜백, 타임아웃, 재시도, 워치독) 적용.
"""
from __future__ import annotations
import time
import traceback
from collections import deque
from dataclasses import dataclass
from typing import Callable, Optional, Deque

from PyQt6.QtCore import QObject, QTimer, QIODeviceBase, pyqtSignal as Signal, pyqtSlot as Slot
from PyQt6.QtSerialPort import QSerialPort, QSerialPortInfo

# --- 프로젝트 상수/설정 ---
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
    # UI/상위 컨트롤러용 시그널(기존과 동일)
    status_message = Signal(str, str)
    rf_power_updated = Signal(float, float)
    dc_power_updated = Signal(float, float, float)
    command_confirmed = Signal(str)
    command_failed = Signal(str, str)

    # ------------- 초기화 -------------
    def __init__(self, parent=None):
        super().__init__(parent)

        self.debug_print = DEBUG_PRINT
        self._closing = False   # 종료 진행 플래그
        self._send_spin = False # [GUARD] _dequeue_and_send 재진입 가드

        # 오류 발생시 재연결 관련
        self._last_error_time = 0.0
        self._error_debounce_s = 1.0  # 1.0~2.0 정도 권장

        # (1) QSerialPort 설정
        self.serial_faduino = QSerialPort(self)
        self.serial_faduino.setBaudRate(FADUINO_BAUD)
        self.serial_faduino.setDataBits(QSerialPort.DataBits.Data8)
        self.serial_faduino.setParity(QSerialPort.Parity.NoParity)
        self.serial_faduino.setStopBits(QSerialPort.StopBits.OneStop)
        self.serial_faduino.setFlowControl(QSerialPort.FlowControl.NoFlowControl)
        self.serial_faduino.readyRead.connect(self._on_ready_read)
        self.serial_faduino.errorOccurred.connect(self._on_serial_error)

        # (2) 수신 버퍼(줄 단위 파싱) + 상한
        self._rx = bytearray()      # 반드시 bytearray 유지
        self._RX_MAX = 16 * 1024    # 상한(예: 16 KiB) — 필요하면 4/8/32 KiB로 조정
        self._LINE_MAX = 512       # 1줄 최대 길이(환경에 따라 256~1024로 조정)
        self._overflow_count = 0   # 오버플로우 발생 통계/알림 스로틀

        # (3) 명령 큐: (cmd_str, on_reply, timeout_ms, gap_ms, tag, retries_left, allow_no_reply)
        self._cmd_q: Deque[Command] = deque()
        self._inflight: Optional[Command] = None

        # (4) 타이머: 명령 타임아웃/인터커맨드 간격/폴링/워치독
        self._cmd_timer = QTimer(self)
        self._cmd_timer.setSingleShot(True)
        self._gap_timer = QTimer(self)
        self._gap_timer.setSingleShot(True)
        self._cmd_timer.timeout.connect(self._on_cmd_timeout)
        self._gap_timer.timeout.connect(self._dequeue_and_send)

        self.polling_timer = QTimer(self)
        self.polling_timer.setInterval(FADUINO_POLLING_INTERVAL_MS)
        self.polling_timer.timeout.connect(self._enqueue_poll_cycle)

        self._watchdog = QTimer(self)
        self._watchdog.setInterval(FADUINO_WATCHDOG_INTERVAL_MS)
        self._watchdog.timeout.connect(self._watch_connection)

        # (5) 연결/재연결 상태
        self._want_connected = False
        self._reconnect_backoff_ms = FADUINO_RECONNECT_BACKOFF_START_MS
        self._reconnect_pending = False # 중복 재연결 예약 방지

        # (6) 상태 값들
        self.expected_relay_mask = 0
        self._is_first_poll = True
        self.is_rf_active = False
        self.is_dc_active = False
        self.rf_forward = 0.0
        self.rf_reflected = 0.0
        self.dc_voltage = 0.0
        self.dc_current = 0.0

    # ------------- 내부 디버그 -------------
    def _dprint(self, *args):
        if self.debug_print:
            try:
                print(*args, flush=True)
            except Exception:
                pass

    # ------------- 연결/해제 & 워치독 -------------
    def connect_faduino(self) -> bool:
        """포트를 열고 워치독 시작. 실패해도 워치독이 재연결 시도."""
        self._want_connected = True
        ok = self._open_port()
        self._watchdog.start()
        return ok

    def _open_port(self) -> bool:
        if self.serial_faduino.isOpen():
            return True

        available = {p.portName() for p in QSerialPortInfo.availablePorts()}
        if FADUINO_PORT not in available:
            msg = f"{FADUINO_PORT} 존재하지 않음. 사용 가능 포트: {sorted(available)}"
            self.status_message.emit("Faduino", msg)
            self._dprint(f"[FAD] {msg}")
            return False

        self.serial_faduino.setPortName(FADUINO_PORT)
        if not self.serial_faduino.open(QIODeviceBase.OpenModeFlag.ReadWrite):
            msg = f"{FADUINO_PORT} 연결 실패: {self.serial_faduino.errorString()}"
            self.status_message.emit("Faduino", msg) 
            self._dprint(f"[FAD] {msg}")
            return False

        # 라인 제어 및 버퍼 초기화
        self.serial_faduino.setDataTerminalReady(True)
        self.serial_faduino.setRequestToSend(False)
        self.serial_faduino.clear(QSerialPort.Direction.AllDirections)
        self._rx.clear()
        self._reconnect_backoff_ms = FADUINO_RECONNECT_BACKOFF_START_MS
        self._reconnect_pending = False  # ← 선택이지만 넣어두면 깔끔
        msg = f"{FADUINO_PORT} 연결 성공 (PyQt6 QSerialPort)"
        self.status_message.emit("Faduino", msg) 
        self._dprint(f"[FAD] {msg}")
        return True

    def _watch_connection(self):
        """주기적으로 연결 상태 확인. 끊겨 있으면 백오프로 재연결."""
        if not self._want_connected or self.serial_faduino.isOpen():
            return
        
        # [GUARD] 이미 예약된 재연결이 있으면 중복 예약 금지  ← 추가
        if self._reconnect_pending:
            self._dprint("[RECON] reconnect already scheduled; skip")
            return
        self._reconnect_pending = True
        
        msg = f"재연결 시도... ({self._reconnect_backoff_ms} ms)"
        self.status_message.emit("Faduino", msg)
        self._dprint(f"[FAD] {msg}")
        QTimer.singleShot(self._reconnect_backoff_ms, self._try_reconnect)

    def _try_reconnect(self):
        # [GUARD] 예약된 시도 소모(해제)  ← 추가
        self._reconnect_pending = False

        # 종료 중이거나 재연결 의사가 없으면 즉시 중단  ← 추가
        if getattr(self, "_closing", False) or not self._want_connected:
            self._dprint("[RECON] skip: closing or not wanted")
            return

        # 이미 열려 있으면 불필요한 재연결 방지  ← 추가
        if self.serial_faduino.isOpen():
            return
        
        if self._open_port():
            msg = "재연결 성공. 대기 중 명령 재개."
            self.status_message.emit("Faduino", msg) 
            self._dprint(f"[FAD] {msg}")

            if self._cmd_q:
                QTimer.singleShot(0, self._dequeue_and_send)
            return
        self._reconnect_backoff_ms = min(self._reconnect_backoff_ms * 2, FADUINO_RECONNECT_BACKOFF_MAX_MS)

    @Slot()
    def cleanup(self):
        """안전 종료: 타이머/큐/포트 정리 + 베스트에포트로 출력 OFF."""
        # 0) 중복 호출 가드 + 종료 진입 선언
        if self._closing:
            self._dprint("[CLOSE] cleanup already in progress")
            return
        self._closing = True
        self._want_connected = False  # 워치독 재연결 의지 해제(선택)

        # 1. 타이머 정지
        self.polling_timer.stop()
        self._cmd_timer.stop() 
        self._gap_timer.stop() 
        self._watchdog.stop()
        self._reconnect_pending = False   # ← 추가: 종료 시 플래그 초기화

        # 2) 진행 중(inflight) 명령에 '취소(None)' 통지
        if self._inflight:
            try:
                # 타임아웃 타이머/상태 정리
                self._cmd_timer.stop()
                cmd = self._inflight
                self._inflight = None
                # 상위에 "응답 없음(취소)"로 알려 다음 로직을 풀어줌
                self._safe_callback(cmd.callback, None)
            except Exception as e:
                self._dprint(f"[WARN] cleanup inflight-cb failed: {e}")

        # 3) 큐에 대기 중인 모든 명령에도 '취소(None)' 통지
        try:
            while self._cmd_q:
                cmd = self._cmd_q.popleft()
                self._safe_callback(cmd.callback, None)
        except Exception as e:
            self._dprint(f"[WARN] cleanup queue-cb failed: {e}")

        # 4) RX 버퍼/가드 등 로컬 상태 정리
        try:
            self._rx.clear()
            self._send_spin = False  # 가드가 걸려 있어도 종료 시점에는 풀어둠(선택)
        except Exception:
            pass

        # 5) 베스트에포트로 모든 릴레이/아날로그 0 (무응답 허용)
        for pin in range(8):
            self.enqueue(f"R,{pin},0\r", lambda _l: None,
                        timeout_ms=CLEAN_TIMEOUT, gap_ms=FADUINO_GAP_MS,
                        tag=f"[CLEAN R {pin}]", retries_left=0, 
                        allow_no_reply=True, allow_when_closing=True)
        self.enqueue("W,0\r", lambda _l: None,
                    timeout_ms=CLEAN_TIMEOUT, gap_ms=FADUINO_GAP_MS,
                    tag="[CLEAN W]", retries_left=0, 
                    allow_no_reply=True, allow_when_closing=True)
        self.enqueue("D,0\r", lambda _l: None,
                    timeout_ms=CLEAN_TIMEOUT, gap_ms=FADUINO_GAP_MS,
                    tag="[CLEAN D]", retries_left=0, 
                    allow_no_reply=True, allow_when_closing=True)

        # 6) 잠시 후 포트 닫기
        QTimer.singleShot(200, self._close_now)

    def _close_now(self):
        if self.serial_faduino.isOpen():
            self.serial_faduino.close()
        self.status_message.emit("Faduino", "연결 종료")

    # ------------- 시리얼 이벤트 -------------
    def _on_serial_error(self, err: QSerialPort.SerialPortError):
        if err == QSerialPort.SerialPortError.NoError or self._closing:
            return
        
        # 연속 오류 디바운스
        now = time.monotonic()
        if now - self._last_error_time < self._error_debounce_s:
            self._dprint("[ERR] debounced serial error")
            return
        self._last_error_time = now

        err_name = getattr(err, "name", str(err))
        err_code = getattr(err, "value", "?")
        msg = f"시리얼 오류: {self.serial_faduino.errorString()} (err={err_name}/{err_code})"
        self.status_message.emit("Faduino", msg) 
        self._dprint(f"[ERR] {msg}")

        # 진행 중 명령 되돌리고 재시도 준비
        if self._inflight is not None:
            cmd = self._inflight
            self._cmd_timer.stop()
            self._inflight = None
            if cmd.retries_left > 0:
                cmd.retries_left -= 1
                self._cmd_q.appendleft(cmd)
            else:
                self._safe_callback(cmd.callback, None)

        # 포트만 닫고, 재연결은 워치독에 맡김
        if self.serial_faduino.isOpen():
            try:
                self.serial_faduino.close()
            except Exception as e:
                self._dprint(f"[WARN] close-on-error failed: {e}")

        # 에러 직후 갭 타이머 정지
        self._gap_timer.stop()

        # 에러 직후 남은 RX 꼬리 정리
        try:
            self._rx.clear()
        except Exception:
            pass

        # 재연결은 워치독이 처리
        return

    def _on_ready_read(self):
        ba = self.serial_faduino.readAll()
        if ba.isEmpty():
            return

        # 1) 버퍼 누적
        self._rx.extend(bytes(ba))

        # RX 오버플로우: 최근 RX_MAX 바이트만 유지(테일 보존, in-place)
        if len(self._rx) > self._RX_MAX:
            del self._rx[:-self._RX_MAX]  # 앞부분 통으로 버리고 꼬리만 보존
            self._overflow_count += 1
            if self._overflow_count % 5 == 1:
                try:
                    self.status_message.emit("Faduino", f"수신 버퍼 과다(RX>{self._RX_MAX}); 최근 {self._RX_MAX}B만 보존.")
                except Exception:
                    pass
            try:
                self._dprint(f"[WARN] RX overflow: keep tail {len(self._rx)}B")
            except Exception:
                pass

        # 3) 줄 경계(\r/\n) 기준으로 라인 단위 파싱
        while True:
            i_cr = self._rx.find(b'\r')
            i_lf = self._rx.find(b'\n')
            if i_cr == -1 and i_lf == -1:
                break

            idx = i_cr if i_lf == -1 else (i_lf if i_cr == -1 else min(i_cr, i_lf))
            line_bytes = self._rx[:idx]

            # CRLF 또는 LFCR 쌍을 한 번에 건너뛰기
            drop = idx + 1
            if drop < len(self._rx):
                ch = self._rx[idx]
                nxt = self._rx[idx + 1]
                if (ch == 13 and nxt == 10) or (ch == 10 and nxt == 13):
                    drop += 1
            del self._rx[:drop]

            # 앞부분(토큰 포함)만 보존하고 긴 꼬리는 잘라서 파서 안정성 유지
            over = len(line_bytes) - self._LINE_MAX
            if over > 0:
                self._dprint(f"[WARN] RX line too long (+{over}B), truncating tail; keep {self._LINE_MAX}B")
                line_bytes = line_bytes[:self._LINE_MAX]

            try:
                line = line_bytes.decode('ascii', errors='ignore').strip()
            except Exception:
                line = None

            if not line:
                continue

            # 에코 라인(보낸 명령 그대로) 스킵 — 문자열만 비교
            if self._inflight:
                sent_cmd_str = self._inflight.cmd_str.strip()
                if line == sent_cmd_str:
                    self._dprint(f"[RECV] echo skipped: {repr(line)}")
                    # 에코는 소비만 하고 다음 줄을 계속 본다
                    continue

            # 여기 도달하면 '실제 응답 1줄' 확보 → 처리하고 종료
            self._dprint(f"[RECV] {repr(line)}")
            self._finish_command(line)
            break  # 한 번에 한 줄만 넘기고 끝낸다
            # 여기서 return 하지 않음: 남은 데이터는 버퍼에 보존 → 다음 readyRead에서 처리

        # 4) 앞쪽에 남은 연속 CR/LF만 정리(빈 라인 재처리 방지)
        while self._rx[:1] in (b'\r', b'\n'):
            del self._rx[0:1]

    # ------------- 명령 큐(완전 비동기) -------------
    def enqueue(self, cmd_str: str, on_reply: Callable[[Optional[str]], None],
                timeout_ms: int = FADUINO_TIMEOUT_MS, gap_ms: int = FADUINO_GAP_MS,
                tag: str = "", retries_left: int = 3, 
                allow_no_reply: bool = False,
                allow_when_closing: bool = False):
        
        # 종료 중이면 외부 enqueue 차단(단, cleanup 내부만 예외)
        if self._closing and not allow_when_closing:
            self._dprint("[CLOSE] enqueue blocked")
            return
        
        if not cmd_str.endswith('\r'):
            cmd_str += '\r'
        cmd = Command(
            cmd_str=cmd_str,
            callback=on_reply,
            timeout_ms=timeout_ms,
            gap_ms=gap_ms,
            tag=tag,
            retries_left=retries_left,
            allow_no_reply=allow_no_reply,
        )
        self._cmd_q.append(cmd)
        if (self._inflight is None) and (not self._gap_timer.isActive()):
            QTimer.singleShot(0, self._dequeue_and_send)

    def _dequeue_and_send(self):
        # --- 사전 체크(빠른 리턴) ---
        # 이미 보낸 명령이 대기 중이거나, 큐가 비었거나, 포트가 닫혀있거나,
        # 인터커맨드 갭 타이머가 뛰는 중이면 보내지 않음.
        if self._inflight is not None or not self._cmd_q:
            return
        if not self.serial_faduino.isOpen():
            return
        if self._gap_timer.isActive():
            return

        # --- [GUARD] 재진입 차단 ---
        if self._send_spin:
            self._dprint("[GUARD] _dequeue_and_send re-enter blocked")
            return
        self._send_spin = True

        try:
            # 1) 큐에서 한 건 꺼내 인플라이트로 설정
            cmd = self._cmd_q.popleft()
            self._inflight = cmd
            self._rx.clear()

            # 2) 로깅/상태
            self._dprint(f"[SEND] {cmd.cmd_str.strip()} (tag={cmd.tag})")
            self.status_message.emit("Faduino > 전송", f"{cmd.tag or ''} {cmd.cmd_str.strip()}".strip())

            # 3) 페이로드 인코딩
            payload = cmd.cmd_str.encode('ascii')

            # 4) write 결과 확인 + 부분 쓰기(Partial write) 보강
            n = self.serial_faduino.write(payload)

            try:
                n_int = int(n)
            except Exception:
                n_int = -1

            if n_int <= 0:
                raise IOError(f"serial write returned {n_int}")
            
            total = n_int
            if total != len(payload):
                # 남은 바이트 한 번 더 밀어 넣기(한 번 추가 시도로 충분)
                remaining = payload[total:]
                m = self.serial_faduino.write(remaining)
                try:
                    m_int = int(m)
                except Exception:
                    m_int = -1
                if m_int > 0:
                    total += m_int

                if total != len(payload):
                    # 여전히 모자라면 실패로 간주 → except로 넘어가 복구/재시도
                    raise IOError(f"partial write: queued {total}/{len(payload)} bytes")

            # 일부 드라이버는 flush()가 no-op일 수 있지만 호출해도 무방
            self.serial_faduino.flush()

            # 명시적으로 stop→start 하면 상태가 더 명확해진다(권장)
            self._cmd_timer.stop()
            self._cmd_timer.start(cmd.timeout_ms)

        except Exception as e:
            # ---- 전송 실패: 상태 복구 + 재시도/재연결 예약 ----
            self._dprint(f"[ERROR] Send failed: {e}")

            failed_cmd = self._inflight
            self._inflight = None
            self._cmd_timer.stop()

            # 같은 명령을 다시 보내고 싶으면 맨 앞으로 재삽입
            if failed_cmd:
                self._cmd_q.appendleft(failed_cmd)

            # 포트 상태에 따라 재연결 or 간격 후 재시도
            try:
                if not self.serial_faduino.isOpen():
                    # 포트가 닫혀 있으면 재연결 루틴으로(0ms 지연으로 재진입 끊기)
                    QTimer.singleShot(0, self._try_reconnect)
                else:
                    # 포트는 열려 있는데 write 실패면, 약간의 간격 후 다시 디큐 시도
                    gap_ms = failed_cmd.gap_ms
                    self._gap_timer.start(gap_ms)
                    QTimer.singleShot(gap_ms + 1, self._dequeue_and_send)
            except Exception as ee:
                self._dprint(f"[WARN] reconnect/retry schedule failed: {ee}")

            self.status_message.emit("Faduino", f"전송 오류: {e}")
            return

        finally:
            # --- 가드 해제(성공/실패 모두) ---
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
        self._cmd_timer.stop()
        self._inflight = None

        if line is None:
            if cmd.allow_no_reply:
                self._dprint(f"[RECV] (no response allowed; proceed) tag={cmd.tag}")
                self._safe_callback(cmd.callback, None)
                self._gap_timer.start(cmd.gap_ms)
                return

            if cmd.retries_left > 0:
                self._dprint(f"[RETRY] re-enqueue (left={cmd.retries_left-1}) tag={cmd.tag}")
                cmd.retries_left -= 1
                self._cmd_q.appendleft(cmd)
                if self.serial_faduino.isOpen():
                    self.serial_faduino.close()
                QTimer.singleShot(0, self._try_reconnect)
                return
            else:
                self._safe_callback(cmd.callback, None)
                self._gap_timer.start(cmd.gap_ms)
                return

        # 정상 응답
        self._safe_callback(cmd.callback, (line or '').strip())
        self._gap_timer.start(cmd.gap_ms)

    # ------------- 폴링/프로세스 연동 -------------
    @Slot(bool)
    def set_process_status(self, should_poll: bool):
        if should_poll:
            if not self.polling_timer.isActive():
                self.status_message.emit("Faduino", "공정 감시 폴링 시작")
                self._dprint("[RUN] POLL START")
                self.polling_timer.start()
        else:
            if self.polling_timer.isActive():
                self.polling_timer.stop()
                self.status_message.emit("Faduino", "공정 감시 폴링 중지")
                self._dprint("[RUN] POLL STOP")

    def _enqueue_poll_cycle(self):
        # 통합 상태 읽기: S → OK_S,mask,rf_for,rf_ref,dc_v,dc_c
        def on_s(line: Optional[str]):
            p = self._parse_ok_and_compute(line or "")
            if p and p.get("type") == "ERROR":
                self.command_failed.emit("Faduino", p.get("msg", "ERROR"))
                return
            
            if not p or p.get("type") != "OK_S":
                return
            try:
                relay_mask = p["relay_mask"]
                # 릴레이 동기화
                if self._is_first_poll:
                    self.expected_relay_mask = relay_mask
                    self._is_first_poll = False
                    self.status_message.emit("Faduino", f"초기 릴레이 상태 동기화 완료: {relay_mask}")
                elif relay_mask != self.expected_relay_mask:
                    msg = f"릴레이 상태 불일치! 예상: {self.expected_relay_mask}, 실제: {relay_mask}"
                    self.status_message.emit("Faduino(경고)", msg)
                    self.command_failed.emit("Faduino", f"Relay 상태 확인 {msg}")
                # RF/DC 갱신
                if self.is_rf_active and "rf" in p:
                    self._update_rf(*p["rf"])
                if self.is_dc_active and "dc" in p:
                    self._update_dc(*p["dc"])
            except Exception:
                pass
        self.enqueue('S\r', on_s, timeout_ms=FADUINO_TIMEOUT_MS, gap_ms=FADUINO_GAP_MS, tag='[POLL S]')

    # ------------- 공개 API (상위에서 호출) -------------
    @Slot(str, bool)
    def handle_named_command(self, name: str, state: bool):
        if name not in BUTTON_TO_PIN:
            self.status_message.emit("Faduino", f"알 수 없는 버튼명: {name}")
            return
        pin = BUTTON_TO_PIN[name]
        self.set_relay(pin, state)

    @Slot(int, bool)
    def set_relay(self, pin: int, state: bool):
        cmd = f"R,{pin},{1 if state else 0}"  # ack: "ACK_R"
        def on_reply(line: Optional[str], pin=pin, state=state):
            if (line or '').strip() == 'ACK_R':
                if state:
                    self.expected_relay_mask |= (1 << pin)
                else:
                    self.expected_relay_mask &= ~(1 << pin)
                self.command_confirmed.emit(f"R,{pin},{1 if state else 0}")
                self.status_message.emit("Faduino", f"Relay({pin}) → {'ON' if state else 'OFF'}")
            else:
                self.command_failed.emit("R", f"Relay({pin}) 응답 불일치: {repr(line)}")
        self.enqueue(cmd, on_reply, timeout_ms=FADUINO_TIMEOUT_MS, gap_ms=FADUINO_GAP_MS, tag=f"[R {pin}]")

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
        # S는 통합 상태 응답("OK_S,mask,rf_for,rf_ref,dc_v,dc_c")
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
        self.enqueue('S\r', on_s, timeout_ms=FADUINO_TIMEOUT_MS, gap_ms=FADUINO_GAP_MS, tag='[FORCE S]')

    @Slot()
    def force_rf_read(self):
        # r → "OK_r,rf_for_raw,rf_ref_raw"
        def on_r(line: Optional[str]):
            p = self._parse_ok_and_compute(line or "")
            if p and p.get("type") == "ERROR":
                self.command_failed.emit("Faduino", p.get("msg", "ERROR"))
                return
            
            if not p or p.get("type") != "OK_r":
                return
            try:
                if self.is_rf_active and "rf" in p:
                    self._update_rf(*p["rf"])
            except Exception:
                pass
        self.enqueue('r\r', on_r, timeout_ms=FADUINO_TIMEOUT_MS, gap_ms=FADUINO_GAP_MS, tag='[FORCE r]')

    @Slot()
    def force_dc_read(self):
        # d → "OK_d,dc_v_raw,dc_c_raw"
        def on_d(line: Optional[str]):
            p = self._parse_ok_and_compute(line or "")
            if p and p.get("type") == "ERROR":
                self.command_failed.emit("Faduino", p.get("msg", "ERROR"))
                return
            
            if not p or p.get("type") != "OK_d":
                return
            try:
                if self.is_dc_active and "dc" in p:
                    self._update_dc(*p["dc"])
            except Exception:
                pass
        self.enqueue('d\r', on_d, timeout_ms=FADUINO_TIMEOUT_MS, gap_ms=FADUINO_GAP_MS, tag='[FORCE d]')

    @Slot()
    def force_pin_read(self):
        # P → "OK_P,relay_mask"
        def on_p(line: Optional[str]):
            p = self._parse_ok_and_compute(line or "")
            if p and p.get("type") == "ERROR":
                self.command_failed.emit("Faduino", p.get("msg", "ERROR"))
                return

            if not p or p.get("type") != "OK_P":
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
            except Exception:
                pass
        self.enqueue('P\r', on_p, timeout_ms=FADUINO_TIMEOUT_MS, gap_ms=FADUINO_GAP_MS, tag='[FORCE P]')

    # ------------- 통합 파싱 + 계산 (OK_* 전용) -------------
    def _parse_ok_and_compute(self, response: str):
        """
        한 줄 응답을 파싱하고, RF/DC는 self._compute_*()를 호출해 계산까지 수행.
        반환 예:
          {"type":"OK_S","relay_mask":int,"rf":(rf_for_w,rf_ref_w),"dc":(dc_p_w,dc_v_v,dc_c_a)}
          {"type":"OK_r","rf":(rf_for_w,rf_ref_w)}
          {"type":"OK_d","dc":(dc_p_w,dc_v_v,dc_c_a)}
          {"type":"OK_P","relay_mask":int}
          {"type":"ACK_R"} / {"type":"ACK_W"} / {"type":"ACK_D"}
          {"type":"ERROR","msg":str}
        포맷 불일치/변환 실패 시 None
        """
        s = (response or "").strip()

        # ---- 전체 상태: OK_S,<relay>,<rf_for>,<rf_ref>,<dc_v>,<dc_c>
        if s.startswith("OK_S,"):
            parts = s.split(",")
            if len(parts) != 6:
                return None
            try:
                relay_mask = int(parts[1])
                rf_for, rf_ref = self._compute_rf(parts[2], parts[3])
                dc_p, dc_v, dc_c = self._compute_dc(parts[4], parts[5])
                return {"type": "OK_S", "relay_mask": relay_mask, "rf": (rf_for, rf_ref), "dc": (dc_p, dc_v, dc_c)}
            except Exception:
                return None

        # ---- 릴레이 상태만: OK_P,<relay_mask>
        if s.startswith("OK_P,"):
            parts = s.split(",")
            if len(parts) != 2:
                return None
            try:
                return {"type": "OK_P", "relay_mask": int(parts[1])}
            except Exception:
                return None

        # ---- RF만: OK_r,<rf_for>,<rf_ref>
        if s.startswith("OK_r,"):
            parts = s.split(",")
            if len(parts) != 3:
                return None
            try:
                rf_for, rf_ref = self._compute_rf(parts[1], parts[2])
                return {"type": "OK_r", "rf": (rf_for, rf_ref)}
            except Exception:
                return None

        # ---- DC만: OK_d,<dc_v>,<dc_c>
        if s.startswith("OK_d,"):
            parts = s.split(",")
            if len(parts) != 3:
                return None
            try:
                dc_p, dc_v, dc_c = self._compute_dc(parts[1], parts[2])
                return {"type": "OK_d", "dc": (dc_p, dc_v, dc_c)}
            except Exception:
                return None

        # ---- ACK / ERROR
        if s in ("ACK_R", "ACK_W", "ACK_D"):
            return {"type": s}
        if s.startswith("ERROR"):
            return {"type": "ERROR", "msg": s}

        return None

    # ------------- 공용 계산 함수 -------------
    def _compute_rf(self, rf_for_raw, rf_ref_raw):
        rf_for_raw = float(rf_for_raw)
        rf_ref_raw = float(rf_ref_raw)
        rf_for = max(0.0, (RF_PARAM_ADC_TO_WATT * rf_for_raw) + RF_OFFSET_ADC_TO_WATT)
        rf_ref_v = (rf_ref_raw / ADC_FULL_SCALE) * ADC_INPUT_VOLT
        rf_ref = max(0.0, rf_ref_v * RF_WATT_PER_VOLT)
        return rf_for, rf_ref

    def _compute_dc(self, dc_v_raw, dc_c_raw):
        dc_v_raw = float(dc_v_raw)
        dc_c_raw = float(dc_c_raw)
        dc_v = max(0.0, (DC_PARAM_ADC_TO_VOLT * dc_v_raw) + DC_OFFSET_ADC_TO_VOLT)
        dc_c = max(0.0, (DC_PARAM_ADC_TO_AMP  * dc_c_raw) + DC_OFFSET_ADC_TO_AMP)
        dc_p = dc_v * dc_c
        return dc_p, dc_v, dc_c
    
    # ------------- 공용 업데이트 함수 -------------
    def _update_rf(self, rf_for, rf_ref):
        self.rf_forward, self.rf_reflected = rf_for, rf_ref
        self.rf_power_updated.emit(rf_for, rf_ref)

    def _update_dc(self, dc_p, dc_v, dc_c):
        self.dc_voltage, self.dc_current = dc_v, dc_c
        self.dc_power_updated.emit(dc_p, dc_v, dc_c)

    # ------------- 상태 플래그 연동 -------------
    @Slot(bool)
    def on_rf_state_changed(self, is_active: bool):
        self.is_rf_active = is_active
        self.status_message.emit("Faduino", f"RF 컨트롤러 상태 감지: {'활성' if is_active else '비활성'}")

    @Slot(bool)
    def on_dc_state_changed(self, is_active: bool):
        self.is_dc_active = is_active
        self.status_message.emit("Faduino", f"DC 컨트롤러 상태 감지: {'활성' if is_active else '비활성'}")

    # ------------- 유틸 -------------
    def _clamp_dac(self, value: int) -> int:
        try:
            v = int(round(value))
        except Exception:
            v = 0
        if v < 0: v = 0
        if v > DAC_FULL_SCALE: v = DAC_FULL_SCALE
        return v
    
    def _safe_callback(self, callback: Callable, *args):
        """안전한 콜백 실행"""
        try:
            callback(*args)
        except Exception as e:
            self._dprint(traceback.format_exc())
            self._dprint(f"[ERROR] Callback failed: {e}")
            self.status_message.emit("Faduino", f"콜백 오류: {e}")

