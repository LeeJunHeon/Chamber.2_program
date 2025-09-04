# controller/chat_notifier.py
from PyQt6.QtCore import (
    QObject, QThread, Qt,
    pyqtSignal as Signal, pyqtSlot as Slot
)
import json, urllib.request, urllib.error, ssl
from typing import Optional, List, Dict, Any


class _HttpWorker(QObject):
    """
    실제 네트워크 전송을 담당하는 워커(별도 스레드).
    메인/UI 스레드를 막지 않도록 모든 HTTP는 여기서만 수행한다.
    """
    def __init__(self, webhook_url: str, parent=None):
        super().__init__(parent)
        self.webhook_url = (webhook_url or "").strip()
        self._ctx = ssl.create_default_context()

    @Slot(dict)
    def post(self, payload: dict):
        if not self.webhook_url or not payload:
            return
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            self.webhook_url, data=data,
            headers={"Content-Type": "application/json"}
        )
        try:
            # 네트워크 지연/오류는 여기서만 발생하고, UI는 전혀 영향 없음
            with urllib.request.urlopen(req, timeout=3, context=self._ctx) as resp:
                resp.read()
        except Exception:
            # 네트워크 실패는 삼킴(로그를 남기려면 여기서 처리)
            pass


class ChatNotifier(QObject):
    """
    Google Chat 웹훅으로 텍스트/리치카드 메시지를 보낸다.

    - 시작 알림은 즉시 전송(urgent=True).
    - 공정 중 알림/오류는 버퍼에 쌓고, process_finished 시 한 번에 flush.
    - 모든 HTTP는 전용 워커스레드에서 비동기로 수행하여 UI 프리징을 방지.
    """
    # ChatNotifier -> _HttpWorker (스레드 경계) 신호
    _post_payload = Signal(dict)

    def __init__(self, webhook_url: Optional[str], parent=None):
        super().__init__(parent)
        self.webhook_url = (webhook_url or "").strip()

        # 전송 지연 & 버퍼
        self._defer: bool = True
        self._buffer: List[dict] = []

        # 시작/종료 컨텍스트
        self._last_started_params: Optional[dict] = None
        self._errors: List[str] = []          # 종료 카드 요약용 오류 모음
        self._finished_sent: bool = False     # 종료 카드 중복 방지

        # ---- 전송 워커 스레드 구성 ----
        self._thread = QThread(self)
        self._worker = _HttpWorker(self.webhook_url)
        self._worker.moveToThread(self._thread)
        self._thread.setObjectName("ChatNotifierThread")
        self._post_payload.connect(self._worker.post, type=Qt.ConnectionType.QueuedConnection)

        # ▶ 추가: 스레드 종료 시 워커를 그 스레드 안에서 안전하게 삭제
        self._thread.finished.connect(self._worker.deleteLater)

    # ---------- 라이프사이클 ----------
    def start(self):
        """메인에서 ChatNotifier 생성 직후 한 번 호출."""
        if not self._thread.isRunning():
            self._thread.start()

    def shutdown(self):
        """앱 종료 시 안전하게 버퍼 flush 후 스레드 종료."""
        self.flush()
        self._thread.quit()
        self._thread.wait(2000)

    # ---------- 내부 전송 ----------
    def _do_post(self, payload: dict):
        if not self._thread.isRunning():
            self._thread.start()
        self._post_payload.emit(payload)

    # ---------- 지연 전송 ----------
    def set_defer(self, on: bool):
        self._defer = bool(on)

    def flush(self):
        if not self._buffer:
            self._buffer.clear()
            return
        for payload in self._buffer:
            self._do_post(payload)
        self._buffer.clear()

    def _post_json(self, payload: dict, urgent: bool = False):
        if self._defer and not urgent:
            self._buffer.append(payload)
            return
        self._do_post(payload)

    # ---------- 텍스트 ----------
    def _post_text(self, text: str, urgent: bool = False):
        if text:
            self._post_json({"text": text}, urgent=urgent)

    # ---------- 카드 ----------
    def _post_card(self, title: str, subtitle: str = "", status: str = "INFO",
                   fields: Optional[Dict[str, Any]] = None, urgent: bool = False):
        icon = {"INFO": "ℹ️", "SUCCESS": "✅", "FAIL": "❌"}.get(status, "ℹ️")
        widgets = [{"textParagraph": {"text": f"<b>{icon} {title}</b>"}}]
        if subtitle:
            widgets.append({"textParagraph": {"text": subtitle}})
        if fields:
            for k, v in fields.items():
                widgets.append({"decoratedText": {"topLabel": str(k), "text": str(v)}})

        payload = {
            "cardsV2": [
                {
                    "cardId": "notify-card",
                    "card": {
                        "header": {
                            "title": "Sputter Controller",
                            "subtitle": "Status Notification"
                        },
                        "sections": [{"widgets": widgets}]
                    }
                }
            ]
        }
        self._post_json(payload, urgent=urgent)

    # ---------- 포맷 헬퍼 ----------
    def _b(self, params: dict, key: str) -> bool:
        v = params.get(key, False)
        if isinstance(v, str):
            v = v.strip().lower()
            return v in ("1", "t", "true", "y", "yes")
        return bool(v)

    def _fmt_min(self, v) -> str:
        try:
            f = float(v)
        except Exception:
            return "—"
        if f <= 0:
            return "—"
        # 분 단위 표시(정수면 정수로)
        return f"{int(f)}분" if abs(f - int(f)) < 1e-6 else f"{f:.1f}분"

    def _guns_and_targets(self, p: dict) -> str:
        out = []
        for gun, use_key, name_key in (("G1", "use_g1", "G1_target_name"),
                                       ("G2", "use_g2", "G2_target_name"),
                                       ("G3", "use_g3", "G3_target_name")):
            if self._b(p, use_key):
                name = (p.get(name_key) or "").strip() or "-"
                out.append(f"{gun}: {name}")
        return ", ".join(out) if out else "—"

    def _power_summary(self, p: dict) -> str:
        items = []
        if self._b(p, "use_dc_power"):
            items.append(f"DC {p.get('dc_power', 0)} W")
        if self._b(p, "use_rf_power"):
            items.append(f"RF {p.get('rf_power', 0)} W")
        if self._b(p, "use_rf_pulse"):
            f = p.get("rf_pulse_freq")
            d = p.get("rf_pulse_duty")
            freq_txt = f"{int(f)} Hz" if isinstance(f, (int, float)) and f is not None else "keep"
            duty_txt = f"{int(d)} %" if isinstance(d, (int, float)) and d is not None else "keep"
            items.append(f"RF Pulse {p.get('rf_pulse_power', 0)} W @ {freq_txt}, {duty_txt}")
        return " / ".join(items) if items else "—"

    # ========== 슬롯들 ==========
    @Slot(dict)
    def notify_process_started(self, params: dict):
        # 실행마다 초기화
        self._last_started_params = dict(params) if params else None
        self._errors.clear()
        self._finished_sent = False
        self._buffer.clear()  # 이전 잔여 버퍼 방지

        name = (params or {}).get("process_note") or (params or {}).get("Process_name") or "Untitled"
        guns = self._guns_and_targets(params or {})
        pwr  = self._power_summary(params or {})
        sh_delay = self._fmt_min((params or {}).get("shutter_delay", 0))
        proc_time = self._fmt_min((params or {}).get("process_time", 0))

        self._post_card(
            title="공정 시작",
            subtitle=name,
            status="INFO",
            fields={
                "사용 Guns / 타겟": guns,
                "파워": pwr,
                "Shutter Delay": sh_delay,
                "Process Time": proc_time,
            },
            urgent=True    # 시작 알림은 즉시 전송
        )

    @Slot(bool, dict)
    def notify_process_finished_detail(self, ok: bool, detail: dict):
        """
        ProcessController.process_finished_detail(bool, dict) 연결용
        dict 예:
          {
            'process_name': str,
            'stopped': bool,         # 사용자 Stop
            'aborting': bool,        # 긴급/비상
            'errors': List[str]      # 종료 중 누적 실패
          }
        """
        name = (detail or {}).get("process_name") or (self._last_started_params or {}).get("process_note") or "Untitled"
        stopped = bool((detail or {}).get("stopped"))
        aborting = bool((detail or {}).get("aborting"))
        errs: List[str] = list((detail or {}).get("errors") or [])
        if not errs and self._errors:
            errs = list(self._errors)

        if ok:
            subtitle = "정상 종료"
            fields = {"공정 이름": name}
            status = "SUCCESS"
        else:
            if stopped:
                subtitle = "사용자 Stop으로 종료"
                fields = {"공정 이름": name}
            elif aborting:
                subtitle = "긴급 중단으로 종료"
                fields = {"공정 이름": name}
            else:
                subtitle = "오류로 종료"
                if errs:
                    preview = " • " + "\n • ".join(errs[:3])
                    if len(errs) > 3:
                        preview += f"\n(+{len(errs)-3}건 더)"
                    fields = {"공정 이름": name, "원인": preview}
                else:
                    fields = {"공정 이름": name, "원인": "알 수 없음"}
            status = "FAIL"

        self._post_card("공정 종료", subtitle, status, fields)
        self.flush()
        self._finished_sent = True
        self._errors.clear()

    @Slot(bool)
    def notify_process_finished(self, ok: bool):
        """
        구버전 연결용(상세신호를 사용하지 않을 때).
        상세 종료 카드가 이미 전송되었으면 무시한다.
        """
        if self._finished_sent:
            self.flush()
            return
        self._post_card("공정 종료", "성공" if ok else "실패",
                        "SUCCESS" if ok else "FAIL",
                        fields={"공정 이름": (self._last_started_params or {}).get("process_note", "Untitled")})
        self.flush()
        self._finished_sent = True

    @Slot(str)
    def notify_text(self, text: str):
        self._post_text(text)

    @Slot(str)
    def notify_error(self, reason: str):
        pretty = reason or "unknown"
        self._errors.append(pretty)
        self._post_card("장비 오류", pretty, "FAIL", urgent=False)

    @Slot(str, str)
    def notify_error_with_src(self, src: str, reason: str):
        pretty = f"[{src}] {reason}" if src else (reason or "unknown")
        self._errors.append(pretty)
        self._post_card("장비 오류", pretty, "FAIL", urgent=False)
