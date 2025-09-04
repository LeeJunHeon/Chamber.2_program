# controller/chat_notifier.py
from PyQt6.QtCore import QObject, pyqtSlot as Slot
import json, urllib.request, urllib.error, ssl
from typing import Optional, List

class ChatNotifier(QObject):
    """
    Google Chat 웹훅으로 텍스트/리치카드 메시지를 보낸다.

    변경점:
    - 전송 지연 모드(기본 on): 공정 중 발생하는 모든 알림을 버퍼에 쌓고,
      공정이 완전히 끝난 뒤(process_finished) 한 번에 전송하여 UI 멈춤을 방지.
    - 네트워크 오류는 삼켜서 UI를 멈추지 않음.
    """

    def __init__(self, webhook_url: Optional[str], parent=None):
        super().__init__(parent)
        self.webhook_url = (webhook_url or "").strip()
        self._ctx = ssl.create_default_context()

        # === 전송 지연 모드/버퍼 ===
        self._defer: bool = True           # 공정 중엔 전송하지 않고 모아두기 (기본 True)
        self._buffer: List[dict] = []      # 전송 대기 payload들
        self._last_started_params: Optional[dict] = None  # (선택) 시작 정보 보관

    # ---------- 지연 전송 제어 ----------
    def set_defer(self, on: bool):
        """전송 지연 모드 on/off"""
        self._defer = bool(on)

    def flush(self):
        """지연 전송된 payload들을 순서대로 전송"""
        if not self.webhook_url or not self._buffer:
            self._buffer.clear()
            return
        for payload in self._buffer:
            self._do_post(payload)
        self._buffer.clear()

    # ---------- 내부 HTTP ----------
    def _do_post(self, payload: dict):
        """실제 HTTP 전송 (urlopen 사용)"""
        if not self.webhook_url:
            return
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            self.webhook_url, data=data,
            headers={"Content-Type": "application/json"}
        )
        try:
            # timeout은 짧게 유지(기본 3초)
            with urllib.request.urlopen(req, timeout=3, context=self._ctx) as resp:
                resp.read()
        except Exception:
            # 로그를 남기고 싶다면 여기서 print나 별도 Signal로 전달
            pass

    def _post_json(self, payload: dict, urgent: bool = False):
        """
        지연 모드가 켜져 있고 급하지 않으면 버퍼에 쌓고,
        그렇지 않으면 즉시 전송.
        """
        if self._defer and not urgent:
            self._buffer.append(payload)
            return
        self._do_post(payload)

    # ---------- 텍스트 ----------
    def _post_text(self, text: str, urgent: bool = False):
        if text:
            self._post_json({"text": text}, urgent=urgent)

    # ---------- 카드 빌더 ----------
    def _post_card(self, title: str, subtitle: str = "", status: str = "INFO",
                   fields: Optional[dict] = None, urgent: bool = False):
        """
        Cards v2 포맷 (Google Chat 웹훅)
        status: INFO | SUCCESS | FAIL
        """
        icon = {"INFO": "ℹ️", "SUCCESS": "✅", "FAIL": "❌"}.get(status, "ℹ️")
        widgets = []

        # 1) 타이틀
        widgets.append({
            "textParagraph": {"text": f"<b>{icon} {title}</b>"}
        })

        # 2) 서브타이틀
        if subtitle:
            widgets.append({"textParagraph": {"text": subtitle}})

        # 3) keyValue(필드)
        if fields:
            for k, v in fields.items():
                widgets.append({
                    "decoratedText": {
                        "topLabel": str(k),
                        "text": str(v)
                    }
                })

        payload = {
            "cardsV2": [
                {
                    "cardId": "notify-card",
                    "card": {
                        "header": {
                            "title": "Sputter Controller",
                            "subtitle": "Status Notification"
                        },
                        "sections": [
                            {"widgets": widgets}
                        ]
                    }
                }
            ]
        }
        self._post_json(payload, urgent=urgent)

    # ========== 슬롯들 ==========
    @Slot(dict)
    def notify_process_started(self, params: dict):
        """
        공정 시작 알림: 지연 모드에서는 즉시 전송하지 않고 버퍼에 쌓임.
        """
        self._last_started_params = dict(params) if params else None
        note = params.get("process_note", "") or params.get("Process_name", "")
        self._post_card("공정 시작", note, "INFO", fields={
            "Targets": f"{params.get('G1 Target','')}, {params.get('G2 Target','')}, {params.get('G3 Target','')}".strip(", "),
            "RF": f"{params.get('rf_power',0)} W" if params.get("use_rf_power") else "-",
            "RF Pulse": f"{params.get('rf_pulse_power',0)} W" if params.get("use_rf_pulse") else "-",
            "DC": f"{params.get('dc_power',0)} W" if params.get("use_dc_power") else "-",
            "Time": f"{params.get('process_time',0)} s"
        })

    @Slot(bool)
    def notify_process_finished(self, ok: bool):
        """
        공정 종료 알림: 종료 카드 생성 후 버퍼를 한 번에 전송(flush).
        """
        self._post_card("공정 종료", "성공" if ok else "실패",
                        "SUCCESS" if ok else "FAIL")
        self.flush()

    @Slot(str)
    def notify_text(self, text: str):
        # 필요 시 텍스트만 보낼 수도 있음 (지연 모드면 버퍼에 쌓임)
        self._post_text(text)

    @Slot(str)
    def notify_error(self, reason: str):
        # 오류도 공정 종료 후에 보내려면 urgent=False 유지
        # 즉시 보내고 싶다면 urgent=True로 변경
        self._post_card("장비 오류", reason, "FAIL", urgent=False)

    @Slot(str, str)
    def notify_error_with_src(self, src: str, reason: str):
        self._post_card("장비 오류", f"[{src}] {reason}", "FAIL", urgent=False)
