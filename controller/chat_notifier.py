# controller/chat_notifier.py
from PyQt6.QtCore import QObject, pyqtSlot as Slot
import json, urllib.request, urllib.error, ssl
from typing import Optional

class ChatNotifier(QObject):
    """
    Google Chat 웹훅으로 텍스트/리치카드 메시지를 보낸다.
    - 네트워크 오류는 삼켜서 UI를 멈추지 않음
    - 슬롯 시그니처를 맞춰 Process/Device 신호에 바로 연결 가능
    """
    def __init__(self, webhook_url: Optional[str], parent=None):
        super().__init__(parent)
        self.webhook_url = (webhook_url or "").strip()
        self._ctx = ssl.create_default_context()

    # ========== 내부 HTTP ==========
    def _post_json(self, payload: dict):
        if not self.webhook_url:
            return
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            self.webhook_url, data=data,
            headers={"Content-Type": "application/json"}
        )
        try:
            with urllib.request.urlopen(req, timeout=3, context=self._ctx) as resp:
                resp.read()
        except Exception:
            # 로그를 남기고 싶다면 여기서 print나 별도 Signal로 전달
            pass

    # ========== 텍스트 ==========
    def _post_text(self, text: str):
        if text:
            self._post_json({"text": text})

    # ========== 카드 빌더 ==========
    def _post_card(self, title: str, subtitle: str = "", status: str = "INFO", fields: Optional[dict]=None):
        """
        Cards v2 포맷 (Google Chat 웹훅)
        status: INFO | SUCCESS | FAIL
        """
        icon = {"INFO":"ℹ️", "SUCCESS":"✅", "FAIL":"❌"}.get(status, "ℹ️")
        widgets = []

        # 1) 타이틀
        widgets.append({
            "textParagraph": { "text": f"<b>{icon} {title}</b>" }
        })

        # 2) 서브타이틀
        if subtitle:
            widgets.append({
                "textParagraph": { "text": subtitle }
            })

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
                            { "widgets": widgets }
                        ]
                    }
                }
            ]
        }
        self._post_json(payload)

    # ========== 슬롯들 ==========
    @Slot(dict)
    def notify_process_started(self, params: dict):
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
        self._post_card("공정 종료", "성공" if ok else "실패", "SUCCESS" if ok else "FAIL")

    @Slot(str)
    def notify_text(self, text: str):
        # 필요 시 텍스트만 보낼 수도 있음
        self._post_text(text)

    @Slot(str)
    def notify_error(self, reason: str):
        self._post_card("장비 오류", reason, "FAIL")

    @Slot(str, str)
    def notify_error_with_src(self, src: str, reason: str):
        self._post_card("장비 오류", f"[{src}] {reason}", "FAIL")
