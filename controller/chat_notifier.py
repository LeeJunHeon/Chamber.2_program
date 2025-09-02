# controller/chat_notifier.py
from PyQt6.QtCore import QObject, pyqtSlot as Slot
import json, urllib.request, urllib.error, ssl

class ChatNotifier(QObject):
    def __init__(self, webhook_url: str | None, parent=None):
        super().__init__(parent)
        self.webhook_url = webhook_url or ""

        # 일부 환경(사내 프록시/SSL 검사)에서 필요 없다면 제거해도 됨
        self._ctx = ssl.create_default_context()

    def _post_text(self, text: str):
        if not self.webhook_url or not text:
            return
        payload = {"text": text}
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            self.webhook_url, data=data,
            headers={"Content-Type": "application/json"}
        )
        try:
            with urllib.request.urlopen(req, timeout=5, context=self._ctx) as resp:
                resp.read()  # 응답 본문은 사용 안 해도 OK
        except Exception as e:
            # 여기서 예외를 먹어 UI/프로세스가 멈추지 않게 함
            # 필요하면 status_message 같은 신호로 로깅 가능
            pass

    # --- 슬롯들 ---
    @Slot(bool)
    def notify_process_finished(self, ok: bool):
        icon = "✅" if ok else "❌"
        self._post_text(f"{icon} 공정 종료: {'성공' if ok else '실패'}")

    @Slot(str)
    def notify_text(self, text: str):
        self._post_text(text)

    @Slot(str)
    def notify_error(self, reason: str):
        self._post_text(f"❌ 장비 오류: {reason}")

    @Slot(str, str)
    def notify_error_with_src(self, src: str, reason: str):
        self._post_text(f"❌ [{src}] 오류: {reason}")
