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
    - 공정 '시작' 알림은 반드시 즉시 전송(urgent=True), 이전 버퍼/상태 초기화.
    - 종료 알림은 정상/사용자중지/오류 종료를 구분해 표시.
    - 네트워크 오류는 삼켜서 UI를 멈추지 않음.
    """

    def __init__(self, webhook_url: Optional[str], parent=None):
        super().__init__(parent)
        self.webhook_url = (webhook_url or "").strip()
        self._ctx = ssl.create_default_context()

        # === 전송 지연 모드/버퍼 ===
        self._defer: bool = True           # 공정 중엔 전송하지 않고 모아두기 (기본 True)
        self._buffer: List[dict] = []      # 전송 대기 payload들

        # === 상태 기억 ===
        self._last_started_params: Optional[dict] = None
        self._stop_pressed: bool = False
        self._last_error_src: Optional[str] = None
        self._last_error_reason: Optional[str] = None

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
            # 필요 시 여기서 로그 출력/Signal 처리 가능
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
            # dict의 삽입 순서가 유지되므로, 구성 순서대로 나옵니다.
            for k, v in fields.items():
                if v is None or v == "":
                    continue
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

    # ---------- 포맷 유틸 ----------
    def _fmt_min(self, v) -> str:
        try:
            m = float(v)
        except Exception:
            return "-"
        if m <= 0:
            return "0 min"
        sec = int(round(m * 60.0))
        return f"{m:g} min ({sec}s)"

    def _fmt_power(self, on: bool, watts) -> str:
        if not on:
            return "-"
        try:
            w = float(watts)
        except Exception:
            w = 0.0
        return f"{w:g} W"

    def _build_start_fields(self, params: dict) -> dict:
        # 공정 이름
        process_name = (params.get("process_note") or
                        params.get("Process_name") or "-")

        # 사용 Gun + 타겟
        guns_used = []
        per_gun = []
        for g in ("G1", "G2", "G3"):
            used = bool(params.get(f"use_{g.lower()}", False))
            tgt = (params.get(f"{g} Target") or "").strip()
            if used:
                guns_used.append(g)
                per_gun.append(f"{g}: {tgt or '-'}")
        guns_field = ", ".join(guns_used) if guns_used else "-"

        # 파워 (RF / RF Pulse / DC)
        use_rf = bool(params.get("use_rf_power", False))
        use_rfp = bool(params.get("use_rf_pulse", False))
        use_dc = bool(params.get("use_dc_power", False))

        rf_w = self._fmt_power(use_rf, params.get("rf_power", 0))
        dc_w = self._fmt_power(use_dc, params.get("dc_power", 0))

        # RF Pulse는 (W, 선택적으로 f, duty)
        if use_rfp:
            try:
                pw = float(params.get("rf_pulse_power", 0))
            except Exception:
                pw = 0.0
            freq = params.get("rf_pulse_freq", None)
            duty = params.get("rf_pulse_duty", None)
            parts = [f"{pw:g} W"]
            if freq is not None:
                parts.append(f"{int(freq)} Hz")
            if duty is not None:
                parts.append(f"{int(duty)} %")
            rfp_txt = ", ".join(parts)
        else:
            rfp_txt = "-"

        # Shutter Delay / Process Time (분)
        shutter_delay_min = params.get("shutter_delay", 0)
        process_time_min  = params.get("process_time", 0)

        fields = {}
        fields["공정 이름"]   = process_name
        fields["사용 Gun"]   = guns_field
        if per_gun:
            fields["타겟"]       = " / ".join(per_gun)  # G1: Ti / G2: ... 형태
        fields["RF Power"]  = rf_w
        fields["RF Pulse"]  = rfp_txt
        fields["DC Power"]  = dc_w
        fields["Shutter Delay"] = self._fmt_min(shutter_delay_min)
        fields["Process Time"]  = self._fmt_min(process_time_min)
        return fields

    # ========== 슬롯들 ==========
    @Slot(dict)
    def notify_process_started(self, params: dict):
        """
        공정 시작 알림: 시작 알림은 즉시 전송(urgent=True).
        이전 실행에서 남았을 수 있는 버퍼/상태는 초기화.
        """
        # 상태 초기화
        self._last_started_params = dict(params) if params else None
        self._stop_pressed = False
        self._last_error_src = None
        self._last_error_reason = None

        # 이전 공정의 잔여 버퍼 제거(보수적으로)
        if self._buffer:
            self._buffer.clear()

        # 카드 전송
        fields = self._build_start_fields(self._last_started_params or {})
        subtitle = fields.pop("공정 이름", "")
        self._post_card(
            "공정 시작",
            subtitle,
            "INFO",
            fields=fields,
            urgent=True  # ★ 시작은 즉시 전송
        )

    @Slot(bool)
    def notify_process_finished(self, ok: bool):
        """
        공정 종료 알림: 정상/중지/오류를 구분해서 카드 생성 후,
        버퍼를 한 번에 전송(flush).
        """
        if ok:
            subtitle = "정상 종료"
            status = "SUCCESS"
            fields = None
        else:
            # 우선순위: 오류 기록 > stop 눌림 > 일반 중단
            if self._last_error_reason:
                src = f"[{self._last_error_src}] " if self._last_error_src else ""
                subtitle = "오류로 종료"
                status = "FAIL"
                fields = {"사유": f"{src}{self._last_error_reason}"}
            elif self._stop_pressed:
                subtitle = "사용자 중지"
                status = "INFO"
                fields = None
            else:
                subtitle = "중단됨"
                status = "INFO"
                fields = None

        self._post_card("공정 종료", subtitle, status, fields=fields, urgent=False)
        self.flush()

        # 종료 후 상태 리셋(다음 공정을 위해)
        self._last_error_src = None
        self._last_error_reason = None
        self._stop_pressed = False

    @Slot()
    def notify_stop_pressed(self):
        """
        사용자가 Stop 버튼을 눌렀음을 기록.
        연결하면 종료 시 '사용자 중지'로 표시됩니다.
        """
        self._stop_pressed = True

    @Slot(str)
    def notify_text(self, text: str):
        # 필요 시 텍스트만 보낼 수도 있음 (지연 모드면 버퍼에 쌓임)
        self._post_text(text)

    @Slot(str)
    def notify_error(self, reason: str):
        """
        장비/프로세스 오류를 기록하고(종료 카드에 반영),
        메시지는 지연 모드면 버퍼에 쌓아 둔다.
        """
        self._last_error_src = None
        self._last_error_reason = reason or "unknown error"
        self._post_card("장비 오류", self._last_error_reason, "FAIL", urgent=False)

    @Slot(str, str)
    def notify_error_with_src(self, src: str, reason: str):
        self._last_error_src = (src or "").strip() or None
        self._last_error_reason = reason or "unknown error"
        self._post_card("장비 오류", f"[{src}] {reason}", "FAIL", urgent=False)
