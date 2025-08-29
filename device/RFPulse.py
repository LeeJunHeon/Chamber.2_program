# device/RFPulse.py
from __future__ import annotations

from typing import Optional, Tuple
from PyQt6.QtCore import QObject, QCoreApplication, QElapsedTimer, pyqtSignal as Signal, pyqtSlot as Slot
from PyQt6.QtSerialPort import QSerialPort, QSerialPort as QSP
from lib.config import RFPULSE_PORT, RFPULSE_BAUD, RFPULSE_ADDR, RFPULSE_DEFAULT_DELAY_MS

# ===== AE Bus command numbers (기존 테스트 코드와 동일) =====
CMD_RF_OFF              = 1
CMD_RF_ON               = 2
CMD_SET_CTRL_MODE       = 3      # data: 6=FWD, 7=LOAD, 8=EXT (1 byte)
CMD_SET_SETPOINT        = 8      # data: 16-bit Watts, LSB first
CMD_SET_ACTIVE_CTRL     = 14     # data: 0x02 -> Host

# Reads (report)
CMD_REPORT_STATUS        = 162
CMD_REPORT_SETPOINT_MODE = 164   # 2B setpoint (W, LSB first) + 1B mode
CMD_REPORT_FORWARD       = 165   # 2B W (LSB first)
CMD_REPORT_REFLECTED     = 166   # 2B W
CMD_REPORT_DELIVERED     = 167   # 2B W

# Pulsing
CMD_SET_PULSING         = 27     # 0=off, 1=int, 2=ext, 3=ext_inv, 4=int_by_ext
CMD_SET_PULSE_FREQ      = 93     # 3 bytes (Hz, LSB first)
CMD_SET_PULSE_DUTY      = 96     # 2 bytes (percent, LSB first)

CSR_CODES = {
    0: "OK",
    1: "Command Not Recognized",
    2: "Not in Host Mode",
    3: "Not Implemented",
    4: "Bad Data Value",
    5: "Busy",
}

MODE_SET  = {"fwd": 6, "load": 7, "ext": 8}
MODE_NAME = {46: "FWD", 47: "LOAD", 48: "EXT", 6: "FWD", 7: "LOAD", 8: "EXT"}

def _u16le(buf: bytes, i: int = 0) -> int:
    return buf[i] | (buf[i+1] << 8)

def _u24le(buf: bytes, i: int = 0) -> int:
    return buf[i] | (buf[i+1] << 8) | (buf[i+2] << 16)

def _build_packet(addr: int, cmd: int, data: bytes=b"") -> bytes:
    if not (0 <= addr <= 31):
        raise ValueError("addr 0..31")
    L = len(data)
    if L <= 6:
        header = ((addr & 0x1F) << 3) | L
        body = bytes([header, cmd]) + data
    else:
        header = ((addr & 0x1F) << 3) | 0x07
        body = bytes([header, cmd, L & 0xFF]) + data
    cs = 0
    for b in body: cs ^= b
    return body + bytes([cs & 0xFF])

# ====== (추가) 간단 리드백 헬퍼들 ======
def _bit(v: int, n: int) -> int:
    return (v >> n) & 1

class RFPulseController(QObject):
    """CESAR AE RS-232(RS-232C) Pulse 제어 (QSerialPort 사용, 워커 스레드에서 동작)"""

    # 로그 / 상태
    status_message = Signal(str, str)            # (src, msg)
    # 공정 컨트롤러 연동
    target_reached = Signal()                    # RF ON 완료 (정상)
    target_failed  = Signal(str)                 # 실패 사유
    power_off_finished = Signal()                # RF OFF 완료

    def __init__(self, parent=None):
        super().__init__(parent)
        self.port: QSerialPort = QSerialPort(self)
        self.addr: int = RFPULSE_ADDR
        self.default_delay_ms = RFPULSE_DEFAULT_DELAY_MS
        self._configured_port: Optional[str] = None
        self._default_port = RFPULSE_PORT
        self._default_baud = RFPULSE_BAUD

    # ---------- 포트 ----------
    # RFPulseController 내부에 추가
    @Slot()
    def connect_rfpulse_device(self) -> bool:
        """
        기본 설정(RFPULSE_ADDR/RFPULSE_PORT/RFPULSE_BAUD)으로 포트 연결.
        main에서 Start 전에 호출해서 미리 연결해두는 용도.
        """
        try:
            # 주소는 항상 config 값으로 동기화
            self.set_address(RFPULSE_ADDR)
        except Exception:
            pass

        ok = self.open_port(self._default_port, self._default_baud)
        if ok:
            self.status_message.emit("RFPulse", f"연결 성공: {self._default_port}@{self._default_baud}")
        else:
            self.status_message.emit("RFPulse", f"연결 실패: {self._default_port}@{self._default_baud}")
        return ok

    @Slot(int)
    def set_address(self, addr: int):
        self.addr = int(addr) & 0x1F

    @Slot(str, int)
    def open_port(self, port_name: str, baud: int = 9600) -> bool:
        """지연 연결 패턴: 이미 열려있다면 같은 포트면 패스, 다르면 교체."""
        if self.port.isOpen() and self._configured_port == port_name:
            return True

        if self.port.isOpen():
            try: self.port.close()
            except: pass

        self.port.setPortName(port_name)
        self.port.setBaudRate(baud)
        self.port.setDataBits(QSP.DataBits.Data8)
        self.port.setParity(QSP.Parity.OddParity)          # 8O1
        self.port.setStopBits(QSP.StopBits.OneStop)
        self.port.setFlowControl(QSP.FlowControl.NoFlowControl)

        ok = self.port.open(QSP.OpenModeFlag.ReadWrite)
        if not ok:
            self.status_message.emit("RFPulse", f"포트 열기 실패: {port_name}")
            return False

        # DTR 필요(장비에 따라 필수)
        try:
            self.port.setDataTerminalReady(True)
        except Exception:
            pass

        self._configured_port = port_name
        self.status_message.emit("RFPulse", f"포트 연결 완료: {port_name}")
        return True

    @Slot()
    def close_port(self):
        if self.port.isOpen():
            try:
                self.port.close()
                self.status_message.emit("RFPulse", "포트 닫힘")
            except Exception:
                pass

    # ---------- 저수준 통신 ----------
    def _delay_ms(self, ms: int):
        t = QElapsedTimer(); t.start()
        while t.elapsed() < ms:
            # 워커 스레드에서도 이벤트를 소화해 타임아웃 폴링을 부드럽게 함
            QCoreApplication.processEvents()

    def _read_bytes(self, need: int, deadline_ms: int) -> Optional[bytes]:
        buf = bytearray()
        t = QElapsedTimer(); t.start()
        while t.elapsed() < deadline_ms:
            if self.port.waitForReadyRead(max(1, deadline_ms - t.elapsed())):
                chunk: bytes = self.port.read(need - len(buf))
                if chunk:
                    buf.extend(chunk)
                    if len(buf) >= need:
                        return bytes(buf)
        return None

    def _recv_frame(self, timeout_ms: int = 800, allow_ack_only: bool = False):
        """ACK(0x06) 스킵, NAK(0x15) 에러, 유효 프레임 리턴"""
        seen_ack = False
        t = QElapsedTimer(); t.start()
        while t.elapsed() < timeout_ms:
            if not self.port.waitForReadyRead(max(1, timeout_ms - t.elapsed())):
                continue
            b = bytes(self.port.read(1))
            if not b:
                continue
            if b == b"\x15":
                self.status_message.emit("RFPulse", "RX: NAK")
                raise RuntimeError("NAK(0x15) received")
            if b == b"\x06":
                self.status_message.emit("RFPulse", "RX: ACK")
                seen_ack = True
                continue

            hdr = b[0]
            length_bits = hdr & 0x07

            cmd_b = self._read_bytes(1, timeout_ms - t.elapsed())
            if not cmd_b: 
                continue

            if length_bits == 7:
                opt_len = self._read_bytes(1, timeout_ms - t.elapsed())
                if not opt_len: 
                    continue
                data_len = opt_len[0]
                data = self._read_bytes(data_len, timeout_ms - t.elapsed())
                if data is None or len(data) != data_len:
                    continue
                cs = self._read_bytes(1, timeout_ms - t.elapsed())
                if not cs: 
                    continue
                pkt = bytes([hdr]) + cmd_b + opt_len + data + cs
            else:
                data_len = length_bits
                data = self._read_bytes(data_len, timeout_ms - t.elapsed())
                if data is None or len(data) != data_len:
                    continue
                cs = self._read_bytes(1, timeout_ms - t.elapsed())
                if not cs: 
                    continue
                pkt = bytes([hdr]) + cmd_b + data + cs

            # checksum
            calc = 0
            for x in pkt[:-1]: calc ^= x
            if (calc ^ pkt[-1]) != 0:
                continue
            return ("FRAME", pkt)

        if allow_ack_only and seen_ack:
            return ("ACK_ONLY", None)
        self.status_message.emit("RFPulse", "RX: timeout waiting frame")
        raise TimeoutError("No response header")

    def _send_exec(self, cmd: int, data: bytes=b"", timeout_ms: int=800) -> Tuple[str, int]:
        self.port.clear(QSP.Direction.AllDirections)
        pkt = _build_packet(self.addr, cmd, data)
        self.status_message.emit("RFPulse",
            f"TX exec: addr={self.addr} cmd=0x{cmd:02X} data={data.hex(' ')} pkt={pkt.hex(' ')}")
        self.port.write(pkt)
        self.port.waitForBytesWritten(timeout_ms)
        kind, payload = self._recv_frame(timeout_ms=timeout_ms, allow_ack_only=True)
        if kind == "ACK_ONLY":
            self.status_message.emit("RFPulse", f"RX exec: ACK_ONLY (cmd=0x{cmd:02X})")
            return ("ACK_ONLY", 0)

        p = payload
        hdr = p[0]
        rx_cmd = p[1]
        length_bits = hdr & 0x07
        idx = 2
        if length_bits == 7:
            dlen = p[idx]; idx += 1
        else:
            dlen = length_bits
        data_bytes = p[idx:idx+dlen]
        csr = data_bytes[0] if dlen >= 1 else 0
        self.status_message.emit("RFPulse",
            f"RX exec FRAME: hdr=0x{hdr:02X} cmd=0x{rx_cmd:02X} dlen={dlen} "
            f"data={data_bytes.hex(' ')} csr={csr}({CSR_CODES.get(csr,'?')}) pkt={p.hex(' ')}")
        return ("FRAME", csr)

    def _send_query(self, cmd: int, data: bytes=b"", timeout_ms: int=800) -> bytes:
        self.port.clear(QSP.Direction.AllDirections)
        pkt = _build_packet(self.addr, cmd, data)
        self.status_message.emit("RFPulse",
            f"TX query: addr={self.addr} cmd=0x{cmd:02X} data={data.hex(' ')} pkt={pkt.hex(' ')}")
        self.port.write(pkt)
        self.port.waitForBytesWritten(timeout_ms)
        kind, payload = self._recv_frame(timeout_ms=timeout_ms, allow_ack_only=False)
        if kind != "FRAME":
            raise TimeoutError("No data frame")
        p = payload
        hdr = p[0]
        rx_cmd = p[1]
        length_bits = hdr & 0x07
        idx = 2
        if length_bits == 7:
            dlen = p[idx]; idx += 1
        else:
            dlen = length_bits
        data_bytes = p[idx:idx+dlen]
        self.status_message.emit("RFPulse",
            f"RX query FRAME: hdr=0x{hdr:02X} cmd=0x{rx_cmd:02X} dlen={dlen} "
            f"data={data_bytes.hex(' ')} pkt={p.hex(' ')}")
        return data_bytes

    # ---------- 고수준 동작 ----------
    def _host_mode(self):
        k, csr = self._send_exec(CMD_SET_ACTIVE_CTRL, b"\x02", 800)
        if k != "ACK_ONLY" and csr != 0:
            raise RuntimeError(f"HOST 실패 CSR={csr}({CSR_CODES.get(csr,'?')})")
        self.status_message.emit("RFPulse", "[HOST] OK")
        self._delay_ms(self.default_delay_ms)

    def _set_mode(self, mode: str = "fwd"):
        m = MODE_SET.get(mode.lower())
        if m is None:
            raise ValueError("mode should be fwd/load/ext")
        k, csr = self._send_exec(CMD_SET_CTRL_MODE, bytes([m]), 800)
        if k != "ACK_ONLY" and csr != 0:
            raise RuntimeError(f"MODE {mode} 실패 CSR={csr}({CSR_CODES.get(csr,'?')})")
        self.status_message.emit("RFPulse", f"[MODE {mode.upper()}] OK")
        self._delay_ms(self.default_delay_ms)

    def _set_power(self, watts: int):
        if watts < 0: watts = 0
        if watts > 65535: watts = 65535
        data = bytes([watts & 0xFF, (watts >> 8) & 0xFF])
        k, csr = self._send_exec(CMD_SET_SETPOINT, data, 800)
        if k != "ACK_ONLY" and csr != 0:
            raise RuntimeError(f"SETP {watts}W 실패 CSR={csr}({CSR_CODES.get(csr,'?')})")
        self.status_message.emit("RFPulse", f"[SETP {watts}W] OK")
        self._delay_ms(self.default_delay_ms)

    def _set_pulsing(self, mode_code: int):
        k, csr = self._send_exec(CMD_SET_PULSING, bytes([mode_code & 0xFF]), 800)
        if k != "ACK_ONLY" and csr != 0:
            raise RuntimeError(f"PULSING({mode_code}) 실패 CSR={csr}({CSR_CODES.get(csr,'?')})")
        self.status_message.emit("RFPulse", f"[PULSING {mode_code}] OK")
        self._delay_ms(self.default_delay_ms)

    def _set_pulse_freq(self, hz: int):
        if hz < 1 or hz > 100000:
            raise ValueError("freq 1..100000Hz")
        data = bytes([hz & 0xFF, (hz >> 8) & 0xFF, (hz >> 16) & 0xFF])
        k, csr = self._send_exec(CMD_SET_PULSE_FREQ, data, 800)
        if k != "ACK_ONLY" and csr != 0:
            raise RuntimeError(f"PULSE FREQ {hz} 실패 CSR={csr}")
        self.status_message.emit("RFPulse", f"[PULSE FREQ {hz}] OK")
        self._delay_ms(self.default_delay_ms)

    def _set_pulse_duty(self, duty_percent: int):
        if duty_percent < 1 or duty_percent > 99:
            raise ValueError("duty 1..99%")
        v = int(duty_percent) & 0xFFFF
        data = bytes([v & 0xFF, (v >> 8) & 0xFF])
        k, csr = self._send_exec(CMD_SET_PULSE_DUTY, data, 800)
        if k != "ACK_ONLY" and csr != 0:
            raise RuntimeError(f"PULSE DUTY {duty_percent}% 실패 CSR={csr}")
        self.status_message.emit("RFPulse", f"[PULSE DUTY {duty_percent}%] OK")
        self._delay_ms(self.default_delay_ms)

    def _rf_on(self):
        k, csr = self._send_exec(CMD_RF_ON, b"", 2500)
        if k != "ACK_ONLY" and csr != 0:
            raise RuntimeError(f"RF ON 실패 CSR={csr}({CSR_CODES.get(csr,'?')})")
        self.status_message.emit("RFPulse", "[RF ON] OK")

    def _rf_off(self):
        try:
            k, csr = self._send_exec(CMD_RF_OFF, b"", 2500)
            if k != "ACK_ONLY" and csr != 0:
                self.status_message.emit("RFPulse", f"[RF OFF] CSR={csr}({CSR_CODES.get(csr,'?')})")
            else:
                self.status_message.emit("RFPulse", "[RF OFF] OK")
        except Exception as e:
            self.status_message.emit("RFPulse", f"[RF OFF] 예외: {e}")

    # ---------- (추가) 리드백 유틸 ----------
    def _report_process_status(self) -> tuple[int, int, bytes]:
        """cmd 162: status bytes 리드백 (최소 2바이트: status byte1, status byte2)"""
        data = self._send_query(CMD_REPORT_STATUS, b"", 1000)
        if len(data) < 2:
            raise RuntimeError("report status payload too short")
        return data[0], data[1], data  # (status1, status2, raw)

    def _read_setpoint_mode(self) -> tuple[int, int]:
        """cmd 164: (u16 setpoint W, u8 mode)"""
        data = self._send_query(CMD_REPORT_SETPOINT_MODE, b"", 800)
        sp = _u16le(data, 0) if len(data) >= 2 else 0
        md = data[2] if len(data) >= 3 else 0
        return sp, md

    def _read_forward_power(self) -> int:
        """cmd 165: u16 forward power (W)"""
        data = self._send_query(CMD_REPORT_FORWARD, b"", 800)
        return _u16le(data, 0) if len(data) >= 2 else 0
    
    # ---------- (추가) RF ON 후 검증 루틴 ----------
    def _wait_until_power_on_and_in_tol(
        self,
        target_w: int,
        timeout_ms: int = 6000,
        poll_ms: int = 150,
        require_fw_within_pct: float = 0.15  # forward ≥ (1 - 15%) * target
    ):
        """
        - status(162)로 Output On(bit5=1) && Setpoint within tolerance(bit7=0) 확인
        - 옵션: forward power가 target의 (1-15%) 이상인지 확인
        """
        t = QElapsedTimer(); t.start()
        stable_hits = 0
        need_hits = 2  # 2~3회 연속 확인 권장

        while t.elapsed() < timeout_ms:
            try:
                s1, _s2, _raw = self._report_process_status()
                output_on = bool(_bit(s1, 5))
                within_tol = not bool(_bit(s1, 7))  # 0=within tolerance
                if output_on and within_tol:
                    fw = self._read_forward_power()
                    if target_w > 0:
                        if fw >= int(target_w * (1.0 - require_fw_within_pct)):
                            stable_hits += 1
                        else:
                            stable_hits = 0
                    else:
                        stable_hits += 1

                    if stable_hits >= need_hits:
                        self.status_message.emit(
                            "RFPulse",
                            f"[VERIFY] OK: output_on={output_on}, within_tol={within_tol}, fwd={fw}W"
                        )
                        return
                else:
                    stable_hits = 0
            except Exception:
                stable_hits = 0

            self._delay_ms(max(1, poll_ms))

        # 타임아웃
        fw = 0
        try:
            fw = self._read_forward_power()
        except Exception:
            pass
        raise TimeoutError(f"RF ON 검증 실패(시간초과). forward={fw}W, target={target_w}W")

    # ---------- 외부에서 호출 ----------
    @Slot(float, object, object)
    def start_pulse_process(
        self,
        target_w: float,
        freq_hz: Optional[int] = None,
        duty_percent: Optional[int] = None,
    ):
        """
        순서: HOST(14) → FWD(3) → SETP(8) → (옵션: 93/96) → RF ON(2) → (검증: 162/165)
        - freq/duty가 None이면 장비 기존값 유지
        """
        try:
            # 시작 시점에 연결 보장 (중앙집중화)
            if not self.is_connected():
                if not self.connect_rfpulse_device():
                    raise RuntimeError(f"포트 '{self._default_port}' 연결 실패")

            # 1) HOST 모드
            self._host_mode()

            # 2) 규제 모드 FWD
            self._set_mode("fwd")  # 6 = Forward power regulation

            # 3) 파워 세트포인트
            sp = int(round(float(target_w)))
            self._set_power(sp)

            # 4) (옵션) pulsing freq/duty 반영
            if freq_hz is not None:
                self._set_pulse_freq(int(freq_hz))
            if duty_percent is not None:
                self._set_pulse_duty(int(duty_percent))

            # 5) RF ON
            self._rf_on()

            # 6) (검증) 실제 출력/허용오차 이내 도달 확인
            self._wait_until_power_on_and_in_tol(target_w=sp)

            # 완료 알림
            self.target_reached.emit()

        except Exception as e:
            # 실패 시 즉시 OFF
            try:
                self._rf_off()
            finally:
                self.power_off_finished.emit()
            self.target_failed.emit(str(e))

    def is_connected(self) -> bool:
        return bool(self.port and self.port.isOpen())

    @Slot()
    def stop_process(self, also_turn_pulsing_off: bool = True):
        """STOP/에러 시 즉시 RF OFF (필요 시 pulsing도 Off)"""
        try:
            if also_turn_pulsing_off:
                try:
                    self._set_pulsing(0)  # off
                except Exception:
                    pass
            self._rf_off()
        finally:
            self.power_off_finished.emit()

    @Slot()
    def cleanup(self):
        """워커 스레드에서 안전 종료: stop → close"""
        try:
            self.stop_process()
        except Exception:
            pass
        try:
            self.close_port()
        except Exception:
            pass
