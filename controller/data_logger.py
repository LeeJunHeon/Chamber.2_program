import csv
import os
from datetime import datetime
from pathlib import Path
from PyQt6.QtCore import QObject, pyqtSlot as Slot

class DataLogger(QObject):
    """
    각 장치로부터 데이터를 받아 리스트에 저장하고,
    공정이 끝나면 평균값을 계산하여 CSV 파일에 기록하는 클래스.
    """
    def __init__(self, parent=None):
        super().__init__(parent)

        # 로그 파일 경로
        log_directory = Path(r"\\VanaM_NAS\VanaM_Sputter\Sputter\Calib\Database")
        log_directory.mkdir(parents=True, exist_ok=True)

        self.log_file = log_directory / "Ch2_log.csv"

        # 수집 버퍼
        self.process_params = {}
        self.ig_pressure_readings = []

        self.dc_power_readings = []
        self.dc_voltage_readings = []
        self.dc_current_readings = []

        self.rf_for_p_readings = []
        self.rf_ref_p_readings = []

        # RF Pulse 폴링값 ★ 추가
        self.rf_pulse_for_p_readings = []
        self.rf_pulse_ref_p_readings = []

        self.mfc_flow_readings = {"Ar": [], "O2": [], "N2": []}
        self.mfc_pressure_readings = []
        
        # ✅ 최종 헤더(새 컬럼 3개 포함)
        self.header = [
            "Timestamp", "Process Note", "Base Pressure",
            "G1 Target", "G2 Target", "G3 Target",
            "Ar flow", "O2 flow", "N2 flow",
            "Working Pressure", "Process Time",
            "RF: For.P", "RF: Ref. P", 
            "DC: V", "DC: I", "DC: P",
            "RF Pulse: P", "RF Pulse: Freq", "RF Pulse: Duty Cycle",
            "RF Pulse: For.P", "RF Pulse: Ref.P"
        ]

        # ⬇︎ 기존 파일이 있으면 헤더를 새 형식으로 1회 업그레이드
        self._ensure_header()

    def _ensure_header(self):
        """기존 Ch2_log.csv가 구헤더면 새 헤더로 업그레이드(기존 행 보존, 새 컬럼 공란)."""
        if not self.log_file.exists():
            return  # 파일이 없으면 그대로 새 헤더로 처음부터 씀

        try:
            with open(self.log_file, 'r', encoding='utf-8-sig', newline='') as rf:
                reader = csv.reader(rf)
                existing_header = next(reader, [])
        except Exception:
            # 읽기 실패하면 건드리지 않음(이후 첫 기록 때 새 헤더로 새 파일처럼 동작)
            return

        # 이미 최신 헤더면 아무것도 안 함
        if existing_header == self.header:
            return

        # 기존 파일 전체를 읽어서 새 헤더로 재작성
        # (기존 헤더의 있는 필드만 복사, 새 필드는 공란)
        temp_path = self.log_file.with_suffix(".tmp")
        try:
            # 구헤더로 다시 읽기
            with open(self.log_file, 'r', encoding='utf-8-sig', newline='') as rf:
                old_reader = csv.DictReader(rf)
                rows = list(old_reader)  # 전부 메모리로

            # 새 헤더로 임시 파일 작성
            with open(temp_path, 'w', encoding='utf-8-sig', newline='') as wf:
                new_writer = csv.DictWriter(wf, fieldnames=self.header)
                new_writer.writeheader()
                for row in rows:
                    new_row = {h: row.get(h, "") for h in self.header}
                    new_writer.writerow(new_row)

            # 임시 파일을 원본으로 교체 (원자적 교체)
            os.replace(temp_path, self.log_file)
        except Exception:
            # 실패 시 임시파일 제거 시도 후 무시
            try:
                if temp_path.exists():
                    temp_path.unlink()
            except Exception:
                pass
            # 업그레이드 실패해도 이후 append 시도는 정상 진행(헤더 mismatch 시 첫 기록에서 헤더 작성)

    @Slot(dict)
    def start_new_log_session(self, params: dict):
        """새로운 공정이 시작될 때 호출되어 데이터 저장소를 초기화합니다."""
        self.process_params = params.copy()
        self.ig_pressure_readings.clear()

        self.dc_power_readings.clear()
        self.dc_voltage_readings.clear()
        self.dc_current_readings.clear()

        self.rf_for_p_readings.clear()
        self.rf_ref_p_readings.clear()

        # RF Pulse 폴링값 초기화 ★ 추가
        self.rf_pulse_for_p_readings.clear()
        self.rf_pulse_ref_p_readings.clear()

        for gas in self.mfc_flow_readings:
            self.mfc_flow_readings[gas].clear()
        self.mfc_pressure_readings.clear()

    # --- 각 장치로부터 데이터를 받는 슬롯들 ---
    @Slot(float)
    def log_ig_pressure(self, pressure):
        self.ig_pressure_readings.append(pressure)

    @Slot(float, float, float)
    def log_dc_power(self, power, voltage, current):
        self.dc_power_readings.append(power)
        self.dc_voltage_readings.append(voltage)
        self.dc_current_readings.append(current)

    @Slot(float, float)
    def log_rf_power(self, for_p, ref_p):
        self.rf_for_p_readings.append(for_p)
        self.rf_ref_p_readings.append(ref_p)

    # RF Pulse 폴링 수신 슬롯 ★ 추가
    @Slot(float, float)
    def log_rfpulse_power(self, for_p, ref_p):
        self.rf_pulse_for_p_readings.append(for_p)
        self.rf_pulse_ref_p_readings.append(ref_p)

    @Slot(str, float)
    def log_mfc_flow(self, gas_name, flow_value):
        if gas_name in self.mfc_flow_readings:
            self.mfc_flow_readings[gas_name].append(flow_value)

    @Slot(str)
    def log_mfc_pressure(self, pressure_str):
        try:
            # 압력 문자열 'V+100.00' 에서 숫자 부분만 파싱
            if '+' in pressure_str:
                pressure_val = float(pressure_str.split('+')[1])
                self.mfc_pressure_readings.append(pressure_val)
        except (ValueError, IndexError):
            pass

    @Slot(bool)
    def finalize_and_write_log(self, was_successful: bool):
        """공정이 끝나면 호출되어, 평균을 계산하고 파일에 한 줄을 기록합니다."""
        if not was_successful:
            # 실패한 공정의 데이터는 기록하지 않음
            return

        # --- 평균값 계산 ---
        def _avg(seq):
            return (sum(seq) / len(seq)) if seq else 0.0

        # IG는 첫 값(또는 params 기본값)
        base_pressure = self.ig_pressure_readings[0] if self.ig_pressure_readings else \
                        float(self.process_params.get("base_pressure", 0.0))

        # 사용 플래그
        use_rf  = bool(self.process_params.get("use_rf_power", False))
        use_dc  = bool(self.process_params.get("use_dc_power", False))
        use_rfp = bool(self.process_params.get("use_rf_pulse", False))

        # RF Pulse 설정값(이미 하던 방식 유지)
        rfp_p = self.process_params.get("rf_pulse_power", None)
        rfp_f = self.process_params.get("rf_pulse_freq", None)
        rfp_d = self.process_params.get("rf_pulse_duty", None)

        log_data = {
            "Timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "Process Note": self.process_params.get("process_note", ""),
            "Base Pressure": f"{base_pressure:.2e}",
            "G1 Target": self.process_params.get("G1 Target", ""),
            "G2 Target": self.process_params.get("G2 Target", ""),
            "G3 Target": self.process_params.get("G3 Target", ""),
            "Ar flow": f"{_avg(self.mfc_flow_readings['Ar']):.2f}",
            "O2 flow": f"{_avg(self.mfc_flow_readings['O2']):.2f}",
            "N2 flow": f"{_avg(self.mfc_flow_readings['N2']):.2f}",
            "Working Pressure": f"{_avg(self.mfc_pressure_readings):.4f}",
            "Process Time": self.process_params.get("process_time", 0.0),

            # ★ 일반 RF: 사용 시에만 평균 기록, 아니면 공란
            "RF: For.P": (f"{_avg(self.rf_for_p_readings):.2f}"
                        if use_rf and self.rf_for_p_readings else ""),
            "RF: Ref. P": (f"{_avg(self.rf_ref_p_readings):.2f}"
                        if use_rf and self.rf_ref_p_readings else ""),

            # ★ DC: 사용 시에만 평균 기록, 아니면 공란
            "DC: V": (f"{_avg(self.dc_voltage_readings):.2f}"
                    if use_dc and self.dc_voltage_readings else ""),
            "DC: I": (f"{_avg(self.dc_current_readings):.2f}"
                    if use_dc and self.dc_current_readings else ""),
            "DC: P": (f"{_avg(self.dc_power_readings):.2f}"
                    if use_dc and self.dc_power_readings else ""),

            # ✅ RF Pulse 설정값 3개: 사용 시에만 기록
            "RF Pulse: P":          (f"{float(rfp_p):.2f}" if use_rfp and rfp_p not in (None, "") else ""),
            "RF Pulse: Freq":       (str(int(rfp_f))        if use_rfp and rfp_f not in (None, "") else ""),
            "RF Pulse: Duty Cycle": (str(int(rfp_d))        if use_rfp and rfp_d not in (None, "") else ""),

            # ★ RF Pulse 폴링 평균값 2개: 사용 시에만 기록, 아니면 공란
            "RF Pulse: For.P": (f"{_avg(self.rf_pulse_for_p_readings):.2f}"
                                if use_rfp and self.rf_pulse_for_p_readings else ""),
            "RF Pulse: Ref.P": (f"{_avg(self.rf_pulse_ref_p_readings):.2f}"
                                if use_rfp and self.rf_pulse_ref_p_readings else ""),
        }

        # --- 파일에 기록 ---
        try:
            file_exists = self.log_file.exists()
            with open(self.log_file, 'a', newline='', encoding='utf-8-sig') as f:
                writer = csv.DictWriter(f, fieldnames=self.header)
                if not file_exists:
                    # 새 파일이면 새 헤더로
                    writer.writeheader()
                writer.writerow(log_data)
        except Exception as e:
            print(f"데이터 로그 파일 작성 실패: {e}")
