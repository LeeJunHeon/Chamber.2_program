import csv
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
        #self.log_file = Path("Ch2_log.csv")

        # Path 객체를 사용하여 네트워크 경로를 지정합니다.
        log_directory = Path(r"\\VanaM_NAS\VanaM_Sputter\Sputter\Calib\Database")
        
        # 지정된 디렉토리가 존재하지 않으면 생성합니다.
        # parents=True: 중간 경로가 없어도 모두 생성
        # exist_ok=True: 폴더가 이미 있어도 에러를 발생시키지 않음
        log_directory.mkdir(parents=True, exist_ok=True)
        
        self.log_file = log_directory / "Ch2_log.csv"

        self.process_params = {}
        self.ig_pressure_readings = []

        self.dc_power_readings = []
        self.dc_voltage_readings = []
        self.dc_current_readings = []

        self.rf_for_p_readings = []
        self.rf_ref_p_readings = []

        self.mfc_flow_readings = {"Ar": [], "O2": [], "N2": []}
        self.mfc_pressure_readings = []
        
        # CSV 파일 헤더 정의
        self.header = [
            "Timestamp", "Process Note", "Base Pressure",
            "G1 Target", "G2 Target", "G3 Target",
            "Ar flow", "O2 flow", "N2 flow",
            "Working Pressure", "Process Time",
            "RF: For.P", "RF: Ref. P", 
            "DC: V", "DC: I", "DC: P",
        ]

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
        def _calculate_avg(data_list):
            return sum(data_list) / len(data_list) if data_list else 0.0

        # IG는 마지막 값 또는 첫 값을 사용 (Base Pressure는 한 번만 측정됨)
        base_pressure = self.ig_pressure_readings[0] if self.ig_pressure_readings else self.process_params.get("base_pressure", 0.0)

        log_data = {
            "Timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "Process Note": self.process_params.get("process_note", ""),
            "Base Pressure": f"{base_pressure:.2e}",
            "G1 Target": self.process_params.get("G1_target_name", ""),
            "G2 Target": self.process_params.get("G2_target_name", ""),
            "G3 Target": self.process_params.get("G3_target_name", ""),
            "Ar flow": f"{_calculate_avg(self.mfc_flow_readings['Ar']):.2f}",
            "O2 flow": f"{_calculate_avg(self.mfc_flow_readings['O2']):.2f}",
            "N2 flow": f"{_calculate_avg(self.mfc_flow_readings['N2']):.2f}",
            "Working Pressure": f"{_calculate_avg(self.mfc_pressure_readings):.4f}",
            "Process Time": self.process_params.get("process_time", 0.0),
            "RF: For.P": f"{_calculate_avg(self.rf_for_p_readings):.2f}",
            "RF: Ref. P": f"{_calculate_avg(self.rf_ref_p_readings):.2f}",
            "DC: V": f"{_calculate_avg(self.dc_voltage_readings):.2f}",
            "DC: I": f"{_calculate_avg(self.dc_current_readings):.2f}",
            "DC: P": f"{_calculate_avg(self.dc_power_readings):.2f}",
        }

        # --- 파일에 기록 ---
        try:
            file_exists = self.log_file.exists()
            with open(self.log_file, 'a', newline='', encoding='utf-8-sig') as f:
                writer = csv.DictWriter(f, fieldnames=self.header)
                if not file_exists:
                    writer.writeheader() # 파일이 없으면 헤더를 먼저 씀
                writer.writerow(log_data)
        except Exception as e:
            print(f"데이터 로그 파일 작성 실패: {e}")