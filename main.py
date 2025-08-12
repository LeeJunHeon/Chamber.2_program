# main.py
import sys
import csv
from datetime import datetime
from PyQt6.QtWidgets import QApplication, QWidget, QMessageBox, QFileDialog
from PyQt6.QtCore import QTimer, QThread, pyqtSlot as Slot

# === controller import ===
from UI import Ui_Form
from controller.graph_controller import GraphController
from device.Faduino import FaduinoController
from device.IG import IGController
from device.RGA import RGAController
from device.MFC import MFCController
from device.OES import OESController
from device.DCpower import DCPowerController
from device.RFpower import RFPowerController
from controller.data_logger import DataLogger
from controller.process_controller import ProcessController


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self._set_default_ui_values()
        self.process_queue = []
        self.current_process_index = -1

        # === 1. 메인 스레드에서 동작하는 컨트롤러 생성 ===
        # UI를 직접 제어하거나, 자체적으로 긴 작업을 하지 않는 컨트롤러들입니다.
        self.graph_controller = GraphController(self.ui.RGA_Graph, self.ui.OES_Graph)

        # DC/RF Power 컨트롤러는 다른 컨트롤러(Faduino)를 통해 명령을 전달하므로
        # 자체적으로는 긴 작업을 하지 않아 메인 스레드에 둡니다.
        self.dc_power_controller = DCPowerController()
        self.rf_power_controller = RFPowerController()

        # === 2. 워커 스레드 및 '일꾼' 컨트롤러 생성 ===
        # 실제 하드웨어와 통신하며 시간이 걸리는 작업을 처리하는 컨트롤러들입니다.

        # Faduino 스레드 설정
        self.faduino_thread = QThread()
        self.faduino_controller = FaduinoController()
        self.faduino_controller.moveToThread(self.faduino_thread)

        # MFC 스레드 설정
        self.mfc_thread = QThread()
        self.mfc_controller = MFCController()
        self.mfc_controller.moveToThread(self.mfc_thread)

        # OES 스레드 설정
        self.oes_thread = QThread()
        self.oes_controller = OESController()
        self.oes_controller.moveToThread(self.oes_thread)
        
        # IG 스레드 설정
        self.ig_thread = QThread()
        self.ig_controller = IGController()
        self.ig_controller.moveToThread(self.ig_thread)

        # RGA 스레드 설정
        self.rga_thread = QThread()
        self.rga_controller = RGAController()
        self.rga_controller.moveToThread(self.rga_thread)

        # DataLogger 스레드 설정
        self.data_logger_thread = QThread()
        self.data_logger = DataLogger()
        self.data_logger.moveToThread(self.data_logger_thread)

        # === 3. 메인 스레드에서 동작하는 '감독관' 컨트롤러 생성 ===
        # 다른 컨트롤러들을 지휘하는 역할을 합니다.
        self.process_controller = ProcessController()

        # === 4. 신호-슬롯 연결 ===
        self._connect_signals()

        # === 5. 모든 워커 스레드 시작 ===
        self.faduino_thread.start()
        self.mfc_thread.start()
        self.oes_thread.start()
        self.ig_thread.start()
        self.rga_thread.start()
        self.data_logger_thread.start()

        # === 6. UI 업데이트 시그널 연결 ===
        self.mfc_controller.update_flow.connect(self.update_mfc_flow_ui)
        self.mfc_controller.update_pressure.connect(self.update_mfc_pressure_ui)

        # ▼▼▼ [추가] 공정이 끝나면 다음 공정을 시작하도록 신호 연결 ▼▼▼
        self.process_controller.process_finished.connect(self._start_next_process_from_queue)

    def _connect_signals(self):
        # ▼▼▼ [추가] 데이터 로깅을 위한 신호 연결 ▼▼▼
        # 1. 공정 시작/종료 신호를 DataLogger에 연결
        self.process_controller.process_started.connect(self.data_logger.start_new_log_session)
        self.process_controller.process_finished.connect(self.data_logger.finalize_and_write_log)

        # # 2. 각 장치 컨트롤러의 데이터 신호를 DataLogger의 슬롯에 연결
        self.ig_controller.pressure_update.connect(self.data_logger.log_ig_pressure)
        self.faduino_controller.dc_power_updated.connect(self.data_logger.log_dc_power)
        self.faduino_controller.rf_power_updated.connect(self.data_logger.log_rf_power)
        self.mfc_controller.update_flow.connect(self.data_logger.log_mfc_flow)
        self.mfc_controller.update_pressure.connect(self.data_logger.log_mfc_pressure)

        # # 로깅 연결
        self.faduino_controller.status_message.connect(self.append_log)
        self.mfc_controller.status_message.connect(self.append_log)
        self.oes_controller.status_message.connect(self.append_log)
        self.dc_power_controller.status_message.connect(self.append_log)
        self.rf_power_controller.status_message.connect(self.append_log)
        self.process_controller.log_message.connect(self.append_log)
        self.ig_controller.status_message.connect(self.append_log)
        self.rga_controller.status_message.connect(self.append_log)
        self.process_controller.update_process_state.connect(self.on_update_process_state)

        # # ProcessController -> 각 장치 (명령)
        self.process_controller.update_faduino_port.connect(self.faduino_controller.handle_named_command)
        self.process_controller.mfc_command_requested.connect(self.mfc_controller.handle_command)
        self.process_controller.oes_command_requested.connect(self.oes_controller.run_measurement)
        self.process_controller.dc_power_command_requested.connect(self.dc_power_controller.start_process)
        self.process_controller.dc_power_stop_requested.connect(self.dc_power_controller.stop_process)
        self.process_controller.rf_power_command_requested.connect(self.rf_power_controller.start_process) 
        self.process_controller.rf_power_stop_requested.connect(self.rf_power_controller.stop_process)
        self.process_controller.rga_external_scan_requested.connect(self.rga_controller.execute_external_scan)
        self.process_controller.ig_command_requested.connect(self.ig_controller.start_wait_for_pressure)

        # # 각 장치 -> ProcessController (완료 보고)
        self.mfc_controller.command_confirmed.connect(self.process_controller.on_step_completed)
        self.mfc_controller.command_failed.connect(self.process_controller.on_step_failed)
        self.oes_controller.oes_finished.connect(self.process_controller.on_step_completed)
        self.oes_controller.oes_failed.connect(self.process_controller.on_step_failed)
        self.dc_power_controller.target_reached.connect(self.process_controller.on_step_completed)
        self.dc_power_controller.target_failed.connect(self.process_controller.on_step_failed)
        self.dc_power_controller.power_off_finished.connect(self.process_controller.on_step_completed)
        self.rf_power_controller.target_failed.connect(self.process_controller.on_step_failed)
        self.rf_power_controller.power_off_finished.connect(self.process_controller.on_step_completed)
        self.rf_power_controller.target_reached.connect(self.process_controller.on_step_completed)
        self.faduino_controller.command_confirmed.connect(self.process_controller.on_step_completed)
        self.faduino_controller.command_failed.connect(self.process_controller.on_step_failed)

        self.rga_controller.scan_finished.connect(self.process_controller.on_step_completed)
        self.rga_controller.scan_failed.connect(self.process_controller.on_step_failed)
        self.ig_controller.base_pressure_reached.connect(self.process_controller.on_step_completed)
        self.ig_controller.base_pressure_failed.connect(self.process_controller.on_step_failed)

        # ProcessController -> 폴링 제어: set_polling 신호 하나로 모든 장비의 폴링 상태를 동기화
        self.process_controller.set_polling.connect(self.faduino_controller.set_process_status)
        self.process_controller.set_polling.connect(self.mfc_controller.set_process_status)

        # ProcessController -> 비상 종료: process_aborted 신호 하나로 모든 장비의 cleanup을 실행
        self.process_controller.process_aborted.connect(self.mfc_controller.cleanup)
        self.process_controller.process_aborted.connect(self.mfc_controller.abort_current_command)
        self.process_controller.process_aborted.connect(self.ig_controller.cleanup)
        self.process_controller.process_aborted.connect(self.faduino_controller.cleanup)
        self.process_controller.process_aborted.connect(self.oes_controller.cleanup)

        # (추가) 파워 컨트롤러의 상태 변경 신호를 Faduino의 새 슬롯에 연결
        self.rf_power_controller.state_changed.connect(self.faduino_controller.on_rf_state_changed)
        self.dc_power_controller.state_changed.connect(self.faduino_controller.on_dc_state_changed)

        # # DC Power 연결
        self.dc_power_controller.request_status_read.connect(self.faduino_controller.force_status_read)
        self.dc_power_controller.send_dc_power_value.connect(self.faduino_controller.set_dc_power)
        self.dc_power_controller.send_dc_power_value_unverified.connect(self.faduino_controller.set_dc_power_unverified)
        self.faduino_controller.dc_power_updated.connect(self.dc_power_controller.update_measurements)
        self.faduino_controller.dc_power_updated.connect(self.handle_dc_power_feedback)

        # # RF Power 연결
        self.rf_power_controller.request_status_read.connect(self.faduino_controller.force_status_read)
        self.rf_power_controller.send_rf_power_value.connect(self.faduino_controller.set_rf_power)
        self.rf_power_controller.send_rf_power_value_unverified.connect(self.faduino_controller.set_rf_power_unverified)
        self.faduino_controller.rf_power_updated.connect(self.rf_power_controller.update_measurements)
        self.rf_power_controller.update_rf_status_display.connect(self.handle_rf_power_display)

        # RGA, OES 그래프 연결
        self.rga_controller.rga_data_updated.connect(self.graph_controller.update_rga_plot)
        self.oes_controller.oes_data_updated.connect(self.graph_controller.update_oes_plot)
        
        # UI 버튼 연결
        self.ui.Start_button.clicked.connect(self.on_start_button_clicked)
        self.ui.Stop_button.clicked.connect(self.process_controller.abort_process)
        self.ui.Process_list_button.clicked.connect(self.on_process_list_button_clicked)

    # <--- 추가: Faduino의 DC 전압/전류 피드백을 처리하는 슬롯 ---
    @Slot(float, float)
    def handle_rf_power_display(self, for_p, ref_p):
        self.ui.For_p_edit.setPlainText(f"{for_p:.1f}")
        self.ui.Ref_p_edit.setPlainText(f"{ref_p:.1f}")

    @Slot(float, float, float)
    def handle_dc_power_feedback(self, power, voltage, current):
        """Faduino로부터 DC 전압/전류를 받아 Power 계산 후 UI와 DC 컨트롤러에 전달"""
        if voltage is None or current is None:
            self.append_log("Main", "voltage, current값이 비어있습니다.")
            return

        # 1. UI 업데이트
        self.ui.Power_edit.setPlainText(f"{power:.2f}")
        self.ui.Voltage_edit.setPlainText(f"{voltage:.2f}")
        self.ui.Current_edit.setPlainText(f"{current:.2f}")
        
        # 2. DCPowerController에 피드백 전달
        self.dc_power_controller.update_measurements(power, voltage, current)

    @Slot(str)
    def update_mfc_pressure_ui(self, pressure_value):
        """MFC로부터 받은 압력 값으로 UI 업데이트"""
        self.ui.Working_pressure_edit.setPlainText(pressure_value)

    @Slot(str, float)
    def update_mfc_flow_ui(self, gas_name, flow_value):
        """MFC로부터 받은 유량 값으로 UI 업데이트"""
        if gas_name == "Ar":
            self.ui.Ar_flow_edit.setPlainText(f"{flow_value:.1f}")
        elif gas_name == "O2":
            self.ui.O2_flow_edit.setPlainText(f"{flow_value:.1f}")
        elif gas_name == "N2":
            self.ui.N2_flow_edit.setPlainText(f"{flow_value:.1f}")

    @Slot()
    def on_process_list_button_clicked(self):
        """파일 선택 버튼을 눌렀을 때 CSV 파일을 열고 파싱 + UI 갱신"""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "프로세스 리스트 파일 선택",
            "", 
            "CSV Files (*.csv);;All Files (*)"
        )
        if not file_path:
            self.append_log("File", "파일 선택이 취소되었습니다.")
            return

        self.append_log("File", f"선택된 파일: {file_path}")
        try:
            with open(file_path, mode='r', encoding='utf-8-sig') as csvfile:
                reader = csv.DictReader(csvfile)

                # ▼▼▼ [수정] 공정 큐와 인덱스 초기화 ▼▼▼
                self.process_queue = [] 
                self.current_process_index = -1

                for row in reader:
                    # CSV의 '#' 컬럼을 'Process_name'으로 사용, 없으면 기본 이름 부여
                    row['Process_name'] = row.get('#', f'공정 {len(self.process_queue) + 1}')
                    self.process_queue.append(row)

                if not self.process_queue:
                    self.append_log("File", "파일에 처리할 공정이 없습니다.")
                    return

                self.append_log("File", f"총 {len(self.process_queue)}개의 공정을 파일에서 읽었습니다.")

                # 읽은 파일을 로그에 띄워주는 코드
                # for params in self.process_queue:
                #     self.append_log("File", f"  - {params['Process_name']}: {params}")

                # UI를 첫 번째 공정 값으로 즉시 갱신하여 보여줌
                self._update_ui_from_params(self.process_queue[0])

        except Exception as e:
            self.append_log("File", f"파일 처리 중 오류 발생: {e}")

    def _update_ui_from_params(self, params: dict):
        """파라미터 딕셔너리를 기반으로 UI의 모든 위젯 값을 업데이트합니다."""
        self.append_log("UI", f"다음 공정 '{params['Process_name']}'의 파라미터로 UI를 업데이트합니다.")

        self.ui.RF_power_edit.setPlainText(params.get('rf_power', '0'))
        self.ui.DC_power_edit.setPlainText(params.get('dc_power', '0'))
        self.ui.Process_time_edit.setPlainText(params.get('process_time', '0'))
        self.ui.Intergration_time_edit.setPlainText(params.get('integration_time', '60'))
        self.ui.Ar_flow_edit.setPlainText(params.get('Ar_flow', '0'))
        self.ui.O2_flow_edit.setPlainText(params.get('O2_flow', '0'))
        self.ui.N2_flow_edit.setPlainText(params.get('N2_flow', '0'))
        self.ui.Working_pressure_edit.setPlainText(params.get('working_pressure', '0'))
        self.ui.Base_pressure_edit.setPlainText(params.get('base_pressure', '0'))
        self.ui.Shutter_delay_edit.setPlainText(params.get('shutter_delay', '0'))

        self.ui.G1_checkbox.setChecked(params.get('gun1', 'F') == 'T')
        self.ui.G2_checkbox.setChecked(params.get('gun2', 'F') == 'T')
        self.ui.G3_checkbox.setChecked(params.get('gun3', 'F') == 'T')
        self.ui.Ar_checkbox.setChecked(params.get('Ar', 'F') == 'T')
        self.ui.O2_checkbox.setChecked(params.get('O2', 'F') == 'T')
        self.ui.N2_checkbox.setChecked(params.get('N2', 'F') == 'T')
        self.ui.Main_shuter_checkbox.setChecked(params.get('main_shutter', 'F') == 'T')
        self.ui.RF_power_checkbox.setChecked(params.get('use_rf_power', 'F') == 'T')
        self.ui.DC_power_checkbox.setChecked(params.get('use_dc_power', 'F') == 'T')

    @Slot(bool)
    def _start_next_process_from_queue(self, was_successful: bool):
        """공정 큐에서 다음 공정을 가져와 UI를 업데이트하고 시작합니다."""
        
        if not was_successful:
            self.append_log("MAIN", "이전 공정이 실패하여 자동 시퀀스를 중단합니다.")
            self.process_queue = []
            self.current_process_index = -1
            self._reset_ui_after_process()
            return
        
        self.current_process_index += 1

        if self.current_process_index < len(self.process_queue):
            # 다음 공정이 남아있을 경우
            params = self.process_queue[self.current_process_index]

            # ▼▼▼ [수정] 현재 공정의 controll_process 값으로 시작 방식을 결정 ▼▼▼
            # 'controll_process'가 'T' 또는 'True'이면 Cold Start, 아니면 Hot Start
            is_cold_start = str(params.get('controll_process', 'True')).upper() in ['TRUE', 'T']

            self._update_ui_from_params(params)
            
            # 잠시 후 (UI가 업데이트될 시간을 준 뒤) 다음 공정 시작
            QTimer.singleShot(100, lambda p=params, cs=is_cold_start: self.process_controller.start_process(p, cs))
        else:
            # 모든 공정이 끝났을 경우
            self.append_log("MAIN", "모든 공정이 완료되었습니다.")
            self.process_queue = []
            self.current_process_index = -1
            self._reset_ui_after_process()

    def _check_and_connect_devices(self):
        """모든 장비의 연결을 확인하고, 끊겨있으면 연결을 시도합니다. 하나라도 실패하면 False를 반환합니다."""
        # 1. Faduino
        if not self.faduino_controller.serial_faduino:
            if not self.faduino_controller.connect_faduino(): return False
        
        # 2. MFC
        if not self.mfc_controller.serial_mfc:
            if not self.mfc_controller.connect_mfc_device(): return False
            
        # 3. OES (main에서 쓰레드를 사용하니 보완필요)
        if self.oes_controller.sChannel < 0:
            if not self.oes_controller.initialize_device(): return False
            
        # 4. IG
        if not self.ig_controller.serial_ig:
            if not self.ig_controller.connect_device(): return False
        
        self.append_log("MAIN", "모든 장비가 성공적으로 연결되었습니다.")
        return True

    @Slot()
    def on_start_button_clicked(self):
        """
        Start 버튼 클릭 시:
        - 모든 장비 연결을 시도하고, 성공하면 공정을 시작합니다.
        - 로드된 공정 파일이 있으면 자동 순차 실행을 시작합니다.
        - 파일이 없으면 현재 UI의 입력값으로 단일 공정을 실행합니다.
        """

        if self.process_controller.is_running:
            QMessageBox.warning(self, "실행 오류", "현재 다른 공정이 실행 중입니다.")
            return
        
        if not self._check_and_connect_devices():
            QMessageBox.critical(self, "장비 연결 오류", "필수 장비 연결에 실패했습니다. 로그를 확인하세요.")
            return

        # --- [수정] 파일 로드 여부에 따라 분기 ---
        if self.process_queue:
            # 경우 1: 파일이 로드된 경우 -> 자동 순차 실행 시작
            self.append_log("MAIN", "입력받은 파일로 자동 공정 시퀀스를 시작합니다.")
            self.current_process_index = -1  # 인덱스를 초기화하고 시작
            self._start_next_process_from_queue(True)
        else:
            # 경우 2: 파일이 로드되지 않은 경우 -> UI 값으로 단일 공정 실행
            self.append_log("MAIN", "입력받은 UI값으로 단일 공정을 시작합니다.")
            
            # === 현재 UI에서 값을 읽어 파라미터 생성 (기존 코드와 동일) ===
            use_rf = self.ui.RF_power_checkbox.isChecked()
            use_dc = self.ui.DC_power_checkbox.isChecked()

            if not use_rf and not use_dc:
                QMessageBox.warning(self, "선택 오류", "RF 파워 또는 DC 파워 중 하나 이상을 반드시 선택해야 합니다.")
                return

            try:
                base_pressure = float(self.ui.Base_pressure_edit.toPlainText() or 1e-5)
                integration_time = int(self.ui.Intergration_time_edit.toPlainText() or 60)
                ar_flow = float(self.ui.Ar_flow_edit.toPlainText() or 0.0)
                o2_flow = float(self.ui.O2_flow_edit.toPlainText() or 0.0)
                n2_flow = float(self.ui.N2_flow_edit.toPlainText() or 0.0)
                working_pressure = float(self.ui.Working_pressure_edit.toPlainText() or 0.0)
                rf_power = float(self.ui.RF_power_edit.toPlainText() or 0.0)
                dc_power = float(self.ui.DC_power_edit.toPlainText() or 0.0)
                shutter_delay = float(self.ui.Shutter_delay_edit.toPlainText() or 0.0)
                process_time = float(self.ui.Process_time_edit.toPlainText() or 0.0) 

            except ValueError:
                self.append_log("UI", "오류: 값 입력란을 확인해주세요.")
                return

            # Gun 체크 유효성 검사
            gun_checkboxes = [self.ui.G1_checkbox, self.ui.G2_checkbox, self.ui.G3_checkbox]
            checked_count = sum(1 for cb in gun_checkboxes if cb.isChecked())
            if checked_count == 0 or checked_count == 3:
                msg_box = QMessageBox(self)
                msg_box.setIcon(QMessageBox.Icon.Warning)
                msg_box.setWindowTitle("선택 오류")
                msg_box.setText("Gun 선택 개수를 확인해주세요.")
                msg_box.setInformativeText("G1, G2, G3 중 1개 또는 2개만 선택해야 합니다.")
                msg_box.setStandardButtons(QMessageBox.StandardButton.Ok)
                msg_box.exec()
                return

            # 최종 params
            params = {
                "base_pressure": base_pressure,
                "integration_time": integration_time,
                "use_ms": self.ui.Main_shuter_checkbox.isChecked(),
                "use_g1": self.ui.G1_checkbox.isChecked(),
                "use_g2": self.ui.G2_checkbox.isChecked(),
                "use_g3": self.ui.G3_checkbox.isChecked(),
                "use_ar": self.ui.Ar_checkbox.isChecked(),
                "use_o2": self.ui.O2_checkbox.isChecked(),
                "use_n2": self.ui.N2_checkbox.isChecked(),
                "ar_flow": ar_flow,
                "o2_flow": o2_flow,
                "n2_flow": n2_flow,
                "working_pressure": working_pressure,
                "use_rf_power": use_rf,
                "use_dc_power": use_dc,
                "rf_power": rf_power,
                "dc_power": dc_power,
                "shutter_delay": shutter_delay,
                "process_time": process_time,
                "process_note": self.ui.note_edit.toPlainText(),
                "G1_target_name": self.ui.G1_target_name.toPlainText(),
                "G2_target_name": self.ui.G2_target_name.toPlainText(),
                "G3_target_name": self.ui.G3_target_name.toPlainText(),
            }

            # ProcessController로 파라미터를 전달하여 단일 공정 시작
            self.process_controller.start_process(params, is_cold_start=True)

    @Slot(str, str)
    def append_log(self, source: str, msg: str):
        """
        [수정됨] 로그 메시지를 UI와 log.txt 파일에 동시에 추가합니다.
        """
        # --- UI에 표시될 로그 메시지 (기존과 동일) ---
        ui_now = datetime.now().strftime("%H:%M:%S")
        ui_msg = f"[{ui_now}] [{source}] {msg}"
        self.ui.log_message.appendPlainText(ui_msg)

        # --- log.txt 파일에 저장될 로그 메시지 ---
        try:
            file_now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            file_msg = f"[{file_now}] [{source}] {msg}\n"
            
            with open("log.txt", "a", encoding="utf-8") as f:
                f.write(file_msg)
        except Exception as e:
            # 파일 쓰기 오류가 발생해도 프로그램이 멈추지 않도록 예외 처리
            error_msg = f"[{ui_now}] [Logger] 파일 로그 작성 실패: {e}"
            self.ui.log_message.appendPlainText(error_msg)

    @Slot(str)
    def on_update_process_state(self, message: str):
        """ProcessController로부터 현재 스텝 정보를 받아 UI에 표시합니다."""
        self.ui.process_state.setPlainText(message)

    def _set_default_ui_values(self):
        # --- 공정 조건 ---
        self.ui.Base_pressure_edit.setPlainText("1e-5") #
        self.ui.Intergration_time_edit.setPlainText("60") #
        self.ui.Working_pressure_edit.setPlainText("2") #
        self.ui.Process_time_edit.setPlainText("10") #
        self.ui.Shutter_delay_edit.setPlainText("5") #

        # --- 가스 및 건 ---
        self.ui.Ar_flow_edit.setPlainText("20") #
        self.ui.O2_flow_edit.setPlainText("0") #
        self.ui.N2_flow_edit.setPlainText("0") #

        # --- 파워 설정 ---
        self.ui.DC_power_edit.setPlainText("200") #
        self.ui.RF_power_edit.setPlainText("200") #

    def _reset_ui_after_process(self):
        """공정 종료(성공/실패) 후 UI를 초기 상태로 정리"""
        # 1) 입력값 기본치로 되돌리기
        self._set_default_ui_values()

        # 2) 체크박스 해제
        for cb in [
            self.ui.G1_checkbox, self.ui.G2_checkbox, self.ui.G3_checkbox,
            self.ui.Ar_checkbox, self.ui.O2_checkbox, self.ui.N2_checkbox,
            self.ui.Main_shuter_checkbox, self.ui.RF_power_checkbox, self.ui.DC_power_checkbox
        ]:
            cb.setChecked(False)

        # 3) 상태/표시값 초기화
        self.ui.process_state.setPlainText("대기 중")
        self.ui.Power_edit.setPlainText("0.00")
        self.ui.Voltage_edit.setPlainText("0.00")
        self.ui.Current_edit.setPlainText("0.00")
        self.ui.For_p_edit.setPlainText("0.0")
        self.ui.Ref_p_edit.setPlainText("0.0")
        # Working pressure/flow들은 위의 _set_default_ui_values()가 이미 채워줍니다.


    def closeEvent(self, event):
        """프로그램 종료 시 생성된 모든 워커 스레드를 안전하게 종료합니다."""
        print("프로그램 종료 절차 시작...")

        # 종료할 스레드 목록을 리스트로 관리하여 코드를 깔끔하게 유지
        threads = [
            getattr(self, 'faduino_thread', None),
            getattr(self, 'mfc_thread', None),
            getattr(self, 'oes_thread', None),
            getattr(self, 'ig_thread', None),
            getattr(self, 'rga_thread', None),
            getattr(self, 'data_logger_thread', None)
        ]

        for thread in threads:
            # 해당 스레드 객체가 존재하고, 현재 실행 중인지 확인
            if thread and thread.isRunning():
                thread.quit()   # 1. 스레드의 이벤트 루프에 종료 요청
                thread.wait()   # 2. 스레드가 완전히 종료될 때까지 대기
                print(f"{thread} 종료 완료.")

        print("모든 스레드 종료. 프로그램을 닫습니다.")
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
