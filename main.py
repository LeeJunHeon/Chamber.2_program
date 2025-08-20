# main.py
import sys
import csv
import atexit, signal
from datetime import datetime
from PyQt6.QtWidgets import QApplication, QWidget, QMessageBox, QFileDialog
from PyQt6.QtCore import QCoreApplication, QTimer, QThread, pyqtSlot as Slot

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

        # ▼▼▼ [추가] 공정이 끝나면 다음 공정을 시작하도록 신호 연결 ▼▼▼
        self.process_controller.process_finished.connect(self._start_next_process_from_queue)

        # === 프로그램이 비정상적으로 종료되었을때의 시그널 ===
        self._about_quit_called = False
        self._emergency_done = False

        app = QCoreApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self._on_about_to_quit)

        # atexit: 인터프리터 정상 종료 시 마지막으로 한 번 더 정리
        atexit.register(self._emergency_cleanup)

        # 콘솔 종료 시그널 핸들러(가능한 플랫폼에서만 작동; 실패/미지원이어도 무해)
        def _sig_handler(*_):
            # 여러 번 들어와도 1회만 실행
            self._emergency_cleanup()
            # 여기서 sys.exit를 추가로 부를 필요는 없음(상황에 따라 UI 앱 종료 흐름과 충돌 가능)
        for sig in (getattr(signal, "SIGINT", None),   # Ctrl+C
                    getattr(signal, "SIGTERM", None),  # 일반 종료 신호
                    getattr(signal, "SIGBREAK", None)  # Windows Ctrl+Break
                    ):
            if sig is not None:
                try:
                    signal.signal(sig, _sig_handler)
                except Exception:
                    pass

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
        #self.process_controller.process_aborted.connect(self.mfc_controller.abort_current_command)
        self.process_controller.process_aborted.connect(self.ig_controller.cleanup)
        self.process_controller.process_aborted.connect(self.faduino_controller.cleanup)
        self.process_controller.process_aborted.connect(self.oes_controller.cleanup)

        # (추가) 파워 컨트롤러의 상태 변경 신호를 Faduino의 새 슬롯에 연결
        self.rf_power_controller.state_changed.connect(self.faduino_controller.on_rf_state_changed)
        self.dc_power_controller.state_changed.connect(self.faduino_controller.on_dc_state_changed)

        # # DC Power 연결
        self.dc_power_controller.request_status_read.connect(self.faduino_controller.force_dc_read)
        self.dc_power_controller.send_dc_power_value.connect(self.faduino_controller.set_dc_power)
        self.dc_power_controller.send_dc_power_value_unverified.connect(self.faduino_controller.set_dc_power_unverified)
        self.faduino_controller.dc_power_updated.connect(self.dc_power_controller.update_measurements)
        self.dc_power_controller.update_dc_status_display.connect(self.handle_dc_power_display)

        # # RF Power 연결
        self.rf_power_controller.request_status_read.connect(self.faduino_controller.force_rf_read)
        self.rf_power_controller.send_rf_power_value.connect(self.faduino_controller.set_rf_power)
        self.rf_power_controller.send_rf_power_value_unverified.connect(self.faduino_controller.set_rf_power_unverified)
        self.faduino_controller.rf_power_updated.connect(self.rf_power_controller.update_measurements)
        self.rf_power_controller.update_rf_status_display.connect(self.handle_rf_power_display)

        # RGA, OES 그래프 연결
        self.rga_controller.rga_data_updated.connect(self.graph_controller.update_rga_plot)
        self.oes_controller.oes_data_updated.connect(self.graph_controller.update_oes_plot)

        # MFC UI 업데이트 연결
        self.mfc_controller.update_flow.connect(self.update_mfc_flow_ui)
        self.mfc_controller.update_pressure.connect(self.update_mfc_pressure_ui)
        
        # UI 버튼 연결
        self.ui.Start_button.clicked.connect(self.on_start_button_clicked)
        self.ui.Stop_button.clicked.connect(self.process_controller.abort_process)
        self.ui.Process_list_button.clicked.connect(self.on_process_list_button_clicked)
        self.process_controller.process_status_changed.connect(self._on_process_status_changed)
        # 초기 UI Stop button 비활성화
        self._on_process_status_changed(False)

    # <--- 추가: Faduino의 DC 전압/전류 피드백을 처리하는 슬롯 ---
    @Slot(float, float)
    def handle_rf_power_display(self, for_p, ref_p):
        if for_p is None or ref_p is None:
            self.append_log("Main", "for.p, ref.p 값이 비어있습니다.")
            return

        self.ui.For_p_edit.setPlainText(f"{for_p:.2f}")
        self.ui.Ref_p_edit.setPlainText(f"{ref_p:.2f}")

    @Slot(float, float, float)
    def handle_dc_power_display(self, power, voltage, current):
        if power is None or voltage is None or current is None:
            self.append_log("Main", "power, voltage, current값이 비어있습니다.")
            return

        # UI 업데이트만 수행
        self.ui.Power_edit.setPlainText(f"{power:.3f}")
        self.ui.Voltage_edit.setPlainText(f"{voltage:.3f}")
        self.ui.Current_edit.setPlainText(f"{current:.3f}")

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

    @Slot(bool)
    def _on_process_status_changed(self, running: bool):
        # 실행/종료 ‘상태’에 따라 버튼 토글
        self.ui.Start_button.setEnabled(not running)
        self.ui.Stop_button.setEnabled(running)

    @Slot()
    @Slot(bool)
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
        self.ui.Main_shutter_checkbox.setChecked(params.get('main_shutter', 'F') == 'T')
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

            self._update_ui_from_params(params)
            
            # 잠시 후 (UI가 업데이트될 시간을 준 뒤) 다음 공정 시작
            QTimer.singleShot(100, lambda p=params: self.process_controller.start_process(p))
        else:
            # 모든 공정이 끝났을 경우
            self.append_log("MAIN", "모든 공정이 완료되었습니다.")
            self.process_queue = []
            self.current_process_index = -1
            self._reset_ui_after_process()

    def _check_and_connect_devices(self):
        """모든 장비의 연결을 확인하고, 끊겨있으면 연결을 시도합니다. 하나라도 실패하면 False를 반환합니다."""

        # Faduino (QSerialPort + 워치독: 즉시 실패해도 백그라운드 재시도)
        if not getattr(self.faduino_controller, "serial_faduino", None) \
        or not self.faduino_controller.serial_faduino.isOpen():
            self.faduino_controller.connect_faduino()  # 성공/실패는 Faduino 워치독이 재시도
        
        # MFC (QSerialPort + 워치독: 즉시 실패해도 백그라운드 재시도)
        if not getattr(self.mfc_controller, "serial_mfc", None) \
        or not self.mfc_controller.serial_mfc.isOpen():
            self.mfc_controller.connect_mfc_device()  # 성공/실패는 MFC 워치독이 재시도

        # 3. IG (QSerialPort + 워치독: 즉시 실패해도 백그라운드 재시도)
        if not getattr(self.ig_controller, "serial_ig", None) \
        or not self.ig_controller.serial_ig.isOpen():
            self.ig_controller.connect_ig_device()  # 성공/실패는 IG 워치독이 재시도
            
        # 4. OES (main에서 쓰레드를 사용하니 보완필요)
        if self.oes_controller.sChannel < 0:
            if not self.oes_controller.initialize_device(): 
                return False
        
        self.append_log("MAIN", "모든 장비가 성공적으로 연결되었습니다.")
        return True

    def _validate_single_run_inputs(self) -> dict | None:
        """UI 단일 공정 시작 전, 위→아래 순서로 즉시 팝업하며 검증. 성공 시 params 일부를 dict로 반환."""
        # ----------------------
        # 1) Gun 개수 (G1/G2/G3 중 1개 또는 2개만 허용)
        # ----------------------
        use_g1 = self.ui.G1_checkbox.isChecked()
        use_g2 = self.ui.G2_checkbox.isChecked()
        use_g3 = self.ui.G3_checkbox.isChecked()
        checked_count = int(use_g1) + int(use_g2) + int(use_g3)
        if checked_count == 0 or checked_count == 3:
            msg_box = QMessageBox(self)
            msg_box.setIcon(QMessageBox.Icon.Warning)
            msg_box.setWindowTitle("선택 오류")
            msg_box.setText("Gun 선택 개수를 확인해주세요.")
            msg_box.setInformativeText("G1, G2, G3 중 1개 또는 2개만 선택해야 합니다.")
            msg_box.setStandardButtons(QMessageBox.StandardButton.Ok)
            msg_box.exec()
            return None

        # ----------------------
        # 2) Gun 타겟 이름 (선택된 것만, G1 → G2 → G3 순서로 즉시 팝업)
        # ----------------------
        g1_name = self.ui.G1_target_name.toPlainText().strip()
        g2_name = self.ui.G2_target_name.toPlainText().strip()
        g3_name = self.ui.G3_target_name.toPlainText().strip()

        if use_g1 and not g1_name:
            QMessageBox.warning(self, "입력값 확인", "G1 타겟 이름이 비어 있습니다.")
            return None
        if use_g2 and not g2_name:
            QMessageBox.warning(self, "입력값 확인", "G2 타겟 이름이 비어 있습니다.")
            return None
        if use_g3 and not g3_name:
            QMessageBox.warning(self, "입력값 확인", "G3 타겟 이름이 비어 있습니다.")
            return None

        # ----------------------
        # 3) 가스 선택 (최소 1개) — 위에서 아래: Ar → O2 → N2
        # ----------------------
        use_ar = self.ui.Ar_checkbox.isChecked()
        use_o2 = self.ui.O2_checkbox.isChecked()
        use_n2 = self.ui.N2_checkbox.isChecked()
        if not (use_ar or use_o2 or use_n2):
            QMessageBox.warning(self, "선택 오류", "가스를 하나 이상 선택해야 합니다.")
            return None

        # ----------------------
        # 4) 선택된 가스의 유량 값 (존재/숫자/양수) — Ar → O2 → N2 순서
        # ----------------------
        if use_ar:
            txt = self.ui.Ar_flow_edit.toPlainText().strip()
            if not txt:
                QMessageBox.warning(self, "입력값 확인", "Ar 유량을 입력하세요.")
                return None
            try:
                ar_flow = float(txt)
                if ar_flow <= 0:
                    QMessageBox.warning(self, "입력값 확인", "Ar 유량은 0보다 커야 합니다.")
                    return None
            except ValueError:
                QMessageBox.warning(self, "입력값 확인", "Ar 유량이 올바른 수치가 아닙니다.")
                return None
        else:
            ar_flow = 0.0

        if use_o2:
            txt = self.ui.O2_flow_edit.toPlainText().strip()
            if not txt:
                QMessageBox.warning(self, "입력값 확인", "O2 유량을 입력하세요.")
                return None
            try:
                o2_flow = float(txt)
                if o2_flow <= 0:
                    QMessageBox.warning(self, "입력값 확인", "O2 유량은 0보다 커야 합니다.")
                    return None
            except ValueError:
                QMessageBox.warning(self, "입력값 확인", "O2 유량이 올바른 수치가 아닙니다.")
                return None
        else:
            o2_flow = 0.0

        if use_n2:
            txt = self.ui.N2_flow_edit.toPlainText().strip()
            if not txt:
                QMessageBox.warning(self, "입력값 확인", "N2 유량을 입력하세요.")
                return None
            try:
                n2_flow = float(txt)
                if n2_flow <= 0:
                    QMessageBox.warning(self, "입력값 확인", "N2 유량은 0보다 커야 합니다.")
                    return None
            except ValueError:
                QMessageBox.warning(self, "입력값 확인", "N2 유량이 올바른 수치가 아닙니다.")
                return None
        else:
            n2_flow = 0.0

        # ----------------------
        # 5) 파워 선택 (최소 1개) — RF → DC 순서
        # ----------------------
        use_rf = self.ui.RF_power_checkbox.isChecked()
        use_dc = self.ui.DC_power_checkbox.isChecked()
        if not (use_rf or use_dc):
            QMessageBox.warning(self, "선택 오류", "RF 파워 또는 DC 파워 중 하나 이상을 반드시 선택해야 합니다.")
            return None

        # ----------------------
        # 6) 선택된 파워 값 (존재/숫자/양수) — RF → DC 순서
        # ----------------------
        if use_rf:
            txt = self.ui.RF_power_edit.toPlainText().strip()
            if not txt:
                QMessageBox.warning(self, "입력값 확인", "RF 파워(W)를 입력하세요.")
                return None
            try:
                rf_power = float(txt)
                if rf_power <= 0:
                    QMessageBox.warning(self, "입력값 확인", "RF 파워(W)는 0보다 커야 합니다.")
                    return None
            except ValueError:
                QMessageBox.warning(self, "입력값 확인", "RF 파워(W)가 올바른 수치가 아닙니다.")
                return None
        else:
            rf_power = 0.0

        if use_dc:
            txt = self.ui.DC_power_edit.toPlainText().strip()
            if not txt:
                QMessageBox.warning(self, "입력값 확인", "DC 파워(W)를 입력하세요.")
                return None
            try:
                dc_power = float(txt)
                if dc_power <= 0:
                    QMessageBox.warning(self, "입력값 확인", "DC 파워(W)는 0보다 커야 합니다.")
                    return None
            except ValueError:
                QMessageBox.warning(self, "입력값 확인", "DC 파워(W)가 올바른 수치가 아닙니다.")
                return None
        else:
            dc_power = 0.0

        # ----------------------
        # 7) 성공: params 일부를 dict로 반환(네가 쓰는 키 이름에 맞춰)
        # ----------------------
        return {
            "use_ms": self.ui.Main_shutter_checkbox.isChecked(),
            "use_g1": use_g1, "use_g2": use_g2, "use_g3": use_g3,
            "use_ar": use_ar, "use_o2": use_o2, "use_n2": use_n2,
            "ar_flow": ar_flow, "o2_flow": o2_flow, "n2_flow": n2_flow,
            "use_rf_power": use_rf, "use_dc_power": use_dc,
            "rf_power": rf_power, "dc_power": dc_power,
            "G1_target_name": g1_name, "G2_target_name": g2_name, "G3_target_name": g3_name,
        }

    @Slot()
    @Slot(bool)
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
            # 1. 먼저 숫자 필드들 파싱
            try:
                base_pressure = float(self.ui.Base_pressure_edit.toPlainText() or 1e-5)
                integration_time = int(self.ui.Intergration_time_edit.toPlainText() or 60)
                working_pressure = float(self.ui.Working_pressure_edit.toPlainText() or 0.0)
                shutter_delay = float(self.ui.Shutter_delay_edit.toPlainText() or 0.0)
                process_time = float(self.ui.Process_time_edit.toPlainText() or 0.0) 

            except ValueError:
                self.append_log("UI", "오류: 값 입력란을 확인해주세요.")
                return
            
            # 2. 검증 + 값 생성
            vals = self._validate_single_run_inputs()
            if vals is None:
                return

            # 3. 최종 params
            params = {
                "base_pressure": base_pressure,
                "integration_time": integration_time,
                "working_pressure": working_pressure,
                "shutter_delay": shutter_delay,
                "process_time": process_time,
                "process_note": self.ui.note_edit.toPlainText(),
                **vals,
            }

            self.append_log("MAIN", "입력 검증 통과 → 공정 시작")
            # ProcessController로 파라미터를 전달하여 단일 공정 시작
            self.process_controller.start_process(params)

    @Slot(str, str)
    def append_log(self, source: str, msg: str):
        """
        로그 메시지를 UI와 log.txt 파일에 동시에 추가합니다.
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
        self.ui.Base_pressure_edit.setPlainText("9e-6") #
        self.ui.Intergration_time_edit.setPlainText("60") #
        self.ui.Working_pressure_edit.setPlainText("2") #
        self.ui.Process_time_edit.setPlainText("1") #
        self.ui.Shutter_delay_edit.setPlainText("1") #

        # --- 가스 및 건 ---
        self.ui.Ar_flow_edit.setPlainText("20") #
        self.ui.O2_flow_edit.setPlainText("0") #
        self.ui.N2_flow_edit.setPlainText("0") #

        # --- 파워 설정 ---
        self.ui.DC_power_edit.setPlainText("100") #
        self.ui.RF_power_edit.setPlainText("100") #

    def _reset_ui_after_process(self):
        """공정 종료(성공/실패) 후 UI를 초기 상태로 정리"""
        # 1) 입력값 기본치로 되돌리기
        self._set_default_ui_values()

        # 2) 체크박스 해제
        for cb in [
            self.ui.G1_checkbox, self.ui.G2_checkbox, self.ui.G3_checkbox,
            self.ui.Ar_checkbox, self.ui.O2_checkbox, self.ui.N2_checkbox,
            self.ui.Main_shutter_checkbox, self.ui.RF_power_checkbox, self.ui.DC_power_checkbox
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
        self.append_log("main", "프로그램 종료 절차 시작...")

        # 0) 공정이 돌고 있다면 비상 종료 시퀀스부터 시작 (장비들이 스스로 정리)
        try:
            if getattr(self.process_controller, "is_running", False):
                self.process_controller.abort_process()
                # abort가 장비 cleanup을 트리거하지만, 혹시 몰라 잠깐 대기
                for _ in range(6):
                    QCoreApplication.processEvents()
                    QThread.msleep(20)
        except Exception:
            pass

        # 1) 장비 직접 cleanup (이중 안전장치)
        try: self.faduino_controller.cleanup()
        except Exception: pass
        try: self.mfc_controller.cleanup()
        except Exception: pass
        try: self.ig_controller.cleanup()
        except Exception: pass
        try: self.oes_controller.cleanup()
        except Exception: pass
        # (RF/DC 컨트롤러는 Faduino를 통해 파워 0으로 내리므로 별도 close 불필요)

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
                self.append_log("main", f"{thread} 종료 완료.")

        self.append_log("Main", "모든 스레드 종료. 프로그램을 닫습니다.")
        super().closeEvent(event)

    @Slot()
    def _on_about_to_quit(self):
        """앱이 정상 종료되기 직전 마지막 청소(중복 호출 방지)."""
        if self._about_quit_called:
            return
        self._about_quit_called = True

        # 공정이 돌고 있으면 안전 종료 시퀀스 먼저
        try:
            if getattr(self.process_controller, "is_running", False):
                self.process_controller.abort_process()
                # abort 직후 ~100ms 정도 이벤트 소화(시그널/타이머 처리 유도)
                for _ in range(6):
                    QCoreApplication.processEvents()
                    QThread.msleep(20)
        except Exception:
            pass

        # 장치 정리(시리얼/타이머 닫기)
        for ctrl in (
            getattr(self, 'faduino_controller', None),
            getattr(self, 'mfc_controller', None),
            getattr(self, 'ig_controller', None),
            getattr(self, 'oes_controller', None),
        ):
            try:
                if ctrl:
                    ctrl.cleanup()
            except Exception:
                pass

    def _emergency_cleanup(self):
        """응답없음/예외 등 비정상 종료 시 마지막으로 포트 닫기(드물게라도 실행되도록)."""
        if self._emergency_done:
            return
        self._emergency_done = True

        # 이벤트 루프가 멈췄을 가능성이 있으니, Qt 위젯 조작은 하지 말고 장치 핸들만 정리
        try:
            if getattr(self, "process_controller", None) and \
               getattr(self.process_controller, "is_running", False):
                # 이벤트 루프가 죽었으면 abort가 즉시 효과 없을 수도 있지만, 시도는 무해
                self.process_controller.abort_process()
        except Exception:
            pass

        for ctrl in (
            getattr(self, 'faduino_controller', None),
            getattr(self, 'mfc_controller', None),
            getattr(self, 'ig_controller', None),
            getattr(self, 'oes_controller', None),
        ):
            try:
                if ctrl:
                    ctrl.cleanup()
            except Exception:
                pass


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
