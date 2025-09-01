# main.py
import sys
import csv
import atexit, signal
from datetime import datetime
from PyQt6.QtWidgets import QApplication, QWidget, QMessageBox, QFileDialog
from PyQt6.QtCore import QCoreApplication, Qt, QTimer, QThread, pyqtSlot as Slot, pyqtSignal as Signal

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
from device.RFPulse import RFPulseController
from controller.data_logger import DataLogger
from controller.process_controller import ProcessController


class MainWindow(QWidget):
    # UI → 장치(워커) 요청 신호들
    request_faduino_connect = Signal()
    request_mfc_connect     = Signal()
    request_ig_connect      = Signal()
    request_rfpulse_connect = Signal()   # ← 추가: RF Pulse 연결 요청
    request_oes_initialize  = Signal()
    request_faduino_cleanup = Signal()
    request_mfc_cleanup     = Signal()
    request_ig_cleanup      = Signal()
    request_oes_cleanup     = Signal()
    request_rfpulse_cleanup = Signal()  # ← 추가: RF Pulse 정리 요청

    stop_all = Signal() # 모든 장치 하드스톱

    def __init__(self):
        super().__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self._set_default_ui_values()
        self.process_queue = []
        self.current_process_index = -1

        # === 1. 메인 스레드 컨트롤러 ===
        self.graph_controller = GraphController(self.ui.RGA_Graph, self.ui.OES_Graph)
        self.dc_power_controller = DCPowerController()
        self.rf_power_controller = RFPowerController()
        self.rf_pulse_controller = RFPulseController()  # 워커 스레드로 이동시킬 예정

        # === 2. 워커 스레드 및 장치 컨트롤러 ===
        # Faduino
        self.faduino_thread = QThread(self); 
        self.faduino_thread.setObjectName("FaduinoThread")
        self.faduino_controller = FaduinoController(); 
        self.faduino_controller.moveToThread(self.faduino_thread)

        # MFC
        self.mfc_thread = QThread(self); 
        self.mfc_thread.setObjectName("MFCThread")
        self.mfc_controller = MFCController(); 
        self.mfc_controller.moveToThread(self.mfc_thread)

        # OES
        self.oes_thread = QThread(self); 
        self.oes_thread.setObjectName("OESThread")
        self.oes_controller = OESController(); 
        self.oes_controller.moveToThread(self.oes_thread)

        # IG
        self.ig_thread = QThread(self); 
        self.ig_thread.setObjectName("IGThread")
        self.ig_controller = IGController(); 
        self.ig_controller.moveToThread(self.ig_thread)

        # RGA
        self.rga_thread = QThread(self); 
        self.rga_thread.setObjectName("RGAThread")
        self.rga_controller = RGAController(); 
        self.rga_controller.moveToThread(self.rga_thread)

        # DataLogger
        self.data_logger_thread = QThread(self); 
        self.data_logger_thread.setObjectName("DataLoggerThread")
        self.data_logger = DataLogger(); 
        self.data_logger.moveToThread(self.data_logger_thread)

        # RF Pulse (새 워커 스레드)
        self.rf_pulse_thread = QThread(self)
        self.rf_pulse_thread.setObjectName("RFPulseThread")
        self.rf_pulse_controller.moveToThread(self.rf_pulse_thread)

        # === 3. 감독관 ===
        self.process_controller = ProcessController()

        # === 4. 신호-슬롯 ===
        self._connect_signals()

        # === 5. 워커 스레드 시작 ===
        self.faduino_thread.start()
        self.mfc_thread.start()
        self.oes_thread.start()
        self.ig_thread.start()
        self.rga_thread.start()
        self.data_logger_thread.start()
        self.rf_pulse_thread.start()  # ← 추가: RF Pulse 워커 시작

        # 공정 완료 → 다음 공정
        self.process_controller.process_finished.connect(self._start_next_process_from_queue)

        # === 종료 처리 훅 ===
        self._about_quit_called = False
        self._emergency_done = False

        app = QCoreApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self._on_about_to_quit)

        atexit.register(self._emergency_cleanup)

        def _sig_handler(*_):
            self._emergency_cleanup()
        for sig in (getattr(signal, "SIGINT", None),
                    getattr(signal, "SIGTERM", None),
                    getattr(signal, "SIGBREAK", None)):
            if sig is not None:
                try:
                    signal.signal(sig, _sig_handler)
                except Exception:
                    pass

    def _connect_signals(self):
        # === DataLogger ===
        self.process_controller.process_started.connect(self.data_logger.start_new_log_session)
        self.process_controller.process_finished.connect(self.data_logger.finalize_and_write_log)
        self.ig_controller.pressure_update.connect(self.data_logger.log_ig_pressure)
        self.faduino_controller.dc_power_updated.connect(self.data_logger.log_dc_power)
        self.faduino_controller.rf_power_updated.connect(self.data_logger.log_rf_power)
        self.mfc_controller.update_flow.connect(self.data_logger.log_mfc_flow)
        self.mfc_controller.update_pressure.connect(self.data_logger.log_mfc_pressure)

        # 공정이 시작될 때만 그래프 초기화 (수동/자동 모두 포함)
        self.process_controller.process_started.connect(
            lambda *args: self.graph_controller.reset(),
            type=Qt.ConnectionType.QueuedConnection
        )

        # === 로그 / 상태 ===
        for src in (self.faduino_controller, self.mfc_controller,
                    self.oes_controller, self.ig_controller, self.rga_controller):
            src.status_message.connect(self.append_log)
        self.dc_power_controller.status_message.connect(self.append_log)
        self.rf_power_controller.status_message.connect(self.append_log)
        self.rf_pulse_controller.status_message.connect(self.append_log)
        self.process_controller.log_message.connect(self.append_log)
        self.process_controller.update_process_state.connect(self.on_update_process_state)

        # === ProcessController -> 장치 (cross-thread) ===
        self.process_controller.update_faduino_port.connect(
            self.faduino_controller.handle_named_command,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.process_controller.mfc_command_requested.connect(
            self.mfc_controller.handle_command,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.process_controller.oes_command_requested.connect(
            self.oes_controller.run_measurement,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.process_controller.dc_power_command_requested.connect(
            self.dc_power_controller.start_process,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.process_controller.dc_power_stop_requested.connect(
            self.dc_power_controller.stop_process,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.process_controller.rf_power_command_requested.connect(
            self.rf_power_controller.start_process,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.process_controller.rf_power_stop_requested.connect(
            self.rf_power_controller.stop_process,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.process_controller.rf_pulse_command_requested.connect(
            self.rf_pulse_controller.start_pulse_process,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.process_controller.rf_pulse_stop_requested.connect(
            self.rf_pulse_controller.stop_process,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.process_controller.rga_external_scan_requested.connect(
            self.rga_controller.execute_external_scan,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.process_controller.ig_command_requested.connect(
            self.ig_controller.start_wait_for_pressure,
            type=Qt.ConnectionType.QueuedConnection
        )

        # === 명령 완료, 실패 시그널 ===
        self.rga_controller.scan_finished.connect(
            self.process_controller.on_rga_finished,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.rga_controller.scan_failed.connect(
            self.process_controller.on_rga_failed,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.oes_controller.oes_finished.connect(
            self.process_controller.on_oes_ok,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.oes_controller.oes_failed.connect(
            self.process_controller.on_oes_failed,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.ig_controller.base_pressure_reached.connect(
            self.process_controller.on_ig_ok,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.ig_controller.base_pressure_failed.connect(
            self.process_controller.on_ig_failed,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.mfc_controller.command_confirmed.connect(
            self.process_controller.on_mfc_confirmed,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.mfc_controller.command_failed.connect(
            self.process_controller.on_mfc_failed,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.faduino_controller.command_confirmed.connect(
            self.process_controller.on_faduino_confirmed,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.faduino_controller.command_failed.connect(
            self.process_controller.on_faduino_failed,
            type=Qt.ConnectionType.QueuedConnection
        )

        # DC 파워 목표 도달 → 공정 진행 (DC는 실패 없음)
        self.dc_power_controller.target_reached.connect(
            self.process_controller.on_dc_target_reached,
            type=Qt.ConnectionType.QueuedConnection
        )
        # ⬇︎ 추가: DC 파워 정지 완료 → 공정 진행
        self.dc_power_controller.power_off_finished.connect(
            self.process_controller.on_device_step_ok,
            type=Qt.ConnectionType.QueuedConnection
        )

        # RF 파워 목표 도달/실패 → 공정 진행/실패
        self.rf_power_controller.target_reached.connect(
            self.process_controller.on_rf_target_reached,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.rf_power_controller.target_failed.connect(
            lambda why: self.process_controller.on_step_failed("RF Power", why),
            type=Qt.ConnectionType.QueuedConnection
        )
        self.rf_power_controller.power_off_finished.connect(
            self.process_controller.on_device_step_ok,
            type=Qt.ConnectionType.QueuedConnection
        )

        # RF Pulse 목표 도달 → 공정 진행
        self.rf_pulse_controller.target_reached.connect(
            self.process_controller.on_rf_target_reached,
            type=Qt.ConnectionType.QueuedConnection
        )
        # RF Pulse 실패 → 공정 실패 처리
        self.rf_pulse_controller.target_failed.connect(
            lambda why: self.process_controller.on_step_failed("RF Pulse", why),
            type=Qt.ConnectionType.QueuedConnection
        )
        # RF Pulse OFF 완료 → 공정 다음 스텝 진행
        self.rf_pulse_controller.power_off_finished.connect(
            self.process_controller.on_device_step_ok,
            type=Qt.ConnectionType.QueuedConnection
        )

        # === set_polling (cross-thread) ===
        self.process_controller.set_polling.connect(
            self.faduino_controller.set_process_status,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.process_controller.set_polling.connect(
            self.mfc_controller.set_process_status,
            type=Qt.ConnectionType.QueuedConnection
        )

        # ✅ 공정 종료 시 큐 즉시 비우기 + 폴링/강제읽기 완전 중단
        self.process_controller.process_finished.connect(
            self.faduino_controller.on_process_finished, 
            type=Qt.ConnectionType.QueuedConnection
            )
        self.process_controller.process_finished.connect(
            self.mfc_controller.on_process_finished,   
            type=Qt.ConnectionType.QueuedConnection
            )

        # === 비상 종료(정리) : cleanup은 BlockingQueued ===
        self.process_controller.process_aborted.connect(
            self.mfc_controller.cleanup,
            type=Qt.ConnectionType.BlockingQueuedConnection
        )
        self.process_controller.process_aborted.connect(
            self.ig_controller.cleanup,
            type=Qt.ConnectionType.BlockingQueuedConnection
        )
        self.process_controller.process_aborted.connect(
            self.faduino_controller.cleanup,
            type=Qt.ConnectionType.BlockingQueuedConnection
        )
        self.process_controller.process_aborted.connect(
            self.oes_controller.cleanup,
            type=Qt.ConnectionType.BlockingQueuedConnection
        )
        self.process_controller.process_aborted.connect(
            self.rf_power_controller.stop_process,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.process_controller.process_aborted.connect(
            self.dc_power_controller.stop_process,
            type=Qt.ConnectionType.QueuedConnection
        )
        # 기존: stop_process → 변경: cleanup (BlockingQueued)
        self.process_controller.process_aborted.connect(
            self.rf_pulse_controller.cleanup,
            type=Qt.ConnectionType.BlockingQueuedConnection
        )

        # === STOP ALL: 즉시 장치 하드스톱/정리 ===
        self.stop_all.connect(
            self.mfc_controller.cleanup,
            type=Qt.ConnectionType.BlockingQueuedConnection
        )
        self.stop_all.connect(
            self.faduino_controller.cleanup,
            type=Qt.ConnectionType.BlockingQueuedConnection
        )
        self.stop_all.connect(
            self.ig_controller.cleanup,
            type=Qt.ConnectionType.BlockingQueuedConnection
        )
        self.stop_all.connect(
            self.oes_controller.cleanup,
            type=Qt.ConnectionType.BlockingQueuedConnection
        )
        self.stop_all.connect(
            self.rf_power_controller.stop_process,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.stop_all.connect(
            self.dc_power_controller.stop_process,
            type=Qt.ConnectionType.QueuedConnection
        )
        # 기존: stop_process → 변경: cleanup (BlockingQueued)
        self.stop_all.connect(
            self.rf_pulse_controller.cleanup,
            type=Qt.ConnectionType.BlockingQueuedConnection
        )

        # === UI → 장치(연결/정리/초기화) 요청 ===
        self.request_faduino_connect.connect(
            self.faduino_controller.connect_faduino,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.request_mfc_connect.connect(
            self.mfc_controller.connect_mfc_device,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.request_ig_connect.connect(
            self.ig_controller.connect_ig_device,
            type=Qt.ConnectionType.QueuedConnection
        )
        self.request_rfpulse_connect.connect(
            self.rf_pulse_controller.connect_rfpulse_device,
            type=Qt.ConnectionType.QueuedConnection
        )

        # OES 초기화/정리(워커 스레드에서 실행, BlockingQueued로 동기 보장)
        self.request_oes_initialize.connect(
            self.oes_controller.initialize_device,
            type=Qt.ConnectionType.BlockingQueuedConnection
        )
        self.request_oes_cleanup.connect(
            self.oes_controller.cleanup,
            type=Qt.ConnectionType.BlockingQueuedConnection
        )

        # ▼ cleanup 신호 연결(각 워커 스레드에서 안전 종료)
        self.request_faduino_cleanup.connect(
            self.faduino_controller.cleanup,
            type=Qt.ConnectionType.BlockingQueuedConnection
        )
        self.request_mfc_cleanup.connect(
            self.mfc_controller.cleanup,
            type=Qt.ConnectionType.BlockingQueuedConnection
        )
        self.request_ig_cleanup.connect(
            self.ig_controller.cleanup,
            type=Qt.ConnectionType.BlockingQueuedConnection
        )
        self.request_rfpulse_cleanup.connect(
            self.rf_pulse_controller.cleanup,
            type=Qt.ConnectionType.BlockingQueuedConnection
        )

        # === 파워 ↔ Faduino 상태 연동 / 표시 ===
        self.rf_power_controller.state_changed.connect(self.faduino_controller.on_rf_state_changed)
        self.dc_power_controller.state_changed.connect(self.faduino_controller.on_dc_state_changed)
        self.dc_power_controller.request_status_read.connect(self.faduino_controller.force_dc_read)
        self.dc_power_controller.send_dc_power_value.connect(self.faduino_controller.set_dc_power)
        self.dc_power_controller.send_dc_power_value_unverified.connect(self.faduino_controller.set_dc_power_unverified)
        self.faduino_controller.dc_power_updated.connect(self.dc_power_controller.update_measurements)
        self.dc_power_controller.update_dc_status_display.connect(self.handle_dc_power_display)

        self.rf_power_controller.request_status_read.connect(self.faduino_controller.force_rf_read)
        self.rf_power_controller.send_rf_power_value.connect(self.faduino_controller.set_rf_power)
        self.rf_power_controller.send_rf_power_value_unverified.connect(self.faduino_controller.set_rf_power_unverified)
        self.faduino_controller.rf_power_updated.connect(self.rf_power_controller.update_measurements)
        self.rf_power_controller.update_rf_status_display.connect(self.handle_rf_power_display)

        # === 그래프 / UI 업데이트 ===
        self.rga_controller.rga_data_updated.connect(self.graph_controller.update_rga_plot)
        self.oes_controller.oes_data_updated.connect(self.graph_controller.update_oes_plot)
        self.mfc_controller.update_flow.connect(self.update_mfc_flow_ui)
        self.mfc_controller.update_pressure.connect(self.update_mfc_pressure_ui)

        # === 버튼/상태 ===
        self.ui.Start_button.clicked.connect(self.on_start_button_clicked)
        self.ui.Stop_button.clicked.connect(self.on_stop_button_clicked)
        self.ui.Process_list_button.clicked.connect(self.on_process_list_button_clicked)
        self.process_controller.process_status_changed.connect(self._on_process_status_changed)
        self._on_process_status_changed(False)

    # --- 표시용 슬롯 ---
    @Slot(float, float)
    def handle_rf_power_display(self, for_p, ref_p):
        if for_p is None or ref_p is None:
            self.append_log("MAIN", "for.p, ref.p 값이 비어있습니다.")
            return
        self.ui.For_p_edit.setPlainText(f"{for_p:.2f}")
        self.ui.Ref_p_edit.setPlainText(f"{ref_p:.2f}")

    @Slot(float, float, float)
    def handle_dc_power_display(self, power, voltage, current):
        if power is None or voltage is None or current is None:
            self.append_log("MAIN", "power, voltage, current값이 비어있습니다.")
            return
        self.ui.Power_edit.setPlainText(f"{power:.3f}")
        self.ui.Voltage_edit.setPlainText(f"{voltage:.3f}")
        self.ui.Current_edit.setPlainText(f"{current:.3f}")

    @Slot(str)
    def update_mfc_pressure_ui(self, pressure_value):
        self.ui.Working_pressure_edit.setPlainText(pressure_value)

    @Slot(str, float)
    def update_mfc_flow_ui(self, gas_name, flow_value):
        if gas_name == "Ar":
            self.ui.Ar_flow_edit.setPlainText(f"{flow_value:.1f}")
        elif gas_name == "O2":
            self.ui.O2_flow_edit.setPlainText(f"{flow_value:.1f}")
        elif gas_name == "N2":
            self.ui.N2_flow_edit.setPlainText(f"{flow_value:.1f}")

    @Slot(bool)
    def _on_process_status_changed(self, running: bool):
        self.ui.Start_button.setEnabled(not running)
        self.ui.Stop_button.setEnabled(True)

    @Slot()
    @Slot(bool)
    def on_process_list_button_clicked(self, _checked: bool=False):
        file_path, _ = QFileDialog.getOpenFileName(
            self, "프로세스 리스트 파일 선택", "", "CSV Files (*.csv);;All Files (*)"
        )
        if not file_path:
            self.append_log("File", "파일 선택이 취소되었습니다.")
            return

        self.append_log("File", f"선택된 파일: {file_path}")
        try:
            with open(file_path, mode='r', encoding='utf-8-sig') as csvfile:
                reader = csv.DictReader(csvfile)
                self.process_queue = []
                self.current_process_index = -1
                for row in reader:
                    row['Process_name'] = row.get('#', f'공정 {len(self.process_queue) + 1}')
                    self.process_queue.append(row)

                if not self.process_queue:
                    self.append_log("File", "파일에 처리할 공정이 없습니다.")
                    return
                self.append_log("File", f"총 {len(self.process_queue)}개의 공정을 파일에서 읽었습니다.")
                self._update_ui_from_params(self.process_queue[0])
        except Exception as e:
            self.append_log("File", f"파일 처리 중 오류 발생: {e}")

    def _update_ui_from_params(self, params: dict):
        if self.process_queue:
            total = len(self.process_queue)
            current = self.current_process_index + 1
            progress_text = f"자동 공정 ({current}/{total}): '{params.get('Process_name', '이름없음')}' 준비 중..."
            self.append_log("UI", progress_text)
        else:
            self.append_log("UI", f"단일 공정 '{params.get('process_note', '이름없음')}'의 파라미터로 UI를 업데이트합니다.")

        self.append_log("UI", f"다음 공정 '{params['Process_name']}'의 파라미터로 UI를 업데이트합니다.")

        # --- 파워 / RF-Pulse ---
        self.ui.RF_power_edit.setPlainText(str(params.get('rf_power', '0')))
        self.ui.DC_power_edit.setPlainText(str(params.get('dc_power', '0')))

        # RF Pulse UI 동기화 (CSV 컬럼명 그대로 사용)
        self.ui.RF_pulse_checkbox.setChecked(params.get('use_rf_pulse_power', 'F') == 'T')
        self.ui.RF_pulse_edit.setPlainText(str(params.get('rf_pulse_power', '0')))

        # freq, duty: 미입력/0이면 공란으로 표시(= 변경하지 않음 의미)
        freq_raw = str(params.get('rf_pulse_freq', '')).strip()
        duty_raw = str(params.get('rf_pulse_duty_cycle', '')).strip()
        self.ui.Rf_pulse_freq_edit.setPlainText('' if freq_raw in ('', '0') else freq_raw)
        self.ui.Rf_pulse_duty_cycle_edit.setPlainText('' if duty_raw in ('', '0') else duty_raw)

        # --- 시간/압력/가스 ---
        self.ui.Process_time_edit.setPlainText(str(params.get('process_time', '0')))
        self.ui.Intergration_time_edit.setPlainText(str(params.get('integration_time', '60')))
        self.ui.Ar_flow_edit.setPlainText(str(params.get('Ar_flow', '0')))
        self.ui.O2_flow_edit.setPlainText(str(params.get('O2_flow', '0')))
        self.ui.N2_flow_edit.setPlainText(str(params.get('N2_flow', '0')))
        self.ui.Working_pressure_edit.setPlainText(str(params.get('working_pressure', '0')))
        self.ui.Base_pressure_edit.setPlainText(str(params.get('base_pressure', '0')))
        self.ui.Shutter_delay_edit.setPlainText(str(params.get('shutter_delay', '0')))

        # --- 체크박스들 (CSV 값이 'T'/'F'라고 가정) ---
        self.ui.G1_checkbox.setChecked(params.get('gun1', 'F') == 'T')
        self.ui.G2_checkbox.setChecked(params.get('gun2', 'F') == 'T')
        self.ui.G3_checkbox.setChecked(params.get('gun3', 'F') == 'T')
        self.ui.Ar_checkbox.setChecked(params.get('Ar', 'F') == 'T')
        self.ui.O2_checkbox.setChecked(params.get('O2', 'F') == 'T')
        self.ui.N2_checkbox.setChecked(params.get('N2', 'F') == 'T')
        self.ui.Main_shutter_checkbox.setChecked(params.get('main_shutter', 'F') == 'T')
        self.ui.RF_power_checkbox.setChecked(params.get('use_rf_power', 'F') == 'T')
        self.ui.DC_power_checkbox.setChecked(params.get('use_dc_power', 'F') == 'T')

        self.ui.Power_select_checkbox.setChecked(params.get('power_select', 'F') == 'T')

        # --- 타겟명 (CSV 헤더 그대로 표시) ---
        self.ui.G1_target_name.setPlainText(str(params.get('G1 Target', '')).strip())
        self.ui.G2_target_name.setPlainText(str(params.get('G2 Target', '')).strip())
        self.ui.G3_target_name.setPlainText(str(params.get('G3 Target', '')).strip())

    @Slot(bool)
    def _start_next_process_from_queue(self, was_successful: bool):
        if self.process_controller.is_running and self.current_process_index > -1:
            self.append_log("MAIN", "경고: 다음 공정 자동 전환 시점에 이미 다른 공정이 실행 중입니다.")
            return
        
        if not was_successful:
            self.append_log("MAIN", "이전 공정이 실패하여 자동 시퀀스를 중단합니다.")
            self.process_queue = []
            self.current_process_index = -1
            self._reset_ui_after_process()
            return
        
        self.current_process_index += 1

        if self.current_process_index < len(self.process_queue):
            params = self.process_queue[self.current_process_index]
            self._update_ui_from_params(params)
            QTimer.singleShot(100, lambda p=params: self._safe_start_process(self._normalize_params_for_process(p)))
        else:
            self.append_log("MAIN", "모든 공정이 완료되었습니다.")
            self.process_queue = []
            self.current_process_index = -1
            self._reset_ui_after_process()

    def _safe_start_process(self, params: dict):
        if self.process_controller.is_running:
            self.append_log("MAIN", "경고: 이미 다른 공정이 실행 중이므로 새 공정을 시작하지 않습니다.")
            return
        
        # ✅ 매 공정 시작 직전 장비 재확인/재초기화(특히 OES)
        if not self._check_and_connect_devices():
            self.append_log("MAIN", "장비 재확인 실패 → 자동 시퀀스를 중단합니다.")
            self._start_next_process_from_queue(False)
            return

        try:
            self.process_controller.start_process(params)
        except Exception as e:
            self.append_log("MAIN", f"오류: '{params.get('Process_name', '알 수 없는')} 공정' 시작에 실패했습니다. ({e})")
            self._start_next_process_from_queue(False)

    def _check_and_connect_devices(self):
        """워커 스레드 컨트롤러들이 '지연 생성'된 시리얼을 열도록 신호만 보낸다."""
        # Faduino
        if not getattr(self.faduino_controller, "serial_faduino", None) \
        or not self.faduino_controller.serial_faduino.isOpen():
            self.request_faduino_connect.emit()

        # MFC
        if not getattr(self.mfc_controller, "serial_mfc", None) \
        or not self.mfc_controller.serial_mfc.isOpen():
            self.request_mfc_connect.emit()

        # IG
        if not getattr(self.ig_controller, "serial_ig", None) \
        or not self.ig_controller.serial_ig.isOpen():
            self.request_ig_connect.emit()

        # OES: 워커 스레드에서 초기화 실행(BlockingQueued) 후 상태 확인
        if getattr(self.oes_controller, "sChannel", -1) < 0:
            self.append_log("MAIN", "OES 초기화를 시도합니다...")
            self.request_oes_initialize.emit()
            if getattr(self.oes_controller, "sChannel", -1) < 0:
                self.append_log("MAIN", "OES 초기화 실패.")
                return False

        # RF Pulse: 해당 공정에서 사용할 예정이면 연결 (컨트롤러에서 중복/지연 연결 처리)
        if self.ui.RF_pulse_checkbox.isChecked():
            self.append_log("MAIN", "RF Pulse 연결을 시도합니다…")
            self.request_rfpulse_connect.emit()

        self.append_log("MAIN", "모든 장비가 성공적으로 연결되었습니다.")
        return True

    def _validate_single_run_inputs(self) -> dict | None:
        # (생략: 기존 코드 그대로)
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

        g1_name = self.ui.G1_target_name.toPlainText().strip()
        g2_name = self.ui.G2_target_name.toPlainText().strip()
        g3_name = self.ui.G3_target_name.toPlainText().strip()
        if use_g1 and not g1_name:
            QMessageBox.warning(self, "입력값 확인", "G1 타겟 이름이 비어 있습니다."); return None
        if use_g2 and not g2_name:
            QMessageBox.warning(self, "입력값 확인", "G2 타겟 이름이 비어 있습니다."); return None
        if use_g3 and not g3_name:
            QMessageBox.warning(self, "입력값 확인", "G3 타겟 이름이 비어 있습니다."); return None

        use_ar = self.ui.Ar_checkbox.isChecked()
        use_o2 = self.ui.O2_checkbox.isChecked()
        use_n2 = self.ui.N2_checkbox.isChecked()
        if not (use_ar or use_o2 or use_n2):
            QMessageBox.warning(self, "선택 오류", "가스를 하나 이상 선택해야 합니다."); return None

        if use_ar:
            txt = self.ui.Ar_flow_edit.toPlainText().strip()
            if not txt: QMessageBox.warning(self, "입력값 확인", "Ar 유량을 입력하세요."); return None
            try:
                ar_flow = float(txt); 
                if ar_flow <= 0: QMessageBox.warning(self, "입력값 확인", "Ar 유량은 0보다 커야 합니다."); return None
            except ValueError:
                QMessageBox.warning(self, "입력값 확인", "Ar 유량이 올바른 수치가 아닙니다."); return None
        else:
            ar_flow = 0.0

        if use_o2:
            txt = self.ui.O2_flow_edit.toPlainText().strip()
            if not txt: QMessageBox.warning(self, "입력값 확인", "O2 유량을 입력하세요."); return None
            try:
                o2_flow = float(txt); 
                if o2_flow <= 0: QMessageBox.warning(self, "입력값 확인", "O2 유량은 0보다 커야 합니다."); return None
            except ValueError:
                QMessageBox.warning(self, "입력값 확인", "O2 유량이 올바른 수치가 아닙니다."); return None
        else:
            o2_flow = 0.0

        if use_n2:
            txt = self.ui.N2_flow_edit.toPlainText().strip()
            if not txt: QMessageBox.warning(self, "입력값 확인", "N2 유량을 입력하세요."); return None
            try:
                n2_flow = float(txt); 
                if n2_flow <= 0: QMessageBox.warning(self, "입력값 확인", "N2 유량은 0보다 커야 합니다."); return None
            except ValueError:
                QMessageBox.warning(self, "입력값 확인", "N2 유량이 올바른 수치가 아닙니다."); return None
        else:
            n2_flow = 0.0

        use_rf = self.ui.RF_power_checkbox.isChecked()
        use_rf_pulse = self.ui.RF_pulse_checkbox.isChecked()
        use_dc = self.ui.DC_power_checkbox.isChecked()

        if not (use_rf or use_rf_pulse or use_dc):
            QMessageBox.warning(self, "선택 오류", "RF 파워, RF Pulse, DC 파워 중 하나 이상을 반드시 선택해야 합니다.")
            return None

        # RF 일반과 RF Pulse는 동시에 켜지 않도록 금지
        if use_rf and use_rf_pulse:
            QMessageBox.warning(self, "선택 오류", "RF 전원과 RF Pulse는 동시에 선택할 수 없습니다. 하나만 선택하세요.")
            return None

        if use_rf:
            txt = self.ui.RF_power_edit.toPlainText().strip()
            if not txt: QMessageBox.warning(self, "입력값 확인", "RF 파워(W)를 입력하세요."); return None
            try:
                rf_power = float(txt); 
                if rf_power <= 0: QMessageBox.warning(self, "입력값 확인", "RF 파워(W)는 0보다 커야 합니다."); return None
            except ValueError:
                QMessageBox.warning(self, "입력값 확인", "RF 파워(W)가 올바른 수치가 아닙니다."); return None
        else:
            rf_power = 0.0

        # RF Pulse: target power는 필수, freq/duty는 선택(미입력 시 변경하지 않음)
        rf_pulse_power = 0.0
        rf_pulse_freq = None
        rf_pulse_duty = None

        if use_rf_pulse:
            txtp = self.ui.RF_pulse_edit.toPlainText().strip()
            if not txtp:
                QMessageBox.warning(self, "입력값 확인", "RF Pulse Target Power(W)를 입력하세요.")
                return None
            try:
                rf_pulse_power = float(txtp)
                if rf_pulse_power <= 0:
                    QMessageBox.warning(self, "입력값 확인", "RF Pulse Target Power(W)는 0보다 커야 합니다.")
                    return None
            except ValueError:
                QMessageBox.warning(self, "입력값 확인", "RF Pulse Target Power(W)가 올바른 수치가 아닙니다.")
                return None

            # 선택 입력: 미입력 시 None → 나중에 컨트롤러가 '변경하지 않음' 처리
            txtf = self.ui.Rf_pulse_freq_edit.toPlainText().strip()
            if txtf:
                try:
                    rf_pulse_freq = int(float(txtf))
                    if rf_pulse_freq < 1 or rf_pulse_freq > 100000:
                        QMessageBox.warning(self, "입력값 확인", "RF Pulse Freq(Hz)는 1..100000 범위로 입력하세요.")
                        return None
                except ValueError:
                    QMessageBox.warning(self, "입력값 확인", "RF Pulse Freq(Hz)가 올바른 수치가 아닙니다.")
                    return None

            txtd = self.ui.Rf_pulse_duty_cycle_edit.toPlainText().strip()
            if txtd:
                try:
                    rf_pulse_duty = int(float(txtd))
                    if rf_pulse_duty < 1 or rf_pulse_duty > 99:
                        QMessageBox.warning(self, "입력값 확인", "RF Pulse Duty(%)는 1..99 범위로 입력하세요.")
                        return None
                except ValueError:
                    QMessageBox.warning(self, "입력값 확인", "RF Pulse Duty(%)가 올바른 수치가 아닙니다.")
                    return None

        if use_dc:
            txt = self.ui.DC_power_edit.toPlainText().strip()
            if not txt: QMessageBox.warning(self, "입력값 확인", "DC 파워(W)를 입력하세요."); return None
            try:
                dc_power = float(txt); 
                if dc_power <= 0: QMessageBox.warning(self, "입력값 확인", "DC 파워(W)는 0보다 커야 합니다."); return None
            except ValueError:
                QMessageBox.warning(self, "입력값 확인", "DC 파워(W)가 올바른 수치가 아닙니다."); return None
        else:
            dc_power = 0.0

        return {
            "use_ms": self.ui.Main_shutter_checkbox.isChecked(),
            "use_g1": use_g1, "use_g2": use_g2, "use_g3": use_g3,
            "use_ar": use_ar, "use_o2": use_o2, "use_n2": use_n2,
            "ar_flow": ar_flow, "o2_flow": o2_flow, "n2_flow": n2_flow,
            "use_rf_power": use_rf, "use_rf_pulse": use_rf_pulse, "use_dc_power": use_dc,
            "rf_power": rf_power, "rf_pulse_power": rf_pulse_power, "dc_power": dc_power,
            "rf_pulse_freq": rf_pulse_freq, "rf_pulse_duty": rf_pulse_duty, 
            "G1_target_name": g1_name, "G2_target_name": g2_name, "G3_target_name": g3_name,

            # ★ 추가
            "use_power_select": self.ui.Power_select_checkbox.isChecked(),
        }

    @Slot()
    @Slot(bool)
    def on_start_button_clicked(self, _checked: bool=False):
        if self.process_controller.is_running:
            QMessageBox.warning(self, "실행 오류", "현재 다른 공정이 실행 중입니다.")
            return
        
        # ★ 자동 시퀀스: 여기서는 장비 체크를 하지 않음
        #    (_safe_start_process() 내부에서 장비 체크/초기화 수행)
        if self.process_queue:
            self.append_log("MAIN", "입력받은 파일로 자동 공정 시퀀스를 시작합니다.")
            self.current_process_index = -1
            self._start_next_process_from_queue(True)
            return  # ← 중요: 아래 단일 실행 코드로 낙하 방지
        
        # ★ 단일 실행: 여기서만 장비 체크
        if not self._check_and_connect_devices():
            QMessageBox.critical(self, "장비 연결 오류", "필수 장비 연결에 실패했습니다. 로그를 확인하세요.")
            return

        try:
            base_pressure = float(self.ui.Base_pressure_edit.toPlainText() or 1e-5)
            integration_time = int(self.ui.Intergration_time_edit.toPlainText() or 60)
            working_pressure = float(self.ui.Working_pressure_edit.toPlainText() or 0.0)
            shutter_delay = float(self.ui.Shutter_delay_edit.toPlainText() or 0.0)
            process_time = float(self.ui.Process_time_edit.toPlainText() or 0.0)
        except ValueError:
            self.append_log("UI", "오류: 값 입력란을 확인해주세요.")
            return

        vals = self._validate_single_run_inputs()
        if vals is None:
            return

        params = {
            "base_pressure": base_pressure,
            "integration_time": integration_time,
            "working_pressure": working_pressure,
            "shutter_delay": shutter_delay,
            "process_time": process_time,
            "process_note": self.ui.note_edit.toPlainText(),
            **vals,
        }

        # ★ 수동 실행 시에도 DataLogger가 원본 헤더로 기록할 수 있도록 동기화
        params["G1 Target"] = vals.get("G1_target_name", "")
        params["G2 Target"] = vals.get("G2_target_name", "")
        params["G3 Target"] = vals.get("G3_target_name", "")

        self.append_log("MAIN", "입력 검증 통과 → 공정 시작")
        self.process_controller.start_process(params)

    @Slot()
    def on_stop_button_clicked(self, _checked: bool=False):
        if not self.process_controller.is_running and not self.process_queue:
            self.append_log("MAIN", "공정 미실행 상태이지만 종료 절차를 수행합니다.")

        # 1) 정상 정지 요청 → 종료 시퀀스로 전환
        self.process_controller.request_stop()

        # 2) 그리고 즉시 장치 하드스톱 (큐/타이머/시리얼 정리, 파워 정지)
        #self.stop_all.emit()

        if self.process_queue:
            self.append_log("MAIN", "자동 시퀀스가 사용자에 의해 중단되었습니다.")
            self.process_queue = []
            self.current_process_index = -1

    @Slot(str, str)
    def append_log(self, source: str, msg: str):
        ui_now = datetime.now().strftime("%H:%M:%S")
        ui_msg = f"[{ui_now}] [{source}] {msg}"
        self.ui.log_message.appendPlainText(ui_msg)

        try:
            file_now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            file_msg = f"[{file_now}] [{source}] {msg}\n"
            with open("log.txt", "a", encoding="utf-8") as f:
                f.write(file_msg)
        except Exception as e:
            error_msg = f"[{ui_now}] [Logger] 파일 로그 작성 실패: {e}"
            self.ui.log_message.appendPlainText(error_msg)

    @Slot(str)
    def on_update_process_state(self, message: str):
        self.ui.process_state.setPlainText(message)

    def _set_default_ui_values(self):
        self.ui.Base_pressure_edit.setPlainText("9e-6")
        self.ui.Intergration_time_edit.setPlainText("60")
        self.ui.Working_pressure_edit.setPlainText("2")
        self.ui.Process_time_edit.setPlainText("1")
        self.ui.Shutter_delay_edit.setPlainText("1")
        self.ui.Ar_flow_edit.setPlainText("20")
        self.ui.O2_flow_edit.setPlainText("0")
        self.ui.N2_flow_edit.setPlainText("0")
        self.ui.DC_power_edit.setPlainText("100")
        self.ui.RF_power_edit.setPlainText("100")
        self.ui.RF_pulse_checkbox.setChecked(False)
        self.ui.RF_pulse_edit.setPlainText("100")      # 예: 기본 100W
        self.ui.Rf_pulse_freq_edit.setPlainText("")    # 빈칸 = 변경하지 않음
        self.ui.Rf_pulse_duty_cycle_edit.setPlainText("")  # 빈칸 = 변경하지 않음

    def _reset_ui_after_process(self):
        self._set_default_ui_values()
        for cb in [
            self.ui.G1_checkbox, self.ui.G2_checkbox, self.ui.G3_checkbox,
            self.ui.Ar_checkbox, self.ui.O2_checkbox, self.ui.N2_checkbox,
            self.ui.Main_shutter_checkbox, self.ui.RF_power_checkbox, self.ui.RF_pulse_checkbox,
            self.ui.DC_power_checkbox
        ]:
            cb.setChecked(False)
        self.ui.process_state.setPlainText("대기 중")
        self.ui.Power_edit.setPlainText("0.00")
        self.ui.Voltage_edit.setPlainText("0.00")
        self.ui.Current_edit.setPlainText("0.00")
        self.ui.For_p_edit.setPlainText("0.0")
        self.ui.Ref_p_edit.setPlainText("0.0")
        # RF Pulse 표시 리셋
        self.ui.RF_pulse_edit.setPlainText("0")
        self.ui.Rf_pulse_freq_edit.setPlainText("")
        self.ui.Rf_pulse_duty_cycle_edit.setPlainText("")

    def closeEvent(self, event):
        self.append_log("main", "프로그램 종료 절차 시작...")

        try:
            if getattr(self.process_controller, "is_running", False):
                self.process_controller.abort_process()
                for _ in range(6):
                    QCoreApplication.processEvents()
                    QThread.msleep(20)
        except Exception:
            pass

        # 장치 cleanup은 각각 워커에서 BlockingQueued로 실행
        try: self.request_faduino_cleanup.emit()
        except Exception: pass
        try: self.request_mfc_cleanup.emit()
        except Exception: pass
        try: self.request_ig_cleanup.emit()
        except Exception: pass
        try: self.request_oes_cleanup.emit()
        except Exception: pass
        try: self.request_rfpulse_cleanup.emit()
        except Exception: pass

        threads = [
            getattr(self, 'faduino_thread', None),
            getattr(self, 'mfc_thread', None),
            getattr(self, 'oes_thread', None),
            getattr(self, 'ig_thread', None),
            getattr(self, 'rga_thread', None),
            getattr(self, 'data_logger_thread', None),
            getattr(self, 'rf_pulse_thread', None),  # ← 추가
        ]
        for thread in threads:
            if thread and thread.isRunning():
                thread.quit()
                thread.wait()
                self.append_log("MAIN", f"{thread.objectName() or type(thread).__name__} 종료 완료.")
        self.append_log("MAIN", "모든 스레드 종료. 프로그램을 닫습니다.")
        super().closeEvent(event)

    @Slot()
    def _on_about_to_quit(self):
        if self._about_quit_called:
            return
        self._about_quit_called = True

        try:
            if getattr(self.process_controller, "is_running", False):
                self.process_controller.abort_process()
                for _ in range(6):
                    QCoreApplication.processEvents()
                    QThread.msleep(20)
        except Exception:
            pass

        # ✅ 워커 스레드 쪽에서 안전하게 정리
        try: self.request_faduino_cleanup.emit()
        except Exception: pass
        try: self.request_mfc_cleanup.emit()
        except Exception: pass
        try: self.request_ig_cleanup.emit()
        except Exception: pass
        try: self.request_oes_cleanup.emit()
        except Exception: pass
        try: self.request_rfpulse_cleanup.emit()
        except Exception: pass


    def _emergency_cleanup(self):
        if self._emergency_done:
            return
        self._emergency_done = True

        try:
            if getattr(self, "process_controller", None) and \
               getattr(self.process_controller, "is_running", False):
                self.process_controller.abort_process()
        except Exception:
            pass

        # ✅ 워커 스레드 쪽에서 안전하게 정리
        try: self.request_faduino_cleanup.emit()
        except Exception: pass
        try: self.request_mfc_cleanup.emit()
        except Exception: pass
        try: self.request_ig_cleanup.emit()
        except Exception: pass
        try: self.request_oes_cleanup.emit()
        except Exception: pass
        try: self.request_rfpulse_cleanup.emit()
        except Exception: pass


    def _normalize_params_for_process(self, raw: dict) -> dict:
        def tf(v):
            return str(v).strip().upper() in ("T","TRUE","1","Y","YES")

        def fget(key, default="0"):
            try:
                return float(str(raw.get(key, default)).strip())
            except Exception:
                return float(default)

        def iget(key, default="0"):
            try:
                return int(float(str(raw.get(key, default)).strip()))
            except Exception:
                return int(default)

        def iget_opt(key):
            s = str(raw.get(key, '')).strip()
            return int(float(s)) if s != '' else None

        # CSV 원본 타겟명 (헤더 고정)
        g1t = str(raw.get("G1 Target", "")).strip()
        g2t = str(raw.get("G2 Target", "")).strip()
        g3t = str(raw.get("G3 Target", "")).strip()

        out = {
            # --- 공통 수치 ---
            "base_pressure":     fget("base_pressure", "1e-5"),
            "working_pressure":  fget("working_pressure", "0"),
            "process_time":      fget("process_time", "0"),
            "shutter_delay":     fget("shutter_delay", "0"),
            "integration_time":  iget("integration_time", "60"),
            "dc_power":          fget("dc_power", "0"),
            "rf_power":          fget("rf_power", "0"),

            # --- RF Pulse (CSV 컬럼명을 내부 표준 키로 정규화) ---
            # 사용여부: CSV는 use_rf_pulse_power, 내부는 use_rf_pulse
            "use_rf_pulse":      tf(raw.get("use_rf_pulse_power", "F")),
            "rf_pulse_power":    fget("rf_pulse_power", "0"),
            "rf_pulse_freq":     iget_opt("rf_pulse_freq"),          # '' -> None
            "rf_pulse_duty":     iget_opt("rf_pulse_duty_cycle"),    # '' -> None

            # --- 파워 사용 여부 ---
            "use_rf_power":      tf(raw.get("use_rf_power", "F")),
            "use_dc_power":      tf(raw.get("use_dc_power", "F")),

            # --- 가스 사용 여부 ---
            "use_ar":            tf(raw.get("Ar", "F")),
            "use_o2":            tf(raw.get("O2", "F")),
            "use_n2":            tf(raw.get("N2", "F")),

            # --- 가스 유량 ---
            "ar_flow":           fget("Ar_flow", "0"),
            "o2_flow":           fget("O2_flow", "0"),
            "n2_flow":           fget("N2_flow", "0"),

            # --- 건 셔터 ---
            "use_g1":            tf(raw.get("gun1", "F")),
            "use_g2":            tf(raw.get("gun2", "F")),
            "use_g3":            tf(raw.get("gun3", "F")),

            # --- 메인 셔터 ---
            "use_ms":            tf(raw.get("main_shutter", "F")),

            # --- 메모(이름) ---
            "process_note":      raw.get("Process_name", raw.get("process_note", "")),

            # --- 내부 표준 키(하위 모듈/검증 로직용) ---
            "G1_target_name":    g1t,
            "G2_target_name":    g2t,
            "G3_target_name":    g3t,

            # --- 원본 CSV 키 그대로 보존(DataLogger 등에서 사용) ---
            "G1 Target":         g1t,
            "G2 Target":         g2t,
            "G3 Target":         g3t,

            
            # ★ 추가: Power_select 체크 플래그
            "use_power_select": tf(raw.get("power_select", "F")),
        }

        return out

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
