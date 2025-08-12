# RGA.py (외부 프로그램 실행 및 결과 분석 버전)

import csv
import numpy as np
import subprocess
from pathlib import Path
from PyQt6.QtCore import QObject, QProcess, QTimer, pyqtSignal as Signal, pyqtSlot as Slot
from lib.config import RGA_PROGRAM_PATH, RGA_CSV_PATH

class RGAController(QObject):
    # ProcessController로 공정 단계의 완료/실패를 알리는 신호
    scan_finished = Signal()
    scan_failed = Signal(str, str)
    
    # GraphController로 그래프 데이터를 보내는 신호
    rga_data_updated = Signal(object, object)
    
    # UI 로그용 신호
    status_message = Signal(str, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.csv_path = Path(RGA_CSV_PATH)

        # 마지막으로 처리한 타임스탬프를 저장
        self.last_processed_timestamp = None

        # QProcess의 신호를 내부 슬롯에 연결
        # self.process.finished.connect(self._on_process_finished)
        # self.process.errorOccurred.connect(self._on_process_error)

    @Slot()
    def execute_external_scan(self):
        """ProcessController로부터 요청을 받아 외부 RGA 프로그램을 실행합니다."""
        program_path = Path(RGA_PROGRAM_PATH)
        
        # 외부 프로그램이 존재하지 않으면 즉시 실패 처리
        if not Path(program_path).exists():
            error_msg = f"외부 프로그램을 찾을 수 없습니다: {program_path}"
            self.status_message.emit("RGA Ext.", error_msg)
            self.scan_failed.emit("EXTERNAL_RGA_SCAN", error_msg)
            return

        self.status_message.emit("RGA Ext.", f"외부 RGA 프로그램 실행 (2초 대기): {program_path}")
        
        try:
            # subprocess.run은 외부 프로그램이 끝날 때까지 이 '워커 스레드'를 차단합니다.
            # UI가 있는 '메인 스레드'는 차단하지 않으므로 UI 멈춤이 발생하지 않습니다.
            # 60초 타임아웃을 설정하여 프로그램이 무한정 멈추는 것을 방지합니다.
            result = subprocess.run(
                [str(program_path)],
                capture_output=True,
                text=True,
                check=False,
                timeout=60  # 60초 후 타임아웃
            )

            # --- 외부 프로그램 실행이 끝나면 여기부터 실행됩니다 ---
            if result.returncode == 0:
                self.status_message.emit("RGA Ext.", "외부 프로그램 정상 종료. CSV 파일 분석 시작.")
                # exe가 파일 쓰기를 완료할 시간을 벌어주기 위해 2초 대기
                QTimer.singleShot(2000, self._parse_csv_and_emit_data)
            else:
                # stderr에 오류 내용이 있을 경우 함께 로깅합니다.
                error_details = result.stderr.strip()
                error_msg = f"외부 프로그램 비정상 종료 (Code: {result.returncode}). {error_details}"
                self.status_message.emit("RGA Ext.", error_msg)
                self.scan_failed.emit("EXTERNAL_RGA_SCAN", error_msg)

        except subprocess.TimeoutExpired:
            error_msg = "외부 프로그램 실행 시간 초과 (60초)."
            self.status_message.emit("RGA Ext.", error_msg)
            self.scan_failed.emit("EXTERNAL_RGA_SCAN", error_msg)
        except Exception as e:
            error_msg = f"외부 프로그램 실행 중 예외 발생: {e}"
            self.status_message.emit("RGA Ext.", error_msg)
            self.scan_failed.emit("EXTERNAL_RGA_SCAN", error_msg)

    def _parse_csv_and_emit_data(self):
        """CSV 파일을 파싱하여 그래프용 데이터를 만들고 신호를 보냅니다."""
        try:
            if not self.csv_path or not self.csv_path.exists():
                raise FileNotFoundError(f"CSV 파일을 찾을 수 없습니다: {self.csv_path}")
            
            with open(self.csv_path, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)

                # 1. 헤더 행을 먼저 읽습니다.
                try:
                    header_row = next(reader)
                except StopIteration:
                    raise ValueError("CSV 파일이 비어 있습니다 (헤더 없음).")

                # 2. 마지막 데이터 행을 효율적으로 찾습니다.
                last_data_row = None
                for row in reader:
                    if row: # 빈 줄은 건너뜁니다.
                        last_data_row = row

                if last_data_row is None:
                    raise ValueError("CSV 파일에 데이터 행이 없습니다.")

                # 1. 마지막 줄의 타임스탬프를 읽어옵니다.
                current_timestamp = last_data_row[0]

                # 2. 이전에 처리한 타임스탬프와 동일하면, 새 데이터가 없는 것이므로 종료합니다.
                if current_timestamp == self.last_processed_timestamp:
                    self.status_message.emit("RGA Ext.", "새로운 데이터가 없습니다. 분석을 건너뜁니다.")
                    self.scan_finished.emit() # 새 데이터가 없어도 공정은 정상 진행
                    return
                
                # 3. 새로운 데이터이므로, 현재 타임스탬프를 기록합니다.
                self.last_processed_timestamp = current_timestamp
                self.status_message.emit("RGA Ext.", f"새 데이터 감지: {current_timestamp}")

                # x축 데이터 파싱 (헤더에서 'Mass_1', 'Mass_2'...)
                mass_axis = np.array([int(h.replace('Mass ', '')) for h in header_row[1:]])
                
                # y축 원시 데이터 파싱 (마지막 줄에서)
                pressures_in_torr = np.array([float(val) for val in last_data_row[1:]])

                # 4. 그래프 업데이트 신호 발생
                self.rga_data_updated.emit(mass_axis, pressures_in_torr)
                self.status_message.emit("RGA Ext.", "그래프 업데이트 완료.")
                
                # 5. ProcessController에 공정 단계 완료 신호 발생
                self.scan_finished.emit()

        except Exception as e:
            self.status_message.emit("RGA Ext.", f"CSV 분석 실패: {e}")
            self.scan_failed.emit("EXTERNAL_RGA_SCAN", str(e))