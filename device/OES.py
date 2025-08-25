# device/OES.py

import ctypes
import numpy as np
import csv
from datetime import datetime
from pathlib import Path
from PyQt6.QtCore import QObject, QTimer, pyqtSignal as Signal, pyqtSlot as Slot

from lib.config import OES_AVG_COUNT

class OESController(QObject):
    # 신호 정의
    status_message = Signal(str, str)
    oes_data_updated = Signal(object, object)
    oes_finished = Signal()
    oes_failed = Signal(str, str) # 측정 실패를 알리는 신호

    def __init__(self, parent=None):
        super().__init__(parent)
        self.dll_path = r"\\VanaM_NAS\VanaM_Sputter\OES\SDKs\DLL\x64\stdcall\SPdbUSBm.dll"

        # [신규] OES 데이터 저장 경로 지정
        self.save_directory = Path(r"\\VanaM_NAS\VanaM_Sputter\OES\CH2")
        # [신규] 프로그램 시작 시 저장 폴더가 없으면 자동으로 생성
        try:
            self.save_directory.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            self.status_message.emit("OES_CSV", f"저장 폴더 생성 실패: {e}")

        self.sp_dll = None
        self.is_running = False
        self.sChannel = -1
        self.wl_table = None

        self.acquisition_timer = QTimer(self)
        self.acquisition_timer.timeout.connect(self._acquire_data_slice)

        self.process_timer = QTimer(self)
        self.process_timer.setSingleShot(True)
        self.process_timer.timeout.connect(lambda: self._end_measurement(was_successful=True))
        
        # --- [수정됨] 와이드 포맷 저장을 위한 변수 ---
        self.measured_data_list = [] # 측정된 모든 intensity 데이터를 저장할 리스트
        self.start_time_str = ""

    def _safe_close_channel(self):
        """TEC OFF + 채널 클로즈(있으면) + sChannel 리셋."""
        try:
            if self.sp_dll and self.sChannel >= 0:
                try:
                    self.sp_dll.spSetTEC(0, self.sChannel)
                except Exception:
                    pass
                try:
                    self.sp_dll.spCloseGivenChannel(self.sChannel)
                except Exception:
                    pass
        finally:
            self.sChannel = -1

    def _end_measurement(self, was_successful: bool, reason: str = ""):
        """모든 측정 종료 시나리오(성공, 실패, 중단) 공통 처리"""
        if not self.is_running:
            return

        # 1) 타이머 정지
        try:
            self.acquisition_timer.stop()
            self.process_timer.stop()
        except Exception:
            pass

        # 2) 성공 시에만 CSV 저장
        if was_successful and self.measured_data_list:
            self._save_data_to_csv_wide()

        # 3) 장비 리소스 해제(안전)
        self._safe_close_channel()

        # 4) 상태 업데이트
        self.is_running = False

        # 5) 알림
        if was_successful:
            self.status_message.emit("OES", "측정 완료 및 장비 연결 종료")
            self.oes_finished.emit()
        else:
            self.status_message.emit("OES", f"측정 실패 ({reason}) 및 장비 연결 종료")
            self.oes_failed.emit("OES", reason)

    @Slot()
    def initialize_device(self):
        try:
            # 혹시 반쯤 열린 채널이 있으면 정리
            self._safe_close_channel()

            self.sp_dll = ctypes.CDLL(self.dll_path)
            self._setup_dll_functions()
            
            result, sChannel = self._test_all_channels()
            if result < 0:
                self.status_message.emit("OES", f"장비 검색 실패, 코드: {result}")
                return False
            
            self.sChannel = sChannel
            
            result = self.sp_dll.spSetupGivenChannel(self.sChannel)
            if result < 0:
                self.status_message.emit("OES", f"채널 설정 실패: {result}")
                self._safe_close_channel()
                return False

            result_model, model = self._get_model(self.sChannel)
            if result_model < 0:
                self.status_message.emit("OES", f"모델 조회 실패: {result_model}")
                self._safe_close_channel()
                return False

            result = self.sp_dll.spInitGivenChannel(model, self.sChannel)
            if result < 0:
                self.status_message.emit("OES", f"채널 초기화 실패: {result}")
                self._safe_close_channel()
                return False
            
            result, self.wl_table = self._get_wavelength_table(self.sChannel)
            if result < 0:
                self.status_message.emit("OES", "파장 테이블 로드 실패.")
                self._safe_close_channel()
                return False

            self.status_message.emit("OES", "초기화 성공")
            return True

        except Exception as e:
            self.status_message.emit("OES", f"초기화 중 예외 발생: {e}")
            self._safe_close_channel()
            return False

    @Slot(float, int)
    def run_measurement(self, duration_sec, integration_time_ms):
        if self.is_running or self.sChannel < 0:
            reason = "이미 실행 중" if self.is_running else "초기화 실패"
            self.status_message.emit("OES", f"오류: {reason}")
            self.oes_failed.emit("OES", reason)
            return
        
        self.status_message.emit("OES", f"{duration_sec/60:.1f}분 동안 측정을 시작합니다.")
        self.is_running = True
        
        # --- [수정됨] 측정 시작 시 데이터 리스트와 타임스탬프 초기화 ---
        self.measured_data_list = []
        self.start_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        self.sp_dll.spSetBaseLineCorrection(self.sChannel)
        self.sp_dll.spAutoDark(self.sChannel)
        self.sp_dll.spSetTrgEx(11, self.sChannel)
        self.sp_dll.spSetTEC(1, self.sChannel)
        self.sp_dll.spSetDblIntEx(float(integration_time_ms), self.sChannel)

        self.acquisition_timer.start(1000)
        self.process_timer.start(int(duration_sec * 1000))

    @Slot()
    def stop_measurement(self):
        """[수정] 이 함수는 이제 성공적인 종료만을 위해 _end_measurement를 호출합니다."""
        self._end_measurement(was_successful=True)

    @Slot()
    def cleanup(self):
        """abort/종료 시 안전 정리"""
        if self.is_running:
            reason = "사용자 요청으로 중단됨"
            self.status_message.emit("OES", reason)
            self._end_measurement(was_successful=False, reason=reason)
        else:
            # 실행 중이 아니어도 열린 채널이 있을 수 있으니 정리
            self._safe_close_channel()
            self.status_message.emit("OES", "중단 요청 수신됨 (실행 중이 아님)")

    def _acquire_data_slice(self):
        if not self.is_running or self.sChannel < 0:
            return

        try:
            # 평균용 누적 버퍼 + 유효 프레임 카운트
            intensity_sum = np.zeros(3680, dtype=float)
            valid = 0

            for _ in range(OES_AVG_COUNT):
                res, temp_intensity = self._read_data_ex(self.sChannel)
                if res > 0 and temp_intensity is not None:
                    arr = np.asarray(temp_intensity, dtype=float)
                    if arr.size >= 1034:  # 최소 길이 가드
                        intensity_sum += arr
                        valid += 1

            # 이번 틱에서 유효 프레임이 하나도 없으면 그냥 스킵(연속 오류는 타이머 틱마다 자연히 재시도)
            if valid == 0:
                return

            avg_intensity = intensity_sum / float(valid)

            # 파장 테이블 유효성 확인 후 구간 슬라이스
            if self.wl_table is None or len(self.wl_table) < 1034:
                raise RuntimeError("파장 테이블이 유효하지 않습니다.")
            x_data = self.wl_table[10:1034]
            y_data = avg_intensity[10:1034]

            # 와이드 포맷 CSV 저장을 위한 한 줄 추가
            current_time = datetime.now().strftime("%H:%M:%S")
            self.measured_data_list.append([current_time] + y_data.tolist())

            # 그래프 업데이트
            self.oes_data_updated.emit(x_data, y_data)

        except Exception as e:
            reason = f"데이터 수집 중 오류: {e}"
            self.status_message.emit("OES", reason)
            self._end_measurement(was_successful=False, reason=reason)

    def _save_data_to_csv_wide(self):
        """[신규] 측정된 모든 OES 데이터를 와이드 포맷 CSV 파일로 저장하는 함수"""
        if not self.measured_data_list:
            self.status_message.emit("OES_CSV", "저장할 데이터가 없습니다.")
            return

        filename = f"OES_Data_{self.start_time_str}.csv"
        full_path = self.save_directory / filename
        try:
            # 헤더 생성: ["Time", wavelength_1, wavelength_2, ...]
            header = ["Time"] + self.wl_table[10:1034].tolist()

            with open(full_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(self.measured_data_list)
            
            self.status_message.emit("OES_CSV", f"측정 결과가 '{full_path}' 파일로 저장되었습니다.")
        except Exception as e:
            self.status_message.emit("OES_CSV", f"CSV 파일 저장 실패: {e}")

    # --- DLL 함수 래퍼 및 시그니처 설정 ---
    def _setup_dll_functions(self):
        self.sp_dll.spTestAllChannels.argtypes = [ctypes.POINTER(ctypes.c_int16)]
        self.sp_dll.spTestAllChannels.restype = ctypes.c_int16
        self.sp_dll.spSetupGivenChannel.argtypes = [ctypes.c_int16]
        self.sp_dll.spSetupGivenChannel.restype = ctypes.c_int16
        self.sp_dll.spGetModel.argtypes = [ctypes.c_int16, ctypes.POINTER(ctypes.c_int16)]
        self.sp_dll.spGetModel.restype = ctypes.c_int16
        self.sp_dll.spInitGivenChannel.argtypes = [ctypes.c_int16, ctypes.c_int32]
        self.sp_dll.spInitGivenChannel.restype = ctypes.c_int16
        self.sp_dll.spGetWLTable.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int16]
        self.sp_dll.spGetWLTable.restype = ctypes.c_int16
        self.sp_dll.spSetBaseLineCorrection.argtypes = [ctypes.c_int16]
        self.sp_dll.spSetBaseLineCorrection.restype = ctypes.c_int16
        self.sp_dll.spAutoDark.argtypes = [ctypes.c_int16]
        self.sp_dll.spAutoDark.restype = ctypes.c_int16
        self.sp_dll.spGetDevIsNew.argtypes = [ctypes.c_int16]
        self.sp_dll.spGetDevIsNew.restype = ctypes.c_int16
        self.sp_dll.spSetTrgEx.argtypes = [ctypes.c_int16, ctypes.c_int16]
        self.sp_dll.spSetTrgEx.restype = ctypes.c_int16
        self.sp_dll.spSetTEC.argtypes = [ctypes.c_int32, ctypes.c_int16]
        self.sp_dll.spSetTEC.restype = ctypes.c_int16
        self.sp_dll.spSetDblIntEx.argtypes = [ctypes.c_double, ctypes.c_int16]
        self.sp_dll.spSetDblIntEx.restype = ctypes.c_int16
        self.sp_dll.spReadDataEx.argtypes = [ctypes.POINTER(ctypes.c_int32), ctypes.c_int16]
        self.sp_dll.spReadDataEx.restype = ctypes.c_int16
        self.sp_dll.spCloseGivenChannel.argtypes = [ctypes.c_uint16]
        self.sp_dll.spCloseGivenChannel.restype = ctypes.c_int16
    
    def _test_all_channels(self):
        sOrderType = ctypes.c_int16()
        result = self.sp_dll.spTestAllChannels(ctypes.byref(sOrderType))
        return result, sOrderType.value

    def _get_model(self, sChannel):
        model = ctypes.c_int16()
        result = self.sp_dll.spGetModel(sChannel, ctypes.byref(model))
        return result, model.value

    def _get_wavelength_table(self, channel):
        dWLTable = (ctypes.c_double * 3680)()
        result = self.sp_dll.spGetWLTable(dWLTable, channel)
        return result, np.array(dWLTable)
    
    def _read_data_ex(self, sChannel):
        temp_intensity = (ctypes.c_int32 * 3680)()
        result = self.sp_dll.spReadDataEx(temp_intensity, sChannel)
        return result, np.array(temp_intensity)
