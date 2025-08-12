# lib/config.py
from pathlib import Path

# === 시리얼 포트 설정 ===
IG_PORT = "COM11"
IG_BAUD = 9600
IG_WAIT_TIMEOUT = 600  # IG 대기 최대 시간 (초). 예: 600초 = 10분

# IG 컨트롤러의 세부 타이머 설정 (초 단위)
MAX_READ_RETRIES = 3
RESPONSE_TIMEOUT_SEC = 5.0

# === RGA ===
# RGA_PORT = "COM17" RGA를 직접 연결하지 않고 외부 프로그램을 사용
# RGA_BAUD = 9600
RGA_PROGRAM_PATH = r"\\VanaM_NAS\VanaM_Sputter\Programs\RGA_Ch2.exe"
RGA_CSV_PATH = r"\\VanaM_NAS\VanaM_Sputter\RGA\Ch.2\RGA_spectrums.csv"
RGA_PROGRAM_PATH = Path(RGA_PROGRAM_PATH)
RGA_CSV_PATH = Path(RGA_CSV_PATH)

# DLL을 통해 연결
# OES_PORT = "COM5"
# OES_BAUD = 9600w

MFC_PORT = "COM12"
MFC_BAUD = 9600

FADUINO_PORT = "COM13"
FADUINO_BAUD = 9600

# === Faduino ===
BUTTON_TO_PIN = {
    "MV": 0,
    "Ar": 1,
    "O2": 2,
    "N2": 3,
    "MS": 4,
    "G1": 5,
    "G2": 6,
    "G3": 7,
}
# === Analog IO 공통 상수 ===
ADC_FULL_SCALE = 26666     # 14.7bit ≈ 0~26666 카운트
ADC_INPUT_VOLT = 10.0      # ADC 입력 범위 0~10 V
DAC_FULL_SCALE = 4095      # 12bit DAC 0~4095

# ======================================================================
# RF Power 보정 및 제어 설정
# ======================================================================
RF_MAX_POWER = 600      # 최대 설정 가능 파워 (W)
RF_RAMP_STEP = 0.5      # Ramp-up 시 한 스텝당 올릴 파워 (W)
RF_MAINTAIN_STEP = 0.1  # 파워 유지 시 미세조정할 파워 (W)
RF_TOLERANCE_POWER = 1  # 목표 파워 도달로 인정할 허용 오차 (± W)

# --- 보정 계수 (Calibration Coefficients) ---
# 1. 목표 파워(W) -> FADUINO DAC 값 변환용 (in RFpower.py)
# 수식: DAC = (RF_PARAM_WATT_TO_DAC * target_watt) + RF_OFFSET_WATT_TO_DAC
RF_PARAM_WATT_TO_DAC = 6.79   # 기울기 (param)
RF_OFFSET_WATT_TO_DAC = 6.93  # y절편 (offset)

# 2. FADUINO ADC 값 -> 실제 파워(W) 변환용 (in Faduino.py)
# 수식: Watts = (RF_PARAM_ADC_TO_WATT * adc_raw_value) + RF_OFFSET_ADC_TO_WATT
RF_PARAM_ADC_TO_WATT = 0.0236431  # 기울기 (param)
RF_OFFSET_ADC_TO_WATT = -1.3362   # y절편 (offset)

# RF Reflected: 전압(볼트) -> 와트 환산 상수 (필요시 조정)
RF_WATT_PER_VOLT = 63.49


# ======================================================================
# DC Power 보정 및 제어 설정
# ======================================================================
# --- 제어 로직 파라미터 ---
DC_MAX_POWER = 1000       # 최대 설정 가능 파워 (W)
DC_INTERVAL_MS = 1000     # 제어 루프 실행 간격 (milliseconds)
DC_RAMP_STEP = 5          # 파워를 올릴 때 DAC 값 조정 단계
DC_MAINTAIN_STEP = 1      # 파워를 유지할 때 DAC 값 미세 조정 단계
DC_TOLERANCE_POWER = 1    # 목표 파워 도달로 간주할 허용 오차 (W)

# --- 보정 계수 (Calibration Coefficients) ---
# 1. 목표 파워(W) -> FADUINO DAC 값 변환용 (in DCpower.py)
# 수식: DAC = (DC_PARAM_WATT_TO_DAC * target_watt) + DC_OFFSET_WATT_TO_DAC
DC_PARAM_WATT_TO_DAC = 4.0835   # 기울기 (param)
DC_OFFSET_WATT_TO_DAC = 5.275   # y절편 (offset)

# 2. FADUINO 전압 ADC 값 -> 실제 전압(V) 변환용 (in Faduino.py)
# 수식: Voltage = (DC_PARAM_ADC_TO_VOLT * adc_v_raw) + DC_OFFSET_ADC_TO_VOLT
DC_PARAM_ADC_TO_VOLT = 0.076112 # 전압 기울기 (param)
DC_OFFSET_ADC_TO_VOLT = -6.8453 # 전압 y절편 (offset)

# 3. FADUINO 전류 ADC 값 -> 실제 전류(A) 변환용 (in Faduino.py)
# 수식: Current = (DC_PARAM_ADC_TO_AMP * adc_c_raw) + DC_OFFSET_ADC_TO_AMP
DC_PARAM_ADC_TO_AMP = 0.000150567 # 전류 기울기 (param)
DC_OFFSET_ADC_TO_AMP = -0.003118  # 전류 y절편 (offset)

# === OES ===
OES_AVG_COUNT = 3 # OES 측정 시 평균을 낼 횟수

# === MFC ===
FLOW_ERROR_TOLERANCE = 0.05  # 5% 오차 허용
FLOW_ERROR_MAX_COUNT = 3     # 3회 연속 불일치 시 경고

# [신규] 채널별 유량 스케일 팩터 정의
MFC_SCALE_FACTORS = {
    1: 1.0,     # Channel 1 (Ar): 1:1 스케일
    2: 0.1,     # Channel 2 (O2): 1/10 스케일
    3: 0.1,     # Channel 3 (N2): 1/10 스케일
}

# 명령어는 ASCII 문자로 전송해야 되며, \r으로 끝나야 함.
MFC_COMMANDS = {
    # --- MFC 쓰기(Write) 명령어 ---
    # --- 채널 지정 명령어 (channel 인자 필요) ---
    'FLOW_ON': lambda channel: f"L{channel} 1",               # 지정된 채널의 Flow를 켭니다 (1=ON).
    'FLOW_OFF': lambda channel: f"L{channel} 0",              # 지정된 채널의 Flow를 끕니다 (0=OFF). 
    'MFC_ZEROING': lambda channel: f"L{4+channel} 1",         # 지정된 채널의 MFC를 Zeroing합니다 (Ch1=L5, Ch2=L6 ...). 
    'FLOW_SET': lambda channel, value: f"Q{channel} {value}", # 지정된 채널의 Flow 값을 설정합니다 (% of Full Scale). 

    # --- 공통 명령어 (채널 지정 불필요) ---
    'VALVE_OPEN': "O",  # Throttle Valve를 엽니다. 
    'VALVE_CLOSE': "C", # Throttle Valve를 닫습니다. 
    'PS_ZEROING': "Z1", # 압력 센서(게이지)를 Zeroing합니다. 
    'SP4_ON': "D4",     # Set-point 4를 실행합니다. 
    'SP1_ON': "D1",     # Set-point 1을 실행합니다. 
    'SP1_SET': lambda value: f"S1 {value}", # Set-point 1의 목표 압력 값을 설정합니다. 

    # --- MFC 읽기(Read) 명령어 ---
    # --- 채널 지정 명령어 (channel 인자 필요) ---
    'READ_FLOW': lambda channel: f"R6{channel}",        # 지정된 채널의 현재 유량을 읽습니다. 
    'READ_FLOW_SET': lambda channel: f"R6{4+channel}",  # 지정된 채널의 Flow 설정 값을 읽습니다 (Ch1=R65, Ch2=R66 ...). 

    # --- 공통 명령어 (채널 지정 불필요) ---
    'READ_PRESSURE': "R5",          # 현재 시스템 압력을 읽습니다. 
    'READ_SP1_VALUE': "R1",         # Set-point 1에 설정된 값을 읽습니다. 
    'READ_VALVE_POSITION': "R6",    # Throttle Valve의 현재 위치(%)를 읽습니다. 
    'READ_SYSTEM_STATUS': "R7",     # 현재 활성화된 Set-point 등 시스템 상태를 읽습니다. 
    'READ_MFC_ON_OFF_STATUS': "R69",# 모든 MFC 채널의 ON/OFF 상태를 한 번에 읽습니다. 
}

