"""
config.py
Atlas Hand 전체 파이프라인 설정 상수

"""

# ── OSC 네트워크 ──────────────────────────────────────────────────────
OSC_LISTEN_IP          = "0.0.0.0"
OSC_SERVER_PORT        = 4040
OSC_CLIENT_PORT        = 4042
OSC_TARGET_IP          = "127.0.0.1"
HEARTBEAT_INTERVAL_SEC   = 1.0
HAPTIC_PING_INTERVAL_SEC = 2.0

# ── AGA 센서 데이터 포맷 ──────────────────────────────────────────────
AGA_JOINT_COUNT     = 18   
AGA_RAW_FLOAT_COUNT = 72   
AGA_SENSOR_COUNT    = 17   
AGA_SKIP_JOINT      = 14   

HAND_LEFT  = 1
HAND_RIGHT = 2

# ── 관절 인덱스 ────────────────────────────────────────
AGA_WRIST      = 0
AGA_THUMB_CMC0 = 1
AGA_THUMB_CMC1 = 2
AGA_THUMB_MCP  = 3
AGA_THUMB_IP   = 4
AGA_INDEX_MCP  = 5
AGA_INDEX_PIP  = 6
AGA_INDEX_DIP  = 7
AGA_MIDDLE_MCP = 8
AGA_MIDDLE_PIP = 9
AGA_MIDDLE_DIP = 10
AGA_RING_MCP   = 11
AGA_RING_PIP   = 12
AGA_RING_DIP   = 13
AGA_PINKY_0    = 14
AGA_PINKY_MCP  = 15
AGA_PINKY_PIP  = 16
AGA_PINKY_DIP  = 17

AGA_JOINT_NAMES = [
    "wrist",
    "thumb_cmc0", "thumb_cmc1", "thumb_mcp", "thumb_ip",
    "index_mcp",  "index_pip",  "index_dip",
    "middle_mcp", "middle_pip", "middle_dip",
    "ring_mcp",   "ring_pip",   "ring_dip",
    "pinky_0",    "pinky_mcp",  "pinky_pip",  "pinky_dip",
]

# ── OSC 햅틱 피드백 송신 ──────────────────────────────────────────────
OSC_ADDR_LEFT_HAPT      = "/left/hapt/set"   # 왼손 햅틱 명령 전송 주소
OSC_ADDR_RIGHT_HAPT     = "/right/hapt/set"  # 오른손 햅틱 명령 전송 주소
OSC_MSG_TYPE_LEFT_HAPT  = "9"                # 왼손 햅틱 메시지 타입 식별자
OSC_MSG_TYPE_RIGHT_HAPT = "10"               # 오른손 햅틱 메시지 타입 식별자
AGA_FINGER_COUNT        = 5                  # 햅틱 액추에이터 수 (thumb~pinky, 인덱스 0~4)

# ── ROS 2 토픽 ────────────────────────────────────────────────────────
TOPIC_LEFT_QUAT        = "/left_hand/quaternions"
TOPIC_RIGHT_QUAT       = "/right_hand/quaternions"
TOPIC_LEFT_HAPTIC      = "/left_hand/haptic"      # 구독: 햅틱 on 명령 (Int32MultiArray ×4)
TOPIC_RIGHT_HAPTIC     = "/right_hand/haptic"     # 구독: 햅틱 on 명령 (Int32MultiArray ×4)
TOPIC_LEFT_HAPTIC_OFF  = "/left_hand/haptic/off"  # 구독: 왼손 햅틱 끄기 트리거 (Empty)
TOPIC_RIGHT_HAPTIC_OFF = "/right_hand/haptic/off" # 구독: 오른손 햅틱 끄기 트리거 (Empty)
TOPIC_LEFT_CONNECTED   = "/left_hand/connected"   # 발행: 왼손 연결 상태 (Bool)
TOPIC_RIGHT_CONNECTED  = "/right_hand/connected"  # 발행: 오른손 연결 상태 (Bool)
