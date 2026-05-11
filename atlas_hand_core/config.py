"""
atlas_hand_core/config.py
하드웨어 / 알고리즘 설정 상수 (ROS2 비의존)
"""

# ── OSC 네트워크 (Atlas 글러브) ───────────────────────────────────────
OSC_LISTEN_IP            = "0.0.0.0"
OSC_SERVER_PORT          = 4040
OSC_CLIENT_PORT          = 4042
OSC_TARGET_IP            = "127.0.0.1"
HEARTBEAT_INTERVAL_SEC   = 1.0
HAPTIC_PING_INTERVAL_SEC = 2.0

# ── UDP 네트워크 (Meta Quest) ─────────────────────────────────────────
QUEST_UDP_PORT  = 9090
QUEST_DISC_PORT = 9001

# ── AGA 센서 데이터 포맷 ──────────────────────────────────────────────
AGA_JOINT_COUNT     = 18
AGA_RAW_FLOAT_COUNT = 72
AGA_SENSOR_COUNT    = 17
AGA_SKIP_JOINT      = 14   # pinky_0 — FK에서 제외

HAND_LEFT  = 1
HAND_RIGHT = 2

# ── 관절 인덱스 ───────────────────────────────────────────────────────
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
OSC_ADDR_LEFT_HAPT      = "/left/hapt/set"
OSC_ADDR_RIGHT_HAPT     = "/right/hapt/set"
OSC_MSG_TYPE_LEFT_HAPT  = "9"
OSC_MSG_TYPE_RIGHT_HAPT = "10"
AGA_FINGER_COUNT        = 5

# ── IK / 리타겟팅 파라미터 ────────────────────────────────────────────
HUBER_DELTA       = 0.025
NORM_DELTA        = 0.01
IK_MAX_TIME       = 0.02    # 두 단계 합산 IK 시간 예산 (초)
VECTOR_WEIGHT     = 1.0
POSITION_WEIGHT   = 4.0
CONTROL_TIMER_SEC = 0.02    # 제어 루프 주기 (50 Hz)


# ── AGA 파싱 유틸 ─────────────────────────────────────────────────────

def parse_aga_raw(raw_floats) -> "np.ndarray":
    """AGA 글러브 원시 데이터 (72 floats / 18×4) → 17×4 쿼터니언 배열.

    pinky_0 (AGA_SKIP_JOINT) 제거.
    """
    import numpy as np
    return np.delete(np.asarray(raw_floats, dtype=np.float32).reshape(18, 4), AGA_SKIP_JOINT, axis=0)
