"""
atlas_hand_core/sources/meta_quest.py
Meta Quest 손 추적 UDP 입력 소스

Unity HandTrackingSender 패킷 구조:
  Header : [magic:2][version:1][timestamp:8] = 11 bytes
  Per hand: [tracked:1][joint_count:1][joints: count×29] bytes
  Per joint: [id:1][px:4][py:4][pz:4][rx:4][ry:4][rz:4][rw:4] = 29 bytes

자동 발견:
  Quest → MXFIND 브로드캐스트 → PC → MXHELLO:port 유니캐스트 응답
"""

import logging
import socket
import struct
import threading
from typing import Callable, Optional

import numpy as np

from atlas_hand_core.config import QUEST_DISC_PORT, QUEST_UDP_PORT
from atlas_hand_core.sources.base import HandInputSource

_log = logging.getLogger(__name__)

_MAGIC        = b'\x4D\x58'   # "MX"
_FIND_MAGIC   = b'MXFIND'
_HELLO_MAGIC  = b'MXHELLO'

# Quest joint ID → AGA 호환 관절명 (17개, AGA pinky_0 제외 순서와 동일)
_JOINT_ORDER = [
    "wrist",
    "thumb_cmc0", "thumb_cmc1", "thumb_mcp", "thumb_ip",
    "index_mcp",  "index_pip",  "index_dip",
    "middle_mcp", "middle_pip", "middle_dip",
    "ring_mcp",   "ring_pip",   "ring_dip",
    "pinky_mcp",  "pinky_pip",  "pinky_dip",
]

_QUEST_JOINT_IDS = {
    "wrist":      0,
    "thumb_cmc0": 2,  "thumb_cmc1": 3,  "thumb_mcp": 4,  "thumb_ip": 5,
    "index_mcp":  6,  "index_pip":  7,  "index_dip": 8,
    "middle_mcp": 9,  "middle_pip": 10, "middle_dip": 11,
    "ring_mcp":   12, "ring_pip":   13, "ring_dip":  14,
    "pinky_mcp":  16, "pinky_pip":  17, "pinky_dip": 18,
}


class MetaQuestSource(HandInputSource):
    """Meta Quest 손 추적 UDP 입력 소스.

    - UDP 수신 스레드 (raw binary 패킷)
    - MXFIND 자동 발견 응답 스레드
    - 햅틱 미지원 (send_haptic 항상 False)
    """

    def __init__(
        self,
        listen_ip: str = '0.0.0.0',
        udp_port: int = QUEST_UDP_PORT,
        disc_port: int = QUEST_DISC_PORT,
        on_left_quat:     Optional[Callable[[np.ndarray], None]] = None,
        on_right_quat:    Optional[Callable[[np.ndarray], None]] = None,
        on_status_change: Optional[Callable[[bool, bool], None]] = None,
        verbose: bool = False,
    ):
        self._listen_ip = listen_ip
        self._udp_port  = udp_port
        self._disc_port = disc_port

        self._on_left_quat     = on_left_quat
        self._on_right_quat    = on_right_quat
        self._on_status_change = on_status_change
        self._verbose = verbose

        self._left_connected  = False
        self._right_connected = False
        self._stop_event = threading.Event()
        self._sock: Optional[socket.socket] = None

    # ─────────────────────────────────────────

    def start(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._listen_ip, self._udp_port))
        self._sock.settimeout(1.0)

        threading.Thread(target=self._recv_loop,      daemon=True, name='QuestUDP').start()
        threading.Thread(target=self._discovery_loop, daemon=True, name='QuestDisc').start()

        _log.info(f'[Quest] UDP 수신 대기: {self._listen_ip}:{self._udp_port}')
        _log.info(f'[Quest] 자동 발견 대기: port {self._disc_port}')

    def stop(self) -> None:
        self._stop_event.set()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass

    # ─────────────────────────────────────────

    @property
    def left_connected(self) -> bool:
        return self._left_connected

    @property
    def right_connected(self) -> bool:
        return self._right_connected

    # ─────────────────────────────────────────

    def _set_status(self, left_ok: bool, right_ok: bool) -> None:
        changed = (left_ok != self._left_connected or right_ok != self._right_connected)
        self._left_connected  = left_ok
        self._right_connected = right_ok
        if changed and self._on_status_change:
            self._on_status_change(left_ok, right_ok)

    def _recv_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                data, _ = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break

            parsed = _parse_packet(data)
            if parsed is None:
                continue

            left  = parsed['left']
            right = parsed['right']

            left_ok  = left  is not None and left['tracked']
            right_ok = right is not None and right['tracked']
            self._set_status(left_ok, right_ok)

            if left_ok and self._on_left_quat:
                self._on_left_quat(_to_quats(left))
            if right_ok and self._on_right_quat:
                self._on_right_quat(_to_quats(right))

            if self._verbose:
                _log.debug(f'[Quest] packet  left={left_ok}  right={right_ok}')

    def _discovery_loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(('', self._disc_port))
        sock.settimeout(1.0)
        response = _HELLO_MAGIC + struct.pack('<H', self._udp_port)

        while not self._stop_event.is_set():
            try:
                data, addr = sock.recvfrom(64)
                if data.startswith(_FIND_MAGIC):
                    sock.sendto(response, addr)
                    _log.info(f'[Quest] Quest 발견 {addr[0]} → MXHELLO:{self._udp_port}')
            except socket.timeout:
                continue
            except Exception as e:
                _log.warning(f'[Quest] 발견 오류: {e}')
        sock.close()


# ── 패킷 파싱 (모듈 수준 함수) ───────────────────────────────────────────

def _parse_packet(data: bytes) -> Optional[dict]:
    if len(data) < 11 or data[0:2] != _MAGIC:
        return None

    timestamp = struct.unpack_from('<d', data, 3)[0]
    offset = 11
    result = {'timestamp': timestamp, 'left': None, 'right': None}

    for side in ('left', 'right'):
        if offset >= len(data):
            break
        tracked     = data[offset] == 1;  offset += 1
        joint_count = data[offset];        offset += 1

        if not tracked or joint_count == 0:
            result[side] = {'tracked': False, 'joints': []}
            continue

        joints = []
        for _ in range(joint_count):
            if offset + 29 > len(data):
                break
            jid = data[offset];  offset += 1
            offset += 12  # position (px, py, pz) — 미사용
            rx, ry, rz, rw = struct.unpack_from('<ffff', data, offset);  offset += 16
            joints.append({'id': jid, 'rot': (rx, ry, rz, rw)})

        result[side] = {'tracked': True, 'joints': joints}

    return result


def _to_quats(hand: dict) -> np.ndarray:
    """손 데이터 → 17×4 numpy array (AGA 관절 순서와 동일)."""
    joint_by_id = {j['id']: j for j in hand['joints']}
    rows = []
    for name in _JOINT_ORDER:
        jid = _QUEST_JOINT_IDS[name]
        j = joint_by_id.get(jid)
        if j:
            rx, ry, rz, rw = j['rot']
            mag = (rx**2 + ry**2 + rz**2 + rw**2) ** 0.5
            if mag > 1e-6:
                rx, ry, rz, rw = rx/mag, ry/mag, rz/mag, rw/mag
            else:
                rx, ry, rz, rw = 0.0, 0.0, 0.0, 1.0
        else:
            rx, ry, rz, rw = 0.0, 0.0, 0.0, 1.0
        rows.append([rx, ry, rz, rw])
    return np.array(rows, dtype=np.float32)
