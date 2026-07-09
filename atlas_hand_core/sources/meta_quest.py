"""
atlas_hand_core/sources/meta_quest.py
Meta Quest 손 추적 OSC 입력 소스 (questTracker 통합본)

신 Quest APK 가 OSC 로 보내는 데이터를 수신·파싱하고, HandInputSource 규약
(17×4 쿼터니언 push)에 맞춰 노드에 전달한다. 파싱부(questTracker)와 소스 어댑터를
한 파일로 통합했다.

파싱 계층(questTracker):
    OscReceiver          (전송)    — OSC 서버 + 자동 발견(MXFIND→MXHELLO)
    └ PoseTracker        (pos/rot) — get_pos / get_rot / connected
       ├ ControllerTracker         — /controller/<side>/pos·rot
       └ HandTracker               — /hand/<side>/pos·rot  +  joints/pos·rot
소스 어댑터:
    MetaQuestSource(HandInputSource) — HandTracker 를 감싸 17×4 콜백으로 push

OSC 주소 규칙(APK ↔ 이 모듈, 좌표는 모두 HMD 로컬):
  /controller/<left|right>/pos     float x,y,z
  /controller/<left|right>/rot     float x,y,z,w
  /hand/<left|right>/pos           float x,y,z            (손목 root)
  /hand/<left|right>/rot           float x,y,z,w
  /hand/<left|right>/joints/pos    float[16*3=48]
  /hand/<left|right>/joints/rot    float[16*4=64]
"""
from __future__ import annotations

import logging
import socket
import struct
import threading
import time
from typing import Callable, Dict, Optional

import numpy as np

from atlas_hand_core.config import QUEST_DISC_PORT, QUEST_OSC_PORT
from atlas_hand_core.sources.base import HandInputSource

_log = logging.getLogger(__name__)

_FIND_MAGIC = b"MXFIND"
_HELLO_MAGIC = b"MXHELLO"
# joints 16관절 순서 (APK 송신 순서)
JOINT_ORDER = [
    "thumb_cmc0", "thumb_cmc1", "thumb_mcp", "thumb_ip",
    "index_mcp", "index_pip", "index_dip",
    "middle_mcp", "middle_pip", "middle_dip",
    "ring_mcp", "ring_pip", "ring_dip",
    "pinky_mcp", "pinky_pip", "pinky_dip",
]
NUM_JOINTS = 16
SIDES = ("left", "right")


# ================================================================ 공용 유틸
def _side_of(args):
    """dispatcher 고정인자(side) + OSC 값 분리. python-osc 버전차로 side 가
    문자열 또는 [문자열] 로 올 수 있어 양쪽 모두 처리."""
    side = args[0]
    if isinstance(side, (list, tuple)):
        side = side[0]
    return side, args[1:]


def _norm_quat(xyzw) -> np.ndarray:
    q = np.array(xyzw[:4], dtype=float)
    n = np.linalg.norm(q)
    return q / n if n > 1e-6 else np.array([0.0, 0.0, 0.0, 1.0])


def _identity_quat() -> np.ndarray:
    return np.array([0.0, 0.0, 0.0, 1.0])


# ======================================================= 부모: OSC 서버 + 발견
class OscReceiver:
    """OSC UDP 서버 생명주기 + 자동 발견(MXFIND→MXHELLO)을 담당하는 부모 클래스.

    자식은 _register(dispatcher) 를 구현해 수신할 OSC 주소를 핸들러에 연결한다.
    전송(서버/발견)은 이 클래스가, 파싱은 자식이 담당한다.
    """

    def __init__(
        self,
        listen_ip: str = "0.0.0.0",
        osc_port: int = QUEST_OSC_PORT,
        disc_port: int = QUEST_DISC_PORT,   # 0이면 발견 비활성
    ):
        self._listen_ip = listen_ip
        self._osc_port = osc_port
        self._disc_port = disc_port
        self._stop_event = threading.Event()
        self._server = None
        self._started = False

    def _register(self, dispatcher) -> None:
        """자식 구현: dispatcher.map(...) 으로 수신할 OSC 주소를 연결."""
        raise NotImplementedError

    def start(self) -> None:
        if self._started:  # idempotent
            return
        from pythonosc.dispatcher import Dispatcher
        from pythonosc.osc_server import ThreadingOSCUDPServer

        disp = Dispatcher()
        self._register(disp)

        self._server = ThreadingOSCUDPServer((self._listen_ip, self._osc_port), disp)
        threading.Thread(target=self._server.serve_forever, daemon=True, name="QuestOSC").start()
        _log.info("[%s] OSC 수신 대기: %s:%d",
                  type(self).__name__, self._listen_ip, self._osc_port)

        if self._disc_port:
            threading.Thread(target=self._discovery_loop, daemon=True, name="QuestDisc").start()
            _log.info("[%s] 자동 발견 대기: port %d", type(self).__name__, self._disc_port)
        self._started = True

    def stop(self) -> None:
        self._stop_event.set()
        if self._server is not None:
            try:
                self._server.shutdown()
            except Exception:  # noqa: BLE001
                pass
            self._server = None
        self._started = False

    def _discovery_loop(self) -> None:
        """MXFIND 브로드캐스트 수신 → MXHELLO+osc_port 응답. stop_event 로 종료."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(("", self._disc_port))
        sock.settimeout(1.0)
        response = _HELLO_MAGIC + struct.pack("<H", self._osc_port)
        discovered = False
        while not self._stop_event.is_set():
            try:
                data, addr = sock.recvfrom(64)
                if data.startswith(_FIND_MAGIC):
                    sock.sendto(response, addr)   # 매번 응답(킵얼라이브)
                    if not discovered:
                        discovered = True
                        _log.info("[%s] 발견됨 %s → MXHELLO:%d",
                                  type(self).__name__, addr[0], self._osc_port)
            except socket.timeout:
                continue
            except OSError:
                break
        sock.close()


# ================================================== 중간 부모: pos/rot 처리
class PoseTracker(OscReceiver):
    """<PREFIX>/<side>/pos·rot (root pose) 를 좌·우로 저장·제공.

    컨트롤러와 핸드 root 는 접두어(PREFIX)만 다르고 값 형식이 같아 여기서 공유한다.
    자식은 PREFIX 만 지정하면 되고, 필요하면 _register 를 확장한다(HandTracker 의 joints).
    """

    PREFIX = ""   # 자식에서 지정 ("/controller" | "/hand")

    def __init__(self, listen_ip="0.0.0.0", osc_port=QUEST_OSC_PORT,
                 disc_port=QUEST_DISC_PORT, stale_timeout=0.0):
        super().__init__(listen_ip, osc_port, disc_port)
        self._stale_timeout = stale_timeout
        self._lock = threading.Lock()
        self._pos: Dict[str, np.ndarray] = {s: np.zeros(3) for s in SIDES}
        self._rot: Dict[str, np.ndarray] = {s: _identity_quat() for s in SIDES}
        self._update: Dict[str, float] = {s: 0.0 for s in SIDES}

    def _register(self, dispatcher) -> None:
        for side in SIDES:
            dispatcher.map(f"{self.PREFIX}/{side}/pos", self._on_pos, side)
            dispatcher.map(f"{self.PREFIX}/{side}/rot", self._on_rot, side)

    # -------------------------------------------------------------- accessors
    def get_pos(self, side: str) -> np.ndarray:
        """side("left"/"right") 최신 위치 (x,y,z)."""
        with self._lock:
            return self._pos[side].copy()

    def get_rot(self, side: str) -> np.ndarray:
        """side("left"/"right") 최신 회전 (x,y,z,w)."""
        with self._lock:
            return self._rot[side].copy()

    def connected(self, side: str) -> bool:
        """패킷 수신 여부(있다면). stale_timeout>0 이면 그 시간 내 수신만 True."""
        with self._lock:
            t = self._update[side]
        if t == 0.0:
            return False
        if self._stale_timeout > 0 and time.monotonic() - t > self._stale_timeout:
            return False
        return True

    # ----------------------------------------------------------- OSC handlers
    def _on_pos(self, address, *args):
        side, v = _side_of(args)
        if len(v) < 3:
            return
        with self._lock:
            self._pos[side] = np.array(v[:3], dtype=float)
            self._update[side] = time.monotonic()

    def _on_rot(self, address, *args):
        side, v = _side_of(args)
        if len(v) < 4:
            return
        with self._lock:
            self._rot[side] = _norm_quat(v[:4])
            self._update[side] = time.monotonic()


# ============================================================ 컨트롤러 트래커
class ControllerTracker(PoseTracker):
    """컨트롤러 pose(/controller/<side>/pos·rot) 트래킹."""

    PREFIX = "/controller"


# ================================================================ 핸드 트래커
class HandTracker(PoseTracker):
    """핸드 트래킹: root pose(/hand/<side>/pos·rot) + 16관절(joints/pos·rot).

    root pose 는 부모(PoseTracker)의 get_pos/get_rot 로, 16관절은
    get_joint_pos/get_joint_rot 로 접근한다.
    """

    PREFIX = "/hand"

    def __init__(self, listen_ip="0.0.0.0", osc_port=QUEST_OSC_PORT,
                 disc_port=QUEST_DISC_PORT, stale_timeout=0.0, verbose=False,
                 on_update: Optional[Callable[[str], None]] = None):
        super().__init__(listen_ip, osc_port, disc_port, stale_timeout)
        self._verbose = verbose
        # on_update(side): 한 손의 joints 프레임 수신 완료 시 호출(push 소비자용, 예: ROS).
        self._on_update = on_update
        self._joint_pos: Dict[str, np.ndarray] = {s: np.zeros((NUM_JOINTS, 3)) for s in SIDES}
        self._joint_rot: Dict[str, np.ndarray] = {
            s: np.tile(_identity_quat(), (NUM_JOINTS, 1)) for s in SIDES
        }

    def _register(self, dispatcher) -> None:
        super()._register(dispatcher)   # /hand/<side>/pos·rot
        for side in SIDES:
            dispatcher.map(f"/hand/{side}/joints/pos", self._on_joints_pos, side)
            dispatcher.map(f"/hand/{side}/joints/rot", self._on_joints_rot, side)

    # -------------------------------------------------------------- accessors
    def get_joint_pos(self, side: str) -> np.ndarray:
        """side 의 16관절 위치 (16,3). 순서 = JOINT_ORDER."""
        with self._lock:
            return self._joint_pos[side].copy()

    def get_joint_rot(self, side: str) -> np.ndarray:
        """side 의 16관절 회전 (16,4). 순서 = JOINT_ORDER."""
        with self._lock:
            return self._joint_rot[side].copy()

    # ----------------------------------------------------------- OSC handlers
    def _on_joints_pos(self, address, *args):
        side, v = _side_of(args)
        arr = np.asarray(v, dtype=float)
        if arr.size < NUM_JOINTS * 3:
            return
        with self._lock:
            self._joint_pos[side] = arr[: NUM_JOINTS * 3].reshape(NUM_JOINTS, 3)
            self._update[side] = time.monotonic()

    def _on_joints_rot(self, address, *args):
        side, v = _side_of(args)
        arr = np.asarray(v, dtype=float)
        if arr.size < NUM_JOINTS * 4:
            return
        rots = arr[: NUM_JOINTS * 4].reshape(NUM_JOINTS, 4)
        rots = rots / (np.linalg.norm(rots, axis=1, keepdims=True) + 1e-9)
        with self._lock:
            self._joint_rot[side] = rots
            self._update[side] = time.monotonic()
        if self._verbose:
            _log.debug("[HandTracker] %s joints rx", side)
        if self._on_update is not None:   # 락 밖에서 호출(소비자가 게터 재진입 가능)
            self._on_update(side)



_STALE_TIMEOUT_SEC = 0.5    # 이 시간 이상 패킷 없으면 disconnected
_WATCHDOG_PERIOD   = 0.2    # 연결 상태 감시 주기


# ============================================== 소스 어댑터 (HandInputSource)
class MetaQuestSource(HandInputSource):
    """Meta Quest 손 추적 OSC 입력 소스.

    - HandTracker 로 OSC 수신 + 자동 발견
    - joints 프레임 수신마다 on_update → 17×4 조립 → on_*_quat 콜백(push)
    - 워치독 스레드가 연결 상태(stale) 전이를 on_status_change 로 통지
    - 햅틱 미지원 (send_haptic 항상 False)

    17×4 레이아웃: row0=손목(wrist) 회전, row1..16=16관절 회전(JOINT_ORDER).
    """

    def __init__(
        self,
        listen_ip: str = "0.0.0.0",
        osc_port: int = QUEST_OSC_PORT,
        disc_port: int = QUEST_DISC_PORT,
        on_left_quat:     Optional[Callable[[np.ndarray], None]] = None,
        on_right_quat:    Optional[Callable[[np.ndarray], None]] = None,
        on_status_change: Optional[Callable[[bool, bool], None]] = None,
        verbose: bool = False,
    ):
        self._on_left_quat     = on_left_quat
        self._on_right_quat    = on_right_quat
        self._on_status_change = on_status_change

        self._tracker = HandTracker(
            listen_ip=listen_ip,
            osc_port=osc_port,
            disc_port=disc_port,
            stale_timeout=_STALE_TIMEOUT_SEC,
            verbose=verbose,
            on_update=self._on_hand_update,
        )

        self._left_connected  = False
        self._right_connected = False
        self._stop_event = threading.Event()

    # ─────────────────────────────────────────

    def start(self) -> None:
        self._tracker.start()
        threading.Thread(target=self._watchdog_loop, daemon=True,
                         name="QuestOscWatchdog").start()
        _log.info("[Quest] OSC 입력 소스 시작")

    def stop(self) -> None:
        self._stop_event.set()
        self._tracker.stop()

    # ─────────────────────────────────────────

    @property
    def left_connected(self) -> bool:
        return self._left_connected

    @property
    def right_connected(self) -> bool:
        return self._right_connected

    # ─────────────────────────────────────────

    def _quats_17x4(self, side: str) -> np.ndarray:
        """손목 회전 + 16관절 회전 → 17×4 (row0=wrist, row1..16=JOINT_ORDER)."""
        wrist = self._tracker.get_rot(side).reshape(1, 4)
        joints = self._tracker.get_joint_rot(side)          # (16, 4)
        return np.vstack([wrist, joints]).astype(np.float32)

    def _on_hand_update(self, side: str) -> None:
        """HandTracker joints 프레임 수신 콜백(락 밖). 17×4 조립 후 push."""
        if side == "left" and self._on_left_quat:
            self._on_left_quat(self._quats_17x4("left"))
        elif side == "right" and self._on_right_quat:
            self._on_right_quat(self._quats_17x4("right"))

    def _watchdog_loop(self) -> None:
        """연결 상태(stale) 전이 감시 → on_status_change 통지."""
        while not self._stop_event.wait(_WATCHDOG_PERIOD):
            left_ok  = self._tracker.connected("left")
            right_ok = self._tracker.connected("right")
            if left_ok != self._left_connected or right_ok != self._right_connected:
                self._left_connected  = left_ok
                self._right_connected = right_ok
                if self._on_status_change:
                    self._on_status_change(left_ok, right_ok)
