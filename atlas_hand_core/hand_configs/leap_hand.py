from typing import ClassVar, Dict, List

from ._base import FingerChain, HandConfig


class LeapHandConfig(HandConfig):
    """LEAP Hand (CMU) — 4-finger, 16-DOF."""

    _MODEL_SUBDIR = "leap_hand"
    _URDF_FILENAME = "leap_hand_{hand_type}.urdf"
    _WRIST_LINK = {"left": "leap_hand", "right": "leap_hand"}
    _FINGERS: ClassVar[Dict[str, List[FingerChain]]] = {"left": [], "right": []}
