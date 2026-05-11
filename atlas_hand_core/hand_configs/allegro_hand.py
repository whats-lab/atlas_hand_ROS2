from typing import ClassVar, Dict, List

from ._base import FingerChain, HandConfig


class AllegroHandConfig(HandConfig):
    """Allegro Hand (Wonik Robotics) — 4-finger, 16-DOF."""

    _MODEL_SUBDIR = "allegro_hand"
    _URDF_FILENAME = "allegro_hand_{hand_type}.urdf"
    _WRIST_LINK = {"left": "wrist", "right": "wrist"}
    _FINGERS: ClassVar[Dict[str, List[FingerChain]]] = {"left": [], "right": []}
