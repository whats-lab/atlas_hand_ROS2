from typing import ClassVar, Dict, List

from ._base import FingerChain, HandConfig


class SchunkSVHConfig(HandConfig):
    """SCHUNK SVH — 5-finger, 9-DOF."""

    _MODEL_SUBDIR  = 'schunk_hand'
    _URDF_FILENAME = 'schunk_svh_hand_{hand_type}.urdf'
    _WRIST_LINK    = {'left': 'left_hand_e1', 'right': 'right_hand_e1'}
    _FINGERS: ClassVar[Dict[str, List[FingerChain]]] = {'left': [], 'right': []}
