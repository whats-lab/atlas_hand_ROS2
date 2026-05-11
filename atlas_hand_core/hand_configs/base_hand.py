from typing import ClassVar, Dict, List

from ._base import FingerChain, HandConfig


class BaseHandConfig(HandConfig):
    _MODEL_SUBDIR  = 'base_hand'
    _URDF_FILENAME = 'urdf/{hand_type}.urdf'
    _RVIZ_FILENAME = {'left': 'base_left.rviz', 'right': 'base_right.rviz'}
    _WRIST_LINK    = {'left': 'left_wrist', 'right': 'right_wrist'}

    _chains = [
        FingerChain(  # Thumb
            links=["{side}_wrist", "{side}_thumb_cmc0", "{side}_thumb_cmc1",
                   "{side}_thumb_mcp", "{side}_thumb_ip", "{side}_thumb_tip"],
            human=[0, 1, 2, 3, 4, 5],
        ),
        FingerChain(  # Index
            links=["{side}_wrist", "{side}_index_mcp", "{side}_index_pip",
                   "{side}_index_dip", "{side}_index_tip"],
            human=[0, 6, 7, 8, 9],
        ),
        FingerChain(  # Middle
            links=["{side}_wrist", "{side}_middle_mcp", "{side}_middle_pip",
                   "{side}_middle_dip", "{side}_middle_tip"],
            human=[0, 10, 11, 12, 13],
        ),
        FingerChain(  # Ring
            links=["{side}_wrist", "{side}_ring_mcp", "{side}_ring_pip",
                   "{side}_ring_dip", "{side}_ring_tip"],
            human=[0, 14, 15, 16, 17],
        ),
        FingerChain(  # Pinky
            links=["{side}_wrist", "{side}_pinky_cmc", "{side}_pinky_mcp",
                   "{side}_pinky_pip", "{side}_pinky_dip", "{side}_pinky_tip"],
            human=[0, 18, 19, 20, 21, 22],
        ),
    ]
    _FINGERS: ClassVar[Dict[str, List[FingerChain]]] = {'left': _chains, 'right': _chains}
