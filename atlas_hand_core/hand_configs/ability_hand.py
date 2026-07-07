from typing import ClassVar, Dict, List

from ._base import FingerChain, HandConfig
import numpy as np


class AbilityHandConfig(HandConfig):
    """PSYONIC Ability Hand — 5-finger."""

    _MODEL_SUBDIR = "ability_hand"
    _URDF_FILENAME = "ability_hand_{hand_type}.urdf"
    _RVIZ_FILENAME = {"left": "ability_hand.rviz", "right": "ability_hand.rviz"}
    _WRIST_LINK = {"left": "base", "right": "base"}
    # q2 계열은 q1의 mimic — 제어 가능한 joint만 명시
    _TARGET_JOINT_NAMES = ["thumb_q1", "thumb_q2", "index_q1", "middle_q1", "ring_q1", "pinky_q1"]
 
    _COORD_TRANSFORM = np.array([
        [0,0,-1],
        [0,1,0],
        [1,0,0]
    ])
    _SCALE_FACTOR = [0.65, 1.03, 1.06, 1.06, 1.05]
    
    # 좌/우 링크명이 동일 → 체인 공용
    _chains = [
        FingerChain(  # Thumb
            links=[
                "{wrist}",
                "thumb_base",
                "thumb_L1",
                "thumb_L2",
                "thumb_tip",
            ],
            human=[0, 1, 2, 3, 4],
        ),
        FingerChain(  # Index
            links=[
                "{wrist}",
                "index_L1",
                "index_L2",
                "index_tip",
            ],
            human=[0, 6, 7, 8],
        ),
        FingerChain(  # Middle
            links=[
                "{wrist}",
                "middle_L1",
                "middle_L2",
                "middle_tip",
            ],
            human=[0, 10, 11, 12],
        ),
        FingerChain(  # Ring
            links=[
                "{wrist}",
                "ring_L1",
                "ring_L2",
                "ring_tip",
            ],
            human=[0, 14, 15, 16],
        ),
        FingerChain(  # Pinky
            links=[
                "{wrist}",
                "pinky_L1",
                "pinky_L2",
                "pinky_tip",
            ],
            human=[0, 19, 20, 21],
        ),
    ]
    _FINGERS: ClassVar[Dict[str, List[FingerChain]]] = {"left": _chains, "right": _chains}

