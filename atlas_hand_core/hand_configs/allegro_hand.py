from typing import ClassVar, Dict, List

import numpy as np

from ._base import FingerChain, HandConfig


class AllegroHandConfig(HandConfig):
    """Allegro Hand (Wonik Robotics) — 4-finger, 16-DOF.

    손가락 배치: Index=link_8~11 / Middle=link_4~7 / Ring=link_0~3 /
    Thumb=link_12~15 (새끼손가락 없음).

    IK 기준(원점): base_link. base_link가 URDF root이자 원점이고 모든 손가락
    joint가 여기 직접 붙는 실제 손 중심이다. (`wrist` 링크는 palm 아래 z=-0.095
    로 떨어진 팔뚝 플랜지라 기준으로 부적합.)
    """

    _MODEL_SUBDIR  = "allegro_hand"
    _URDF_FILENAME = "allegro_hand_{hand_type}.urdf"
    _RVIZ_FILENAME = {"left": "allegro_hand.rviz", "right": "allegro_hand.rviz"}
    _WRIST_LINK    = {"left": "base_link", "right": "base_link"}

    _COORD_TRANSFORM: ClassVar[np.ndarray] = np.array(
        [[0, 1, 0], [0, 0, 1], [1, 0, 0]], dtype=np.float32
    )
    _SCALE_FACTOR = [0.8, 0.8, 0.8, 0.8]

    # 손가락당 로봇 링크 5구간 vs human 키포인트 4구간 → 영벡터를 팁(fixed) 구간으로
    # 밀어 모든 구동관절(joint_N)에 방향 목표를 부여한다.
    _chains = [
        FingerChain(  # Index (joint_8~11)
            links=[
                "{wrist}",
                "link_8.0",
                "link_9.0",
                "link_10.0",
                "link_11.0",
                "link_11.0_tip",
            ],
            human=[0, 6, 7, 8, 9, 9],
        ),
        FingerChain(  # Middle (joint_4~7)
            links=[
                "{wrist}",
                "link_4.0",
                "link_5.0",
                "link_6.0",
                "link_7.0",
                "link_7.0_tip",
            ],
            human=[0, 10, 11, 12, 13, 13],
        ),
        FingerChain(  # Ring (joint_0~3)
            links=[
                "{wrist}",
                "link_0.0",
                "link_1.0",
                "link_2.0",
                "link_3.0",
                "link_3.0_tip",
            ],
            human=[0, 14, 15, 16, 17, 17],
        ),
        FingerChain(  # Thumb (joint_12~15)
            links=[
                "{wrist}",
                "link_12.0",
                "link_13.0",
                "link_14.0",
                "link_15.0",
                "link_15.0_tip",
            ],
            human=[0, 1, 2, 3, 4, 5],
        ),
    ]
    _FINGERS: ClassVar[Dict[str, List[FingerChain]]] = {"left": _chains, "right": _chains}
