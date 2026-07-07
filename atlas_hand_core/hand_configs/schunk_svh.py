from typing import ClassVar, Dict, List
import numpy as np

from ._base import FingerChain, HandConfig


class SchunkSVHConfig(HandConfig):
    """SCHUNK SVH — 5-finger, 9-DOF (mimic 다수).

    링크/조인트 접두어: {side}_hand_ (left/right). tip 링크(thtip/fftip/…)는 공용.
    IK 기준(원점): {side}_hand_e1 (손 베이스).

    mimic 조인트가 많아 실제 제어 가능한 9개 joint만 _TARGET_JOINT_NAMES로 명시한다
    (좌우 접두어가 달라 dict). 나머지(j3/j4/j12~j17, *_spread 등)는 이들의 mimic이다.

    """

    _MODEL_SUBDIR  = "schunk_hand"
    _URDF_FILENAME = "schunk_svh_hand_{hand_type}.urdf"
    _RVIZ_FILENAME = {"left": "schunk_hand.rviz", "right": "schunk_hand.rviz"}
    _WRIST_LINK    = {"left": "left_hand_e1", "right": "right_hand_e1"}
    _COORD_TRANSFORM: ClassVar[np.ndarray] = np.array(
        [[0, 0, -1], [0, 1,0], [1, 0, 0]], dtype=np.float32
    )
    _TARGET_JOINT_NAMES = {
        side: [
            f"{side}_hand_Thumb_Flexion",
            f"{side}_hand_Thumb_Opposition",
            f"{side}_hand_Index_Finger_Proximal",
            f"{side}_hand_Index_Finger_Distal",
            f"{side}_hand_Middle_Finger_Proximal",
            f"{side}_hand_Middle_Finger_Distal",
            f"{side}_hand_Ring_Finger",
            f"{side}_hand_Pinky",
            f"{side}_hand_Finger_Spread",
        ]
        for side in ("left", "right")
    }

    # 손가락당 로봇 링크 구간 > human 키포인트 → 영벡터를 팁(fixed)/spread(abduction,
    # 최소 중요) 구간으로 밀어 실제 굴곡관절에 방향 목표를 부여한다.
    _chains = [
        FingerChain(  # Thumb: e1 → z(Opp) → a(Flex) → b → c → thtip
            links=["{wrist}", "{side}_hand_z", "{side}_hand_a",
                   "{side}_hand_b", "{side}_hand_c", "thtip"],
            human=[0, 1, 2, 3, 4, 5],
        ),
        FingerChain(  # Index: e1 → virtual_l(spread) → l(Prox) → p(Dist) → t → fftip
            links=["{wrist}", "{side}_hand_virtual_l", "{side}_hand_l",
                   "{side}_hand_p", "{side}_hand_t", "fftip"],
            human=[0, 6, 7, 8, 9, 9],
        ),
        FingerChain(  # Middle: e1 → virtual_k → k(Prox) → o(Dist) → s → mftip
            links=["{wrist}", "{side}_hand_virtual_k", "{side}_hand_k",
                   "{side}_hand_o", "{side}_hand_s", "mftip"],
            human=[0, 10, 11, 12, 13, 13],
        ),
        FingerChain(  # Ring: e1 → e2 → virtual_j(spread) → j(Ring) → n → r → rftip
            links=["{wrist}", "{side}_hand_e2", "{side}_hand_virtual_j",
                   "{side}_hand_j", "{side}_hand_n", "{side}_hand_r", "rftip"],
            human=[0, 14, 14, 15, 16, 17, 17],
        ),
        FingerChain(  # Pinky: e1 → e2 → virtual_i(spread) → i(Pinky) → m → q → lftip
            links=["{wrist}", "{side}_hand_e2", "{side}_hand_virtual_i",
                   "{side}_hand_i", "{side}_hand_m", "{side}_hand_q", "lftip"],
            human=[0, 19, 19, 20, 21, 22, 22],
        ),
    ]
    _FINGERS: ClassVar[Dict[str, List[FingerChain]]] = {"left": _chains, "right": _chains}
