from typing import ClassVar, Dict, List

import numpy as np

from ._base import FingerChain, HandConfig


class OrcaHandConfig(HandConfig):
    """OrcaHand v2 설정.

    left/right 링크명이 완전히 달라 _FINGERS에 좌우 별도 정의.
    Left hand는 TIP_OFFSET 링크 미지원 (WIP).
    """

    _MODEL_SUBDIR = "orca_hand"
    _URDF_FILENAME = "urdf/{hand_type}.urdf"
    _WRIST_LINK = {"right": "R-Carpals_8d1f1041", "left": "L-Carpals_719fff8c"}
    _COORD_TRANSFORM: ClassVar[np.ndarray] = np.array(
        [[0, 0, -1], [1, 0, 0], [0, -1, 0]], dtype=np.float32
    )
    _SCALE_FACTOR = [0.95, 1.03, 1.06, 1.06, 1.05]
    _FIXED_JOINTS = {
        "right": "R-Carpals_8d1f1041_to_TopTower-Model_4a80d30e",
        "left": "L-Carpals_719fff8c_to_TopTower-Model_4a80d30e",
    }
    _WRIST_JOINTS = {
        "left": {},
        "right": {"R-Carpals_8d1f1041_to_TopTower-Model_4a80d30e": [-1, 0, 0]},
    }

    _FINGERS: ClassVar[Dict[str, List[FingerChain]]] = {
        "right": [
            FingerChain(  # Thumb
                links=[
                    "{wrist}",
                    "T-TP-R_1c2b802d",
                    "R-T-AP_a9723101",
                    "T-PP_68395e98",
                    "T-DP_b7429e50",
                    "T-TIP_OFFSET",
                ],
                human=[0, 2, 2, 3, 4, 5],
            ),
            FingerChain(  # Index
                links=[
                    "{wrist}",
                    "I-AP-R_d95d02d1",
                    "I-PP_bacbd481",
                    "I-FingerTipAssembly_ec49c16c",
                    "I-TIP_OFFSET",
                ],
                human=[0, 6, 6, 7, 9],
            ),
            FingerChain(  # Middle
                links=[
                    "{wrist}",
                    "M-AP_e04a96f2",
                    "M-PP_08efa608",
                    "M-FingerTipAssembly_34afb748",
                    "M-TIP_OFFSET",
                ],
                human=[0, 10, 10, 11, 13],
            ),
            FingerChain(  # Ring
                links=[
                    "{wrist}",
                    "M-AP_6ec59111",
                    "M-PP_8660a1eb",
                    "M-FingerTipAssembly_424a8e75",
                    "MR-TIP_OFFSET",
                ],
                human=[0, 14, 14, 15, 17],
            ),
            FingerChain(  # Pinky
                links=[
                    "{wrist}",
                    "P-AP_f5e42b61",
                    "P-PP_1d411b9b",
                    "P-FingerTipAssembly_cd219176",
                    "P-TIP_OFFSET",
                ],
                human=[0, 19, 19, 20, 22],
            ),
        ],
        "left": [
            FingerChain(  # Thumb (no TIP_OFFSET yet)
                links=[
                    "{wrist}",
                    "T-TP-L_92b8100b",
                    "L-T-AP_58680c44",
                    "T-PP_ef067304",
                    "T-DP_307db3cc",
                ],
                human=[0, 2, 2, 3, 4],
            ),
            FingerChain(  # Index
                links=[
                    "{wrist}",
                    "I-AP-L_57ce92f7",
                    "I-PP_3df4f91d",
                    "I-FingerTipAssembly_ed91b18a",
                ],
                human=[0, 6, 7, 8],
            ),
            FingerChain(  # Middle
                links=[
                    "{wrist}",
                    "M-AP_e04a96f2",
                    "M-PP_08efa608",
                    "M-FingerTipAssembly_34afb748",
                ],
                human=[0, 10, 11, 12],
            ),
            FingerChain(  # Ring
                links=[
                    "{wrist}",
                    "M-AP_6ec59111",
                    "M-PP_8660a1eb",
                    "M-FingerTipAssembly_424a8e75",
                ],
                human=[0, 14, 15, 16],
            ),
            FingerChain(  # Pinky
                links=[
                    "{wrist}",
                    "P-AP_f5e42b61",
                    "P-PP_1d411b9b",
                    "P-FingerTipAssembly_cd219176",
                ],
                human=[0, 19, 20, 21],
            ),
        ],
    }
