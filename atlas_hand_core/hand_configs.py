"""
hand_configs.py
핸드 리타겟팅 설정 추상화 및 구현체

새로운 로봇 핸드를 추가하려면:
  1. HandConfig를 상속하는 클래스를 구현 — 클래스 변수 선언만으로 완성
     - _MODEL_SUBDIR    : models/{subdir}/urdf 경로
     - _FINGERS         : {'left': [...], 'right': [...]}  FingerChain 리스트
     - _WRIST_LINK      : {'left': '<link>', 'right': '<link>'}
     - _COORD_TRANSFORM : 3×3 변환 행렬 (기본: 단위행렬)
     - _SCALE_FACTOR    : Human 포지션 스케일 (기본: 1.0)
     - _URDF_FILENAME   : URDF 파일명 포맷 (기본: '{hand_type}.urdf')
     - _SIDE_MAP        : side 포맷 키 (기본: {'left':'left', 'right':'right'})
     - _FIXED_JOINTS    : {'left': '<joint>', 'right': '<joint>'} (기본: 없음)
  2. CONFIG_REGISTRY에 등록
  3. ros2 run --ros-args -p robot_config:=<key> 로 선택

Human 손 인덱스 레이아웃 (23 포인트, 0~22)
──────────────────────────────────────────
 0 : wrist
 1 : thumb_cmc0   2 : thumb_cmc1   3 : thumb_mcp    4 : thumb_ip     5 : thumb_tip
 6 : index_mcp    7 : index_pip    8 : index_dip    9 : index_tip
10 : middle_mcp  11 : middle_pip  12 : middle_dip  13 : middle_tip
14 : ring_mcp    15 : ring_pip    16 : ring_dip    17 : ring_tip
18 : pinky0      19 : pinky_mcp   20 : pinky_pip   21 : pinky_dip   22 : pinky_tip
"""

import os
import numpy as np

from abc import ABC
from dataclasses import dataclass
from typing import ClassVar, Dict, List, Type, Union


def _get_package_share() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("atlas_hand")
    except Exception:
        return os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))


# ==============================================================================
# DATA CLASS
# ==============================================================================

@dataclass
class FingerChain:
    """Single finger: URDF link chain + corresponding MediaPipe indices.

    links  — root→tip link names (may contain {side} / {wrist} placeholders)
    human  — MediaPipe keypoint indices, same length as links
    """
    links: List[str]
    human: List[int]


# ==============================================================================
# BASE CLASS
# ==============================================================================

class HandConfig(ABC):
    """로봇 핸드 리타겟팅 설정 기반 클래스.

    서브클래스는 클래스 변수 선언만으로 동작을 완전히 정의할 수 있습니다.
    메서드 오버라이드는 필요하지 않습니다.

    필수
    ----
    _MODEL_SUBDIR  : models/{subdir}/urdf 경로
    _FINGERS       : {'left': [...], 'right': [...]}
    _WRIST_LINK    : {'left': '<link>', 'right': '<link>'}

    선택 (기본값 있음)
    ------------------
    _COORD_TRANSFORM : 3×3 변환 행렬     (기본: 단위행렬)
    _SCALE_FACTOR    : 스케일 팩터       (기본: 1.0)
    _URDF_FILENAME   : 파일명 포맷 문자열 (기본: '{hand_type}.urdf')
    _SIDE_MAP        : side 포맷 값      (기본: {'left':'left', 'right':'right'})
    _FIXED_JOINTS    : 고정 조인트 맵    (기본: {})
    _WRIST_JOINTS    : {'left': {조인트명: [x,y,z]}, 'right': {...}} — 빈 내부 dict = off
    """

    _MODEL_SUBDIR:    ClassVar[str]                                    = ''
    _FINGERS:         ClassVar[Dict[str, List[FingerChain]]]           = {'left': [], 'right': []}
    _WRIST_LINK:      ClassVar[Dict[str, str]]                         = {'left': 'world', 'right': 'world'}
    _COORD_TRANSFORM: ClassVar[np.ndarray]                             = np.eye(3, dtype=np.float32)
    _SCALE_FACTOR:    ClassVar[Union[float, List[float]]]              = 1.0
    _URDF_FILENAME:   ClassVar[str]                                    = '{hand_type}.urdf'
    _SIDE_MAP:        ClassVar[Dict[str, str]]                         = {'left': 'left', 'right': 'right'}
    _FIXED_JOINTS:    ClassVar[Dict[str, str]]                         = {}
    # 손목 직접 매핑: {'left': {조인트명: 회전축}, 'right': {...}} — 빈 내부 dict = off
    _WRIST_JOINTS:    ClassVar[Dict[str, Dict[str, List[float]]]]      = {'left': {}, 'right': {}}

    def __init__(self):
        share_dir = _get_package_share()
        self._urdf_dir = os.path.join(share_dir, "models", self._MODEL_SUBDIR, "urdf")

    def _get_urdf_path(self, hand_type: str) -> str:
        return os.path.join(self._urdf_dir, self._URDF_FILENAME.format(hand_type=hand_type))

    def _get_fingers(self, hand_type: str) -> List[FingerChain]:
        fmt = {'side': self._SIDE_MAP[hand_type], 'wrist': self._WRIST_LINK[hand_type]}
        return [
            FingerChain([l.format(**fmt) for l in f.links], f.human)
            for f in self._FINGERS[hand_type]
        ]

    def get_two_stage_config(self, hand_type: str):
        """2단계 최적화 설정 (stage1_dict, stage2_dict) 반환."""
        urdf_path = self._get_urdf_path(hand_type)
        fingers   = self._get_fingers(hand_type)

        stage1 = {
            'type': 'vector',
            'urdf_path': urdf_path,
            'target_origin_link_names':  [l for f in fingers for l in f.links[:-1]],
            'target_task_link_names':    [l for f in fingers for l in f.links[1:]],
            'target_link_human_indices': [
                [h for f in fingers for h in f.human[:-1]],
                [h for f in fingers for h in f.human[1:]],
            ],
            'low_pass_alpha': -1.0,
        }
        stage2 = {
            'type': 'position',
            'urdf_path': urdf_path,
            'target_link_names':         [f.links[-1] for f in fingers],
            'target_link_human_indices': [f.human[-1] for f in fingers],
            'low_pass_alpha': 0.8,
        }
        return stage1, stage2

    def get_coord_transform(self, _hand_type: str) -> np.ndarray:
        return self._COORD_TRANSFORM

    def get_scale_factor(self) -> Union[float, List[float]]:
        return self._SCALE_FACTOR

    def get_wrist_link_name(self, hand_type: str) -> str:
        return self._WRIST_LINK[hand_type]

    def get_fixed_joint_names(self, hand_type: str) -> List[str]:
        joint = self._FIXED_JOINTS.get(hand_type, '')
        return [joint] if joint else []

    def get_tf_coord_transform(self, hand_type: str) -> np.ndarray:
        return self.get_coord_transform(hand_type)


# ==============================================================================
# BASE HAND CONFIG
# ==============================================================================

class BaseHandConfig(HandConfig):
    """models/base/urdf/{hand_type}.urdf 기반 설정."""

    _MODEL_SUBDIR = 'base'
    _WRIST_LINK   = {'left': 'left_wrist', 'right': 'right_wrist'}

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


# ==============================================================================
# ROBOTIS HX5 CONFIG
# ==============================================================================

class RobotisHX5Config(HandConfig):
    """Robotis HX5 핸드 설정.

    - {side} : 'l' / 'r' 약어
    - {wrist} : wrist base 링크명
    """

    _MODEL_SUBDIR  = 'robotis'
    _URDF_FILENAME = 'hx5_d20_{hand_type}.urdf'
    _SIDE_MAP      = {'left': 'l',                 'right': 'r'}
    _WRIST_LINK    = {'left': 'hx5_d20_left_base', 'right': 'hx5_d20_right_base'}
    _COORD_TRANSFORM: ClassVar[np.ndarray] = np.array(
        [[0, 1, 0], [0, 0, 1], [1, 0, 0]], dtype=np.float32
    )
    _SCALE_FACTOR  = [1.2, 1.25, 1.25, 1.3, 1.4]

    _chains = [
        FingerChain(  # Thumb
            links=["{wrist}", "finger_{side}_link1", "finger_{side}_link2",
                   "finger_{side}_link3", "finger_{side}_link4", "finger_end_{side}_link1"],
            human=[0, 1, 2, 3, 4, 5],
        ),
        FingerChain(  # Index
            links=["{wrist}", "finger_{side}_link5", "finger_{side}_link7",
                   "finger_{side}_link8", "finger_end_{side}_link2"],
            human=[0, 6, 7, 8, 9],
        ),
        FingerChain(  # Middle
            links=["{wrist}", "finger_{side}_link9", "finger_{side}_link11",
                   "finger_{side}_link12", "finger_end_{side}_link3"],
            human=[0, 10, 11, 12, 13],
        ),
        FingerChain(  # Ring
            links=["{wrist}", "finger_{side}_link13", "finger_{side}_link15",
                   "finger_{side}_link16", "finger_end_{side}_link4"],
            human=[0, 14, 15, 16, 17],
        ),
        FingerChain(  # Pinky
            links=["{wrist}", "finger_{side}_link17", "finger_{side}_link19",
                   "finger_{side}_link20", "finger_end_{side}_link5"],
            human=[0, 19, 20, 21, 22],
        ),
    ]
    _FINGERS: ClassVar[Dict[str, List[FingerChain]]] = {'left': _chains, 'right': _chains}


# ==============================================================================
# ORCA HAND CONFIG
# ==============================================================================

class OrcaHandConfig(HandConfig):
    """OrcaHand v2 설정.

    left/right 링크명이 완전히 달라 _FINGERS에 좌우 별도 정의.
    Left hand는 TIP_OFFSET 링크 미지원 (WIP).
    """

    _MODEL_SUBDIR = 'orca'
    _WRIST_LINK   = {'right': 'R-Carpals_8d1f1041', 'left': 'L-Carpals_719fff8c'}
    _COORD_TRANSFORM: ClassVar[np.ndarray] = np.array(
        [[0, 0, -1], [1, 0, 0], [0, -1, 0]], dtype=np.float32
    )
    _SCALE_FACTOR = [0.95,1.03,1.06,1.06,1.05 ]
    _FIXED_JOINTS = {
        'right': 'R-Carpals_8d1f1041_to_TopTower-Model_4a80d30e',
        'left':  'L-Carpals_719fff8c_to_TopTower-Model_4a80d30e',
    }

    _WRIST_JOINTS    = {'left': {}, 'right': {"R-Carpals_8d1f1041_to_TopTower-Model_4a80d30e" : [-1,0,0]}}



    _FINGERS: ClassVar[Dict[str, List[FingerChain]]] = {
        'right': [
            FingerChain(  # Thumb
                links=["{wrist}", "T-TP-R_1c2b802d", "R-T-AP_a9723101",
                       "T-PP_68395e98", "T-DP_b7429e50", "T-TIP_OFFSET"],
                human=[0, 2, 2, 3, 4, 5],
            ),
            FingerChain(  # Index
                links=["{wrist}", "I-AP-R_d95d02d1", "I-PP_bacbd481",
                       "I-FingerTipAssembly_ec49c16c", "I-TIP_OFFSET"],
                human=[0, 6, 6, 7, 9],
            ),
            FingerChain(  # Middle
                links=["{wrist}", "M-AP_e04a96f2", "M-PP_08efa608",
                       "M-FingerTipAssembly_34afb748", "M-TIP_OFFSET"],
                human=[0, 10, 10, 11, 13],
            ),
            FingerChain(  # Ring
                links=["{wrist}", "M-AP_6ec59111", "M-PP_8660a1eb",
                       "M-FingerTipAssembly_424a8e75", "MR-TIP_OFFSET"],
                human=[0, 14, 14, 15, 17],
            ),
            FingerChain(  # Pinky
                links=["{wrist}", "P-AP_f5e42b61", "P-PP_1d411b9b",
                       "P-FingerTipAssembly_cd219176", "P-TIP_OFFSET"],
                human=[0, 19, 19, 20, 22],
            ),
        ],
        'left': [
            FingerChain(  # Thumb (no TIP_OFFSET yet)
                links=["{wrist}", "T-TP-L_92b8100b", "L-T-AP_58680c44",
                       "T-PP_ef067304", "T-DP_307db3cc"],
                human=[0, 2, 2, 3, 4],
            ),
            FingerChain(  # Index
                links=["{wrist}", "I-AP-L_57ce92f7", "I-PP_3df4f91d",
                       "I-FingerTipAssembly_ed91b18a"],
                human=[0, 6, 7, 8],
            ),
            FingerChain(  # Middle
                links=["{wrist}", "M-AP_e04a96f2", "M-PP_08efa608",
                       "M-FingerTipAssembly_34afb748"],
                human=[0, 10, 11, 12],
            ),
            FingerChain(  # Ring
                links=["{wrist}", "M-AP_6ec59111", "M-PP_8660a1eb",
                       "M-FingerTipAssembly_424a8e75"],
                human=[0, 14, 15, 16],
            ),
            FingerChain(  # Pinky
                links=["{wrist}", "P-AP_f5e42b61", "P-PP_1d411b9b",
                       "P-FingerTipAssembly_cd219176"],
                human=[0, 19, 20, 21],
            ),
        ],
    }


# ==============================================================================
# CONFIG REGISTRY
# ==============================================================================

CONFIG_REGISTRY: Dict[str, Type[HandConfig]] = {
    "robotis_hx5": RobotisHX5Config,
    "base":        BaseHandConfig,
    "orca_hand":   OrcaHandConfig,
}
