"""
hand_configs.py
핸드 리타겟팅 설정 추상화 및 구현체

새로운 로봇 핸드를 추가하려면:
  1. HandConfig를 상속하는 클래스를 구현
  2. CONFIG_REGISTRY에 등록
  3. ros2 run --ros-args -p robot_config:=<key> 로 선택

등록된 설정:
  - "robotis_hx5" : Robotis HX5 핸드 (hands/robotis/ 로컬 URDF)
  - "base"  : models/base/urdf/left|right.urdf (base hand URDF)
  - "orca_hand"   : OrcaHand v2 (DexPilot optimizer)

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

from abc import ABC, abstractmethod
from typing import Dict, List, Type
from ament_index_python.packages import get_package_share_directory




# ==============================================================================
# ABSTRACT BASE
# ==============================================================================

class HandConfig(ABC):
    """로봇 핸드 리타겟팅 설정 추상 클래스.

    구현 필수 메서드
    ----------------
    get_coord_transform         : Quest → 로봇 좌표계 3×3 변환 행렬

    선택적 오버라이드
    -----------------
    get_scale_factor            : Human 포지션 스케일 (기본 1.0)
    get_fixed_joint_names       : IK 후 0으로 고정할 조인트 (기본 [])
    """

    @abstractmethod
    def get_coord_transform(self, hand_type: str) -> np.ndarray:
        """Atlas 좌표계 → 로봇 URDF 좌표계 변환 행렬(3×3)."""

    def get_scale_factor(self) -> float:
        """Human 포지션에 적용할 스케일 팩터 (기본 1.0)."""
        return 1.0

    def get_fixed_joint_names(self, _hand_type: str) -> List[str]:
        """IK 결과에서 0으로 강제할 조인트 이름 목록."""
        return []

    def get_two_stage_config(self, _hand_type: str):
        """2단계 최적화 설정 (stage1_dict, stage2_dict) 반환.

        stage1: VectorOptimizer — 전체 손 방향 정규화 (low_pass_alpha=-1 로 LP 비활성)
        stage2: PositionOptimizer — 손끝 위치 정밀 보정 (LP filter 적용)
        """
        return None

    def get_wrist_link_name(self, _hand_type: str) -> str:
        """TF parent frame으로 사용할 로봇 wrist/base 링크 이름."""
        return 'world'

    def get_tf_coord_transform(self, hand_type: str) -> np.ndarray:
        """TF 시각화 전용 좌표 변환 (기본값: IK 변환과 동일)."""
        return self.get_coord_transform(hand_type)


# ==============================================================================
# ROBOTIS HX5 CONFIG
# ==============================================================================

class RobotisHX5Config(HandConfig):
    """Robotis HX5 핸드 설정."""

    _SIDE_MAP = {'left': 'l', 'right': 'r'}

    _WRIST_LINK: Dict[str, str] = {
        'left':  'hx5_d20_left_base',
        'right': 'hx5_d20_right_base',
    }
    # Stage 1: per-finger bone chain (root→tip); f[:-1]=origins, f[1:]=tasks
    # {wrist} is filled at runtime with _WRIST_LINK[hand_type]
    _FINGER_BONES: List[List[str]] = [
        ["{wrist}",  "finger_{side}_link1", "finger_{side}_link2",  "finger_{side}_link3",  "finger_{side}_link4",  "finger_end_{side}_link1"],  # Thumb
        ["{wrist}", "finger_{side}_link5",  "finger_{side}_link7",  "finger_{side}_link8",  "finger_end_{side}_link2"],  # Index
        ["{wrist}", "finger_{side}_link9", "finger_{side}_link11", "finger_{side}_link12", "finger_end_{side}_link3"],  # Middle
        ["{wrist}", "finger_{side}_link13", "finger_{side}_link15", "finger_{side}_link16", "finger_end_{side}_link4"],  # Ring
        ["{wrist}", "finger_{side}_link17", "finger_{side}_link19", "finger_{side}_link20", "finger_end_{side}_link5"],  # Pinky
    ]
    _FINGER_HUMAN: List[List[int]] = [
        [0, 1, 2, 3, 4, 5],      # Thumb
        [0, 6, 7, 8, 9],      # Index
        [0, 10, 11, 12, 13],  # Middle
        [0, 14, 15, 16, 17],  # Ring
        [0, 19, 20, 21, 22],  # Pinky
    ]

    _LEFT_COORD_TRANSFORM = np.array([[0,1,0],[0,0,1],[1,0,0]], dtype=np.float32)
    _RIGHT_COORD_TRANSFORM = np.array([[0,1,0],[0,0,1],[1,0,0]], dtype=np.float32)

    def __init__(self):
        share_dir = get_package_share_directory("atlas_hand")
        self._urdf_dir = os.path.join(share_dir, "models", "robotis", "urdf")

    def get_coord_transform(self, hand_type: str) -> np.ndarray:
        return (self._LEFT_COORD_TRANSFORM if hand_type == 'left'
                else self._RIGHT_COORD_TRANSFORM)

    def get_scale_factor(self) -> float:
        return 1.35

    def get_wrist_link_name(self, hand_type: str) -> str:
        return self._WRIST_LINK[hand_type]

    def get_two_stage_config(self, hand_type: str):
        side  = self._SIDE_MAP.get(hand_type, 'l')
        fmt   = {'side': side, 'wrist': self._WRIST_LINK[hand_type]}
        urdf_path = os.path.join(self._urdf_dir, f"hx5_d20_{hand_type}.urdf")

        stage1 = {
            'type': 'vector',
            'urdf_path': urdf_path,
            'target_origin_link_names': [t.format(**fmt) for f in self._FINGER_BONES for t in f[:-1]],
            'target_task_link_names':   [t.format(**fmt) for f in self._FINGER_BONES for t in f[1:]],
            'target_link_human_indices': [
                [h for f in self._FINGER_HUMAN for h in f[:-1]],
                [h for f in self._FINGER_HUMAN for h in f[1:]],
            ],
            'low_pass_alpha': -1.0,
        }
        stage2 = {
            'type': 'position',
            'urdf_path': urdf_path,
            'target_link_names':         [f[-1].format(**fmt) for f in self._FINGER_BONES],
            'target_link_human_indices': [f[-1] for f in self._FINGER_HUMAN],
            'low_pass_alpha': -1.0,
            
        }
        return stage1, stage2


# ==============================================================================
# Base HAND CONFIG
# ==============================================================================

class BaseHandConfig(HandConfig):
    """models/base/urdf/left.urdf / right.urdf 기반 설정."""

    # Stage 1: per-finger bone chain (root→tip); f[:-1]=origins, f[1:]=tasks
    _FINGER_BONES: List[List[str]] = [
        ["{side}_wrist", "{side}_thumb_cmc0", "{side}_thumb_cmc1", "{side}_thumb_mcp", "{side}_thumb_ip","{side}_thumb_tip"],   # Thumb
        ["{side}_wrist", "{side}_index_mcp",  "{side}_index_pip",  "{side}_index_dip","{side}_index_tip"],                      # Index
        ["{side}_wrist", "{side}_middle_mcp", "{side}_middle_pip", "{side}_middle_dip","{side}_middle_tip"],                     # Middle
        ["{side}_wrist", "{side}_ring_mcp",   "{side}_ring_pip",   "{side}_ring_dip","{side}_ring_tip"],                       # Ring
        ["{side}_wrist", "{side}_pinky_cmc",  "{side}_pinky_mcp",  "{side}_pinky_pip", "{side}_pinky_dip","{side}_pinky_tip"],  # Pinky
    ]
    _FINGER_HUMAN: List[List[int]] = [
        [0, 1, 2, 3, 4,5],      # Thumb
        [0, 6, 7, 8,9],         # Index
        [0, 10, 11, 12,13],      # Middle
        [0, 14, 15, 16,17],      # Ring
        [0, 18, 19, 20, 21,22],  # Pinky
    ]

    _LEFT_COORD_TRANSFORM  = np.eye(3, dtype=np.float32)
    _RIGHT_COORD_TRANSFORM = np.eye(3, dtype=np.float32)

    def __init__(self):
        share_dir = get_package_share_directory("atlas_hand")
        self._urdf_dir = os.path.join(share_dir, "models", "base", "urdf")

    def get_coord_transform(self, hand_type: str) -> np.ndarray:
        return np.eye(3, dtype=np.float32)

    def get_wrist_link_name(self, hand_type: str) -> str:
        return f'{hand_type}_wrist'

    def get_two_stage_config(self, hand_type: str):
        urdf_path = os.path.join(self._urdf_dir, f"{hand_type}.urdf")

        stage1 = {
            'type': 'vector',
            'urdf_path': urdf_path,
            'target_origin_link_names': [t.format(side=hand_type) for f in self._FINGER_BONES for t in f[:-1]],
            'target_task_link_names':   [t.format(side=hand_type) for f in self._FINGER_BONES for t in f[1:]],
            'target_link_human_indices': [
                [h for f in self._FINGER_HUMAN for h in f[:-1]],
                [h for f in self._FINGER_HUMAN for h in f[1:]],
            ],
            'low_pass_alpha': -1.0,
        }
        stage2 = {
            'type': 'position',
            'urdf_path': urdf_path,
            'target_link_names':         [f[-1].format(side=hand_type) for f in self._FINGER_BONES],
            'target_link_human_indices': [f[-1] for f in self._FINGER_HUMAN],
            'low_pass_alpha': -1.0,
        }
        return stage1, stage2


# ==============================================================================
# ORCA HAND CONFIG
# ==============================================================================

class OrcaHandConfig(HandConfig):
    """OrcaHand v2 설정 (DexPilot optimizer + 2단계 VectorOptimizer).

    - URDF: orca_hand/models/urdf/orcahand_{side}.urdf
    - Stage 1 체인: wrist → TP/AP → PP → tip (엄지 5단계, 나머지 4단계)
    """

    # Per-finger bone chain (wrist → TP/AP → PP → tip), per side
    _RIGHT_FINGER_BONES: List[List[str]] = [
        ["R-Carpals_8d1f1041", "T-TP-R_1c2b802d", "R-T-AP_a9723101", "T-PP_68395e98", "T-DP_b7429e50","T-TIP_OFFSET"],               # Thumb
        ["R-Carpals_8d1f1041", "I-AP-R_d95d02d1", "I-PP_bacbd481",   "I-FingerTipAssembly_ec49c16c", "I-TIP_OFFSET"],                  # Index
        ["R-Carpals_8d1f1041", "M-AP_e04a96f2",   "M-PP_08efa608",    "M-FingerTipAssembly_34afb748", "M-TIP_OFFSET"],                  # Middle
        ["R-Carpals_8d1f1041", "M-AP_6ec59111",   "M-PP_8660a1eb",    "M-FingerTipAssembly_424a8e75", "MR-TIP_OFFSET"],                  # Ring
        ["R-Carpals_8d1f1041", "P-AP_f5e42b61",   "P-PP_1d411b9b",    "P-FingerTipAssembly_cd219176", "P-TIP_OFFSET"],                  # Pinky
    ]
    _LEFT_FINGER_BONES: List[List[str]] = [
        ["L-Carpals_719fff8c", "T-TP-L_92b8100b", "L-T-AP_58680c44", "T-PP_ef067304", "T-DP_307db3cc"],                # Thumb
        ["L-Carpals_719fff8c", "I-AP-L_57ce92f7", "I-PP_3df4f91d",   "I-FingerTipAssembly_ed91b18a"],                  # Index
        # ["R-Carpals_8d1f1041", "I-AP-R_d95d02d1", "I-PP_bacbd481", "I-FingerTipAssembly_ec49c16c_to_I-PP_bacbd481",   "I-FingerTipAssembly_ec49c16c"],                  # Index
        
        ["L-Carpals_719fff8c", "M-AP_e04a96f2",   "M-PP_08efa608",   "M-FingerTipAssembly_34afb748"],                  # Middle
        ["L-Carpals_719fff8c", "M-AP_6ec59111",   "M-PP_8660a1eb",   "M-FingerTipAssembly_424a8e75"],                  # Ring
        ["L-Carpals_719fff8c", "P-AP_f5e42b61",   "P-PP_1d411b9b",   "P-FingerTipAssembly_cd219176"],                  # Pinky
    ]
    _FINGER_HUMAN: List[List[int]] = [
        [0,  2,  2,  3, 4,  5],  # Thumb:  wrist→TP(cmc0)→AP(cmc1)→PP(mcp)→DP(tip)
        [0,  6,  7, 8,9],      # Index:  wrist→AP(mcp)→PP(pip)→tip
        [0, 10, 11, 12,13],      # Middle
        [0, 14, 15, 16,17],      # Ring
        [0, 19, 20, 21,22],      # Pinky
    ]
    _FIXED_JOINTS: Dict[str, str] = {
        'right': 'R-Carpals_8d1f1041_to_TopTower-Model_4a80d30e',
        'left':  'L-Carpals_719fff8c_to_TopTower-Model_4a80d30e',
    }
    _COORD_TRANSFORM = np.array([
        [0,  0, -1],
        [1,  0,  0],
        [0, -1,  0],
    ], dtype=np.float32)


    def __init__(self):
        share_dir = get_package_share_directory("atlas_hand")
        self._urdf_dir = os.path.join(share_dir, "models", "orca", "urdf")

    def get_coord_transform(self, _hand_type: str) -> np.ndarray:
        return self._COORD_TRANSFORM

    def get_wrist_link_name(self, hand_type: str) -> str:
        bones = self._RIGHT_FINGER_BONES if hand_type == 'right' else self._LEFT_FINGER_BONES
        return bones[0][0]

    def get_scale_factor(self) -> float:
        return 1.05

    def get_fixed_joint_names(self, hand_type: str) -> List[str]:
        return [self._FIXED_JOINTS[hand_type]]

    def get_two_stage_config(self, hand_type: str):
        urdf_path = os.path.join(self._urdf_dir, f"{hand_type}.urdf")
        bones = self._RIGHT_FINGER_BONES if hand_type == 'right' else self._LEFT_FINGER_BONES

        stage1 = {
            'type': 'vector',
            'urdf_path': urdf_path,
            'target_origin_link_names': [b for f in bones for b in f[:-1]],
            'target_task_link_names':   [b for f in bones for b in f[1:]],
            'target_link_human_indices': [
                [h for f in self._FINGER_HUMAN for h in f[:-1]],
                [h for f in self._FINGER_HUMAN for h in f[1:]],
            ],
            'low_pass_alpha': -1.0,
            
        }
        stage2 = {
            'type': 'position',
            'urdf_path': urdf_path,
            'target_link_names':         [f[-1] for f in bones],
            'target_link_human_indices': [f[-1] for f in self._FINGER_HUMAN],
            'low_pass_alpha': -1.0,
            
        }
        return stage1, stage2


# ==============================================================================
# CONFIG REGISTRY
# ==============================================================================

CONFIG_REGISTRY: Dict[str, Type[HandConfig]] = {
    "robotis_hx5": RobotisHX5Config,
    "base":  BaseHandConfig,
    "orca_hand":   OrcaHandConfig,
}
