"""
hand_configs.py
핸드 리타겟팅 설정 추상화 및 구현체 

새로운 로봇 핸드를 추가하려면:
  1. HandConfig를 상속하는 클래스를 구현
  2. CONFIG_REGISTRY에 등록하
  3. ros2 run --ros-args -p robot_config:=<key> 로 선택

등록된 설정:
  - "robotis_hx5" : Robotis HX5 핸드 (hands/robotis/ 로컬 URDF)
  - "hand_rerun"  : left/right_hand_rerun.urdf (직접 URDF)
"""

import os
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Type

import numpy as np


# ==============================================================================
# ABSTRACT BASE
# ==============================================================================

class HandConfig(ABC):
    """로봇 핸드 리타겟팅 설정 추상 클래스.

    구현 필수 메서드
    ----------------
    get_urdf_path        : 로드 가능한 URDF 파일 경로 반환
    get_target_link_names: IK 타겟 링크 이름 목록 반환
    get_human_joint_indices: FK 출력 positions[] 인덱스 매핑 반환
    get_coord_transform  : Quest → 로봇 좌표계 3×3 변환 행렬 반환

    선택적 오버라이드
    -----------------
    get_scale_factor      : Human 포지션 스케일 (기본 1.0)
    get_optimizer_weights : IK 링크별 가중치 (기본 None = 균등)
    """

    @abstractmethod
    def get_urdf_path(self, hand_type: str) -> str:
        """처리 완료된 URDF 파일 경로를 반환합니다.

        Args:
            hand_type: 'left' 또는 'right'
        Returns:
            실제 로드 가능한 .urdf 파일 경로
        """

    @abstractmethod
    def get_target_link_names(self, hand_type: str) -> List[str]:
        """IK 최적화에 사용할 타겟 링크 이름 목록을 반환합니다.

        반환 길이는 get_human_joint_indices() 길이와 동일해야 합니다.

        Args:
            hand_type: 'left' 또는 'right'
        """

    @abstractmethod
    def get_human_joint_indices(self) -> List[int]:
        """HandSphericalFK.compute_positions() 출력 배열의 인덱스 매핑.

        Human positions[i] 와 target_link_names[j] 가 1:1 대응됩니다.

        Human 손 인덱스 레이아웃 (23 포인트, 0~22)
        ──────────────────────────────────────────
         0 : wrist
         1 : thumb_cmc0   2 : thumb_cmc1   3 : thumb_mcp    4 : thumb_ip     5 : thumb_tip
         6 : index_mcp    7 : index_pip    8 : index_dip    9 : index_tip
        10 : middle_mcp  11 : middle_pip  12 : middle_dip  13 : middle_tip
        14 : ring_mcp    15 : ring_pip    16 : ring_dip    17 : ring_tip
        18 : pinky0      19 : pinky_mcp   20 : pinky_pip   21 : pinky_dip   22 : pinky_tip
        """

    @abstractmethod
    def get_coord_transform(self, hand_type: str) -> np.ndarray:
        """Atlas 좌표계 → 로봇 URDF 좌표계 변환 행렬(3×3)을 반환"""

    def get_scale_factor(self) -> float:
        """Human 포지션에 적용할 스케일 팩터 (기본 1.0)."""
        return 1.0

    def get_optimizer_weights(self, _n_links: int) -> Optional[np.ndarray]:
        """IK 최적화 링크별 가중치. None이면 균등 가중치."""
        return None


# ==============================================================================
# ROBOTIS HX5 CONFIG
# ==============================================================================

class RobotisHX5Config(HandConfig):
    """Robotis HX5 핸드 설정.

    - URDF: hands/robotis/hx5_d20_{side}.urdf 
    - 링크 명명: finger_{side}_link* / finger_end_{side}_link*
    """

    _SIDE_MAP = {'left': 'l', 'right': 'r'}

    _TARGET_LINK_TEMPLATES: List[str] = [
        # Thumb (5 targets: human 1~5)
        "finger_{side}_link2",  "finger_{side}_link2",  "finger_{side}_link3",
        "finger_{side}_link4",  "finger_end_{side}_link1",
        # Index (4 targets: human 6~9)
        "finger_{side}_link6",  "finger_{side}_link7",  "finger_{side}_link8",
        "finger_end_{side}_link2",
        # Middle (4 targets: human 10~13)
        "finger_{side}_link10", "finger_{side}_link11", "finger_{side}_link12",
        "finger_end_{side}_link3",
        # Ring (4 targets: human 14~17)
        "finger_{side}_link14", "finger_{side}_link15", "finger_{side}_link16",
        "finger_end_{side}_link4",
        # Pinky (4 targets: human 19~22, 18 제외)
        "finger_{side}_link18", "finger_{side}_link19", "finger_{side}_link20",
        "finger_end_{side}_link5",
    ]

    _HUMAN_JOINT_INDICES: List[int] = [
        1, 2, 3, 4, 5,       # Thumb
        6, 7, 8, 9,           # Index
        10, 11, 12, 13,       # Middle
        14, 15, 16, 17,       # Ring
        19, 20, 21, 22,       # Pinky (18 제외)
    ]

    _LEFT_COORD_TRANSFORM = np.array([
        [0, 1, 0],
        [0, 0, 1],
        [1, 0, 0],
    ], dtype=np.float32)

    _RIGHT_COORD_TRANSFORM = np.array([
        [0, 1, 0],
        [0, 0, 1],
        [1, 0, 0],
    ], dtype=np.float32)

    def __init__(self):
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory("atlas_hand")
        self._urdf_dir = os.path.join(share_dir, "urdf", "hands")

    def get_urdf_path(self, hand_type: str) -> str:
        return os.path.join(self._urdf_dir, f"hx5_d20_{hand_type}.urdf")

    def get_target_link_names(self, hand_type: str) -> List[str]:
        side = self._SIDE_MAP.get(hand_type, 'l')
        return [t.format(side=side) for t in self._TARGET_LINK_TEMPLATES]

    def get_human_joint_indices(self) -> List[int]:
        return self._HUMAN_JOINT_INDICES

    def get_coord_transform(self, hand_type: str) -> np.ndarray:
        return (self._LEFT_COORD_TRANSFORM if hand_type == 'left'
                else self._RIGHT_COORD_TRANSFORM)

    def get_scale_factor(self) -> float:
        return 1.4

    def get_optimizer_weights(self, n_links: int) -> Optional[np.ndarray]:
        weights = np.ones(n_links)
        weights[[2, 3, 6, 7, 10, 11, 14, 15, 18, 19]] = 20.0
        weights[[4, 8, 12, 16, 20]] = 5.0
        return weights


# ==============================================================================
# HAND RERUN CONFIG
# ==============================================================================

class HandRerunConfig(HandConfig):
    """left_hand_rerun.urdf / right_hand_rerun.urdf 기반 설정.

    - URDF: atlas_hand 패키지의 urdf/{hand_type}_hand/urdf/{hand_type}_hand_rerun.urdf
    - 링크 명명: {hand_type}_{finger}_{joint}
      예) left_thumb_cmc0, left_index_mcp, right_pinky_dip
    
    """

    _TARGET_LINK_TEMPLATES: List[str] = [
        # Thumb (4 targets: human 1~4)
        "{side}_thumb_cmc0", "{side}_thumb_cmc1",
        "{side}_thumb_mcp",  "{side}_thumb_ip",
        # Index (3 targets: human 6~8)
        "{side}_index_mcp",  "{side}_index_pip",  "{side}_index_dip",
        # Middle (3 targets: human 10~12)
        "{side}_middle_mcp", "{side}_middle_pip", "{side}_middle_dip",
        # Ring (3 targets: human 14~16)
        "{side}_ring_mcp",   "{side}_ring_pip",   "{side}_ring_dip",
        # Pinky (3 targets: human 19~21)
        "{side}_pinky_mcp",  "{side}_pinky_pip",  "{side}_pinky_dip",
    ]

    _HUMAN_JOINT_INDICES: List[int] = [
        1, 2, 3, 4,       # Thumb  (cmc0, cmc1, mcp, ip)
        6, 7, 8,           # Index  (mcp, pip, dip)
        10, 11, 12,        # Middle (mcp, pip, dip)
        14, 15, 16,        # Ring   (mcp, pip, dip)
        19, 20, 21,        # Pinky  (mcp, pip, dip)
    ]

    
    _LEFT_COORD_TRANSFORM = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
    ], dtype=np.float32)

    _RIGHT_COORD_TRANSFORM = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
    ], dtype=np.float32)

    def __init__(self):
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory("atlas_hand")
        self._urdf_dir = os.path.join(share_dir, "urdf")

    def get_urdf_path(self, hand_type: str) -> str:
        return os.path.join(
            self._urdf_dir,
            f"{hand_type}_hand",
            "urdf",
            f"{hand_type}_hand_rerun.urdf",
        )

    def get_target_link_names(self, hand_type: str) -> List[str]:
        return [t.format(side=hand_type) for t in self._TARGET_LINK_TEMPLATES]

    def get_human_joint_indices(self) -> List[int]:
        return self._HUMAN_JOINT_INDICES

    def get_coord_transform(self, hand_type: str) -> np.ndarray:
        return (self._LEFT_COORD_TRANSFORM if hand_type == 'left'
                else self._RIGHT_COORD_TRANSFORM)

    def get_scale_factor(self) -> float:
        return 1.0


# ==============================================================================
# CONFIG REGISTRY
# ==============================================================================

CONFIG_REGISTRY: Dict[str, Type[HandConfig]] = {
    "robotis_hx5": RobotisHX5Config,
    "hand_rerun":  HandRerunConfig,
}
