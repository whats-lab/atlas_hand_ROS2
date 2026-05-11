"""
핸드 리타겟팅 설정 기반 클래스 및 공통 유틸리티

새로운 로봇 핸드를 추가하려면:
  1. HandConfig를 상속하는 클래스를 구현 — 클래스 변수 선언만으로 완성
     - _MODEL_SUBDIR    : models/{subdir} 경로
     - _FINGERS         : {'left': [...], 'right': [...]}  FingerChain 리스트
     - _WRIST_LINK      : {'left': '<link>', 'right': '<link>'}
     - _COORD_TRANSFORM : 3×3 변환 행렬 (기본: 단위행렬)
     - _SCALE_FACTOR    : Human 포지션 스케일 (기본: 1.0)
     - _URDF_FILENAME   : URDF 파일명 포맷 (기본: '{hand_type}.urdf', 서브디렉토리 포함 가능)
     - _SIDE_MAP        : side 포맷 키 (기본: {'left':'left', 'right':'right'})
     - _FIXED_JOINTS    : {'left': '<joint>', 'right': '<joint>'} (기본: 없음)
  2. hand_configs/__init__.py 의 CONFIG_REGISTRY에 등록

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
from abc import ABC
from dataclasses import dataclass
from typing import ClassVar, Dict, List, Union

import numpy as np


def _get_package_share() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("atlas_hand")
    except Exception:
        return os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..'))


@dataclass
class FingerChain:
    """Single finger: URDF link chain + corresponding MediaPipe indices.

    links  — root→tip link names (may contain {side} / {wrist} placeholders)
    human  — MediaPipe keypoint indices, same length as links
    """
    links: List[str]
    human: List[int]


class HandConfig(ABC):
    """로봇 핸드 리타겟팅 설정 기반 클래스.

    서브클래스는 클래스 변수 선언만으로 동작을 완전히 정의할 수 있습니다.
    메서드 오버라이드는 필요하지 않습니다.

    필수
    ----
    _MODEL_SUBDIR  : models/{subdir} 경로
    _FINGERS       : {'left': [...], 'right': [...]}
    _WRIST_LINK    : {'left': '<link>', 'right': '<link>'}

    선택 (기본값 있음)
    ------------------
    _COORD_TRANSFORM : 3×3 변환 행렬     (기본: 단위행렬)
    _SCALE_FACTOR    : 스케일 팩터       (기본: 1.0)
    _URDF_FILENAME   : 파일명 포맷 문자열 (기본: '{hand_type}.urdf', 서브디렉토리 포함 가능)
    _SIDE_MAP        : side 포맷 값      (기본: {'left':'left', 'right':'right'})
    _FIXED_JOINTS    : 고정 조인트 맵    (기본: {})
    _WRIST_JOINTS    : {'left': {조인트명: [x,y,z]}, 'right': {...}} — 빈 내부 dict = off
    _RVIZ_FILENAME   : RViz 설정 파일명  (기본: {}, hand_view.launch.py에서 사용)
    _TARGET_JOINT_NAMES : mimic joint 포함 URDF용 제어 joint 명시 (기본: [] = 자동)
    """

    _MODEL_SUBDIR:         ClassVar[str]                               = ''
    _FINGERS:              ClassVar[Dict[str, List[FingerChain]]]      = {'left': [], 'right': []}
    _WRIST_LINK:           ClassVar[Dict[str, str]]                    = {'left': 'world', 'right': 'world'}
    _COORD_TRANSFORM:      ClassVar[np.ndarray]                        = np.eye(3, dtype=np.float32)
    _SCALE_FACTOR:         ClassVar[Union[float, List[float]]]         = 1.0
    _URDF_FILENAME:        ClassVar[str]                               = '{hand_type}.urdf'
    _SIDE_MAP:             ClassVar[Dict[str, str]]                    = {'left': 'left', 'right': 'right'}
    _FIXED_JOINTS:         ClassVar[Dict[str, str]]                    = {}
    _WRIST_JOINTS:         ClassVar[Dict[str, Dict[str, List[float]]]] = {'left': {}, 'right': {}}
    _RVIZ_FILENAME:        ClassVar[Dict[str, str]]                    = {}
    # mimic joint가 있는 URDF에서 실제 제어 가능한 joint만 명시. 비어있으면 자동 탐색.
    _TARGET_JOINT_NAMES:   ClassVar[List[str]]                         = []

    def __init__(self):
        share_dir = _get_package_share()
        self._urdf_dir = os.path.join(share_dir, "models", self._MODEL_SUBDIR)

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
            'low_pass_alpha': -1.0,
        }
        if self._TARGET_JOINT_NAMES:
            stage1['target_joint_names'] = self._TARGET_JOINT_NAMES
            stage2['target_joint_names'] = self._TARGET_JOINT_NAMES
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
