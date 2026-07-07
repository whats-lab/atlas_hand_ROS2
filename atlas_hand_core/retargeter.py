"""
retargeter.py
Hand 리타겟팅 파이프라인 코어 (ROS2 비의존)

사용법:
    from atlas_hand_core.retargeter import HandRetargeter
    retargeter = HandRetargeter('right', 'orca_hand')
    joint_angles = retargeter.compute(sensor_quats_17)  # shape (17, 4)
"""

import logging
import sys
from typing import List, Union

import numpy as np
from scipy.spatial.transform import Rotation as ScipyR

logger = logging.getLogger(__name__)

try:
    from dex_retargeting.retargeting_config import RetargetingConfig
except ImportError as e:
    logger.error("dex_retargeting 없음: %s", e)
    sys.exit(1)

from atlas_hand_core.config import (
    HUBER_DELTA,
    IK_MAX_TIME,
    NORM_DELTA,
    POSITION_WEIGHT,
    VECTOR_WEIGHT,
)
from atlas_hand_core.hand_configs import CONFIG_REGISTRY, HandConfig
from atlas_hand_core.hand_spherical_fk import HandSphericalFK


class HandRetargeter:
    """AGA 센서 쿼터니언 → 로봇 조인트 각도 변환기.

    FK → 좌표 변환 → 스케일링 → 2단계 IK (vector + position) → 손목 직접 매핑
    """

    def __init__(
        self,
        hand_type: str,
        config_name: str = 'base_hand',
        vector_weight: float = VECTOR_WEIGHT,
        position_weight: float = POSITION_WEIGHT,
    ):
        if config_name not in CONFIG_REGISTRY:
            raise ValueError(
                f"Unknown robot_config '{config_name}'. Available: {list(CONFIG_REGISTRY.keys())}"
            )
        config = CONFIG_REGISTRY[config_name]()

        self.hand_type = hand_type.lower()
        self.fk        = HandSphericalFK(self.hand_type)

        self._coord_transform = config.get_coord_transform(self.hand_type)

        sf = config.get_scale_factor()
        if isinstance(sf, list):
            self._scale_array  = self._build_scale_array(config, self.hand_type, sf)
            self._scale_factor = 1.0
        else:
            self._scale_array  = None
            self._scale_factor = float(sf)

        total       = vector_weight + position_weight
        stage1_time = IK_MAX_TIME * vector_weight  / total
        stage2_time = IK_MAX_TIME * position_weight / total

        s1_dict, s2_dict = config.get_two_stage_config(self.hand_type)
        s1_dict.update({'normal_delta': NORM_DELTA, 'huber_delta': HUBER_DELTA})
        s2_dict.update({'normal_delta': NORM_DELTA, 'huber_delta': HUBER_DELTA})

        cfg1 = RetargetingConfig.from_dict(s1_dict)
        self._seq_stage1 = cfg1.build()
        self._seq_stage1.optimizer.opt.set_maxtime(stage1_time)
        
        cfg2 = RetargetingConfig.from_dict(s2_dict)
        self._seq_stage2 = cfg2.build()
        self._seq_stage2.optimizer.opt.set_maxtime(stage2_time)

        s1_human            = np.array(cfg1.target_link_human_indices)
        self._s1_origin_idx = s1_human[0].astype(np.int32)
        self._s1_task_idx   = s1_human[1].astype(np.int32)
        self._s2_tip_idx    = np.array(cfg2.target_link_human_indices, dtype=np.int32)

        robot               = self._seq_stage1.optimizer.robot
        self.joint_names    = list(robot.dof_joint_names)
        fixed_names         = set(config.get_fixed_joint_names(self.hand_type))
        self._fixed_indices = [i for i, n in enumerate(robot.dof_joint_names) if n in fixed_names]

        wrist_cfg          = config._WRIST_JOINTS.get(self.hand_type, {})
        self._wrist_joints = list(wrist_cfg.keys())

    # ─────────────────────────────────────────

    def compute(self, sensor_quats_17: np.ndarray) -> np.ndarray:
        """17×4 센서 쿼터니언 → 로봇 조인트 각도 (rad).

        Returns:
            np.ndarray: len(joint_names) 크기의 조인트 각도 배열
        Side-effect:
            self.last_human_positions 에 변환 후 human 포지션 저장 (TF 발행용)
        """
        positions          = self.fk.compute_positions(sensor_quats_17)
        positions_centered = positions - positions[0]
        positions_robot    = (self._coord_transform @ positions_centered.T).T

        if self._scale_array is not None:
            positions_robot *= self._scale_array[:, None]
        elif self._scale_factor != 1.0:
            positions_robot *= self._scale_factor

        self.last_human_positions = positions_robot  # TF 발행용

        robot_qpos = self._two_stage_retarget(positions_robot)

        if self._fixed_indices:
            robot_qpos[self._fixed_indices] = 0.0

        if self._wrist_joints:
            robot_qpos[0] = self._wrist_retarget(sensor_quats_17[0])

        return robot_qpos

    # ─────────────────────────────────────────

    def _two_stage_retarget(self, positions_robot: np.ndarray) -> np.ndarray:
        ref_vec     = positions_robot[self._s1_task_idx] - positions_robot[self._s1_origin_idx]
        stage1_qpos = self._seq_stage1.retarget(ref_vec)
        self._seq_stage2.set_qpos(stage1_qpos)
        return self._seq_stage2.retarget(positions_robot[self._s2_tip_idx])

    def _wrist_retarget(self, raw_xyzw: np.ndarray) -> float:
        """AGA 손목 쿼터니언 → 로봇 손목 조인트 각도 (rad, swing-twist)."""
        q_corr  = raw_xyzw * np.array([1.0, -1.0, -1.0, 1.0])
        R_aga   = ScipyR.from_quat(q_corr)
        C       = ScipyR.from_matrix(self._coord_transform.astype(np.float64))
        R_robot = C * R_aga * C.inv()
        q = R_robot.as_quat()
        if q[3] < 0:
            q = -q
        euler = (ScipyR.from_euler('x', 90, degrees=True) * R_robot).as_euler('xyz', degrees=False)
        return -euler[0]

    @staticmethod
    def _build_scale_array(config: HandConfig, hand_type: str, sf_list: List[float]) -> np.ndarray:
        fingers = config._get_fingers(hand_type)
        arr = np.ones(23, dtype=np.float32)
        for i, f in enumerate(fingers):
            scale = float(sf_list[i]) if i < len(sf_list) else 1.0
            for idx in f.human[1:]:
                arr[idx] = scale
        return arr
