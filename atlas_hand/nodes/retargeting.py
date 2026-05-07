#!/usr/bin/env python3
"""
retargeting.py
2단계 Hand Retargeting ROS 2 노드

파이프라인 (2단계 모드 — orca_hand):
  quaternions
    -> HandSphericalFK
    -> 좌표 변환 (base -> Robot)
    -> Stage 1 : VectorOptimizer  — 전체 손 방향/형태 정규화  (warm-start)
    -> Stage 2 : PositionOptimizer — 손끝 위치 정밀 보정 + LP filter
    -> /joint_states

파이프라인 (단일 단계 모드 — hand_rerun, robotis_hx5):
  quaternions -> FK -> 좌표 변환 -> SeqRetargeting -> /joint_states
"""

import sys
from typing import Optional, List, Union

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation as ScipyR

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import tf2_ros

from atlas_hand.core.hand_configs import CONFIG_REGISTRY, HandConfig
from atlas_hand.core.hand_spherical_fk import HandSphericalFK

# ==============================================================================
# PARAMETERS
# ==============================================================================

HUBER_DELTA: float     = 0.025
NORM_DELTA: float      = 0.01
TIMER_SEC: float       = 0.02
IK_MAX_TIME: float     = 0.02   # 두 단계 합산 시간 예산

VECTOR_WEIGHT: float   = 1.0     # Stage 1 시간 비중 (vector_weight ROS param 기본값)
POSITION_WEIGHT: float = 4.0     # Stage 2 시간 비중 (position_weight ROS param 기본값)

OUTPUT_TOPIC: str = "/joint_states"
INPUT_TOPIC: str  = "/{side}_hand/quaternions"

# 23-point human hand joint names (wrist=0, thumb=1-5, index=6-9, middle=10-13,
# ring=14-17, pinky_cmc=18, pinky=19-22)
_HUMAN_JOINT_NAMES: List[str] = [
    'wrist',
    'thumb_cmc', 'thumb_mcp', 'thumb_ip', 'thumb_dip', 'thumb_tip',
    'index_mcp',  'index_pip',  'index_dip',  'index_tip',
    'middle_mcp', 'middle_pip', 'middle_dip', 'middle_tip',
    'ring_mcp',   'ring_pip',   'ring_dip',   'ring_tip',
    'pinky_cmc','pinky_mcp',  'pinky_pip',  'pinky_dip',  'pinky_tip',
]

try:
    from dex_retargeting.retargeting_config import RetargetingConfig
except ImportError as e:
    print(f"Error: {e}")
    sys.exit(1)

# 2단계 human-index 배열은 각 HandConfig.get_two_stage_config()에서 per-instance로 읽음.
# Stage 1 target_link_human_indices: shape (2, n_vectors) → [0]=origin, [1]=task
# Stage 2 target_link_human_indices: shape (n_tips,)      → 절대 위치 인덱스


# ==============================================================================
# ROS 2 NODE
# ==============================================================================

class RetargetingNode(Node):

    def __init__(self):
        super().__init__('retargeting_node')
        self.declare_parameter('hand_type',       'left')
        self.declare_parameter('robot_config',    'hand_rerun')
        self.declare_parameter('vector_weight',   VECTOR_WEIGHT)
        self.declare_parameter('position_weight', POSITION_WEIGHT)
        self.declare_parameter('tf_parent_frame', '')

        self.hand_type  = self.get_parameter('hand_type').value.lower()
        config_name     = self.get_parameter('robot_config').value
        config          = self._create_config(config_name)

        self._coord_transform    = config.get_coord_transform(self.hand_type)
        self._tf_coord_transform = config.get_tf_coord_transform(self.hand_type)
        sf = config.get_scale_factor()
        if isinstance(sf, list):
            self._scale_array  = self._build_scale_array(config, sf)
            self._scale_factor = 1.0
        else:
            self._scale_array  = None
            self._scale_factor = float(sf)
        tf_param                 = self.get_parameter('tf_parent_frame').value
        self._tf_parent_frame    = tf_param if tf_param else config.get_wrist_link_name(self.hand_type)

        self.latest_quats: Optional[np.ndarray] = None

        self._init_seq_retarget(config)
        self._init_wrist_retarget(config)

        self.fk = HandSphericalFK(self.hand_type)
        self.joint_state_pub = self.create_publisher(JointState, OUTPUT_TOPIC, 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.quat_sub = self.create_subscription(
            Float32MultiArray,
            INPUT_TOPIC.format(side=self.hand_type),
            self._quaternion_callback,
            10,
        )
        self.timer = self.create_timer(TIMER_SEC, self._control_loop)
        self.get_logger().info(
            f"Node Ready | hand={self.hand_type} | robot_config={config_name}"
        )

    # ------------------------------------------------------------------

    def _create_config(self, name: str) -> HandConfig:
        cls = CONFIG_REGISTRY.get(name)
        if cls is None:
            raise ValueError(
                f"Unknown robot_config '{name}'. Available: {list(CONFIG_REGISTRY.keys())}"
            )
        return cls()

    def _init_seq_retarget(self, config: HandConfig):
        vw = self.get_parameter('vector_weight').value
        pw = self.get_parameter('position_weight').value
        total = vw + pw
        stage1_time = IK_MAX_TIME * vw / total
        stage2_time = IK_MAX_TIME * pw / total

        two_stage = config.get_two_stage_config(self.hand_type)

        self._setup_two_stage(config, two_stage, stage1_time, stage2_time)


    def _setup_two_stage(self, config, two_stage, stage1_time, stage2_time):
        s1_dict, s2_dict = two_stage
        s1_dict.update({'normal_delta': NORM_DELTA, 'huber_delta': HUBER_DELTA})
        s2_dict.update({
            'normal_delta':   NORM_DELTA,
            'huber_delta':    HUBER_DELTA,
        })

        cfg1 = RetargetingConfig.from_dict(s1_dict)
        cfg2 = RetargetingConfig.from_dict(s2_dict)

        self.seq_stage1 = cfg1.build()
        self.seq_stage1.optimizer.opt.set_maxtime(stage1_time)

        self.seq_stage2 = cfg2.build()
        self.seq_stage2.optimizer.opt.set_maxtime(stage2_time)

        # Human-index 배열: 각 로봇 config 의 target_link_human_indices 에서 추출
        s1_human = np.array(cfg1.target_link_human_indices)   # (2, n_vectors)
        self._s1_origin_idx = s1_human[0].astype(np.int32)
        self._s1_task_idx   = s1_human[1].astype(np.int32)
        self._s2_tip_idx    = np.array(cfg2.target_link_human_indices, dtype=np.int32)

        robot = self.seq_stage1.optimizer.robot
        self.joint_names = list(robot.dof_joint_names)
        self._fixed_indices = self._get_fixed_indices(config, robot)
        self._two_stage_mode = True
        self.get_logger().info(
            f"Optimizer: Vector({stage1_time*1000:.0f}ms) → Position({stage2_time*1000:.0f}ms)"
        )



    def _build_scale_array(self, config: HandConfig, sf_list: List[float]) -> np.ndarray:
        fingers = config._get_fingers(self.hand_type)
        arr = np.ones(23, dtype=np.float32)
        for i, f in enumerate(fingers):
            scale = float(sf_list[i]) if i < len(sf_list) else 1.0
            for idx in f.human[1:]:  # wrist(0)은 centering 후 [0,0,0]이므로 제외
                arr[idx] = scale
        return arr

    def _init_wrist_retarget(self, config):
        wrist_cfg          = config._WRIST_JOINTS.get(self.hand_type, {})
        self._wrist_joints = list(wrist_cfg.keys())
        self._wrist_axes   = [np.array(v, dtype=np.float64) for v in wrist_cfg.values()]
        if self._wrist_joints:
            self.joint_names = self.joint_names + self._wrist_joints
            self.get_logger().info(f"Wrist retargeting ON: {self._wrist_joints}")

    def _get_fixed_indices(self, config, robot):
        fixed_names = set(config.get_fixed_joint_names(self.hand_type))
        return [i for i, n in enumerate(robot.dof_joint_names) if n in fixed_names]

    # ------------------------------------------------------------------

    def _quaternion_callback(self, msg: Float32MultiArray):
        if len(msg.data) == 68:
            self.latest_quats = np.array(msg.data).reshape(17, 4)

    def _control_loop(self):
        if self.latest_quats is None:
            return

        try:
            positions    = self.fk.compute_positions(self.latest_quats)
            positions_centered = positions - positions[0]
            positions_robot    = (self._coord_transform    @ positions_centered.T).T
            positions_tf       = (self._tf_coord_transform @ positions_centered.T).T

            if self._scale_array is not None:
                positions_robot *= self._scale_array[:, None]
                positions_tf    *= self._scale_array[:, None]
            elif self._scale_factor != 1.0:
                positions_robot *= self._scale_factor
                positions_tf    *= self._scale_factor

            self._broadcast_human_tf(positions_tf)

            robot_qpos = self._retarget_two_stage(positions_robot)

            if self._fixed_indices:
                robot_qpos[self._fixed_indices] = 0.0

            if self._wrist_joints:
                wrist_angles = self._retarget_wrist(self.latest_quats[0])
                robot_qpos   = np.concatenate([robot_qpos, wrist_angles])

            self._publish(robot_qpos)

        except Exception as e:
            self.get_logger().error(f"Processing error: {e}", throttle_duration_sec=1.0)

    def _broadcast_human_tf(self, positions_robot: np.ndarray):
        now = self.get_clock().now().to_msg()
        transforms = []
        n = min(len(positions_robot), len(_HUMAN_JOINT_NAMES))
        for i in range(n):
            t = TransformStamped()
            t.header.stamp    = now
            t.header.frame_id = self._tf_parent_frame
            t.child_frame_id  = f'human_{self.hand_type}_{_HUMAN_JOINT_NAMES[i]}'
            # t.transform.translation.x = float(positions_robot[i, 0])
            # t.transform.translation.y = -float(positions_robot[i, 2])
            # t.transform.translation.z = float(positions_robot[i, 1])
            t.transform.translation.x = float(positions_robot[i, 0])
            t.transform.translation.y = float(positions_robot[i, 1])
            t.transform.translation.z = float(positions_robot[i, 2])
            t.transform.rotation.w    = 1.0
            transforms.append(t)
        self._tf_broadcaster.sendTransform(transforms)

    def _retarget_wrist(self, raw_xyzw: np.ndarray) -> np.ndarray:
        """AGA 손목 쿼터니언 → 로봇 손목 조인트 각도 (직접 매핑, swing-twist).

        raw_xyzw : AGA 센서 0번 쿼터니언 [x, y, z, w]
        반환     : len(_wrist_joints) 크기의 각도 배열 (rad)
        """
        # 센서 좌표계 보정 (FK와 동일: y, z 반전)
        q_corr = raw_xyzw * np.array([1.0, -1.0, -1.0, 1.0])
        R_aga  = ScipyR.from_quat(q_corr)

        # AGA → 로봇 프레임 변환
        C       = ScipyR.from_matrix(self._coord_transform.astype(np.float64))
        R_robot = C * R_aga * C.inv()

        # 정규 형식: w >= 0 (q와 -q는 같은 회전)
        q = R_robot.as_quat()   # [x, y, z, w]
        if q[3] < 0:
            q = -q
        w, xyz = q[3], q[:3]

        # 각 축별 swing-twist 분해 → 회전 각도
        angles = np.zeros(len(self._wrist_axes))
        for i, axis in enumerate(self._wrist_axes):
            proj       = float(np.dot(xyz, axis))   # = sin(θ/2)
            angles[i]  = 2.0 * np.arctan2(proj, w)  # θ ∈ (−π, π)
        return angles

    def _retarget_two_stage(self, positions_robot: np.ndarray) -> np.ndarray:
        ref_vec     = positions_robot[self._s1_task_idx] - positions_robot[self._s1_origin_idx]
        stage1_qpos = self.seq_stage1.retarget(ref_vec)
        # return stage1_qpos
        self.seq_stage2.set_qpos(stage1_qpos)
        ref_pos = positions_robot[self._s2_tip_idx]
        return self.seq_stage2.retarget(ref_pos)

    def _publish(self, robot_qpos: np.ndarray):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = self.joint_names
        msg.position     = robot_qpos.tolist()
        self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RetargetingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
