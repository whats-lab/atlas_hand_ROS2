#!/usr/bin/env python3
"""
retargeting.py
Position 기반 Hand Retargeting ROS 2 노드

파이프라인:
  /left_hand/quaternions or /right_hand/quaternions (Float32MultiArray, 17x4=68 floats)
    -> HandSphericalFK (FK)
    -> 좌표 변환 (Quest -> Robot)
    -> PositionOptimizer (IK)
    -> Low-pass filter
    -> /joint_states (JointState)

로봇 선택:
  ros2 run atlas_hand retarget --ros-args -p hand_type:=left
    # robot_config 미지정 → hand_rerun (내부 뼈대 URDF, 기본값)
    # robot_config 지정   → CONFIG_REGISTRY 등록 로봇
  ros2 run atlas_hand retarget --ros-args -p hand_type:=left -p robot_config:=robotis_hx5
"""

import sys
from typing import List, Optional

import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from atlas_hand.core.hand_configs import CONFIG_REGISTRY, HandConfig
from atlas_hand.core.hand_spherical_fk import HandSphericalFK

# ==============================================================================
# GENERIC PARAMETERS
# ==============================================================================

HUBER_DELTA: float     = 0.001
NORM_DELTA: float      = 0.0005
LP_FILTER_ALPHA: float = 0.35
TIMER_SEC: float       = 0.02
IK_MAX_TIME: float     = 0.01

OUTPUT_TOPIC: str = "/joint_states"
INPUT_TOPIC: str  = "/{side}_hand/quaternions"

try:
    from dex_retargeting.optimizer import PositionOptimizer
    from dex_retargeting.robot_wrapper import RobotWrapper
except ImportError as e:
    print(f"Error: {e}")
    sys.exit(1)


# ==============================================================================
# UTILITIES
# ==============================================================================

class LowPassFilter:
    def __init__(self, alpha: float = 0.3):
        self.alpha = np.clip(alpha, 0.0, 1.0)
        self._y = None

    def filter(self, x: np.ndarray) -> np.ndarray:
        if self._y is None:
            self._y = x.copy()
        else:
            self._y = self.alpha * x + (1.0 - self.alpha) * self._y
        return self._y.copy()


# ==============================================================================
# ROS 2 NODE
# ==============================================================================

class QuestRetargetingNode(Node):

    def __init__(self):
        super().__init__('retargeting_node')
        self.declare_parameter('hand_type',    'left')
        self.declare_parameter('robot_config', 'hand_rerun')

        self.hand_type = self.get_parameter('hand_type').value.lower()
        config_name    = self.get_parameter('robot_config').value

        config = self._create_config(config_name)

        self._coord_transform     = config.get_coord_transform(self.hand_type)
        self._scale_factor        = config.get_scale_factor()
        self._human_joint_indices = config.get_human_joint_indices()

        target_link_names = config.get_target_link_names(self.hand_type)
        human_indices     = np.array(self._human_joint_indices, dtype=np.int32)

        self.latest_quats: Optional[np.ndarray] = None

        self._init_robot(config)
        self._init_optimizer(
            target_link_names,
            human_indices,
            config.get_optimizer_weights(len(target_link_names)),
        )

        self.fk        = HandSphericalFK(self.hand_type)
        self.last_qpos = np.zeros(self.robot.dof, dtype=np.float32)
        self.lp_filter = LowPassFilter(alpha=LP_FILTER_ALPHA)

        self.joint_state_pub = self.create_publisher(JointState, OUTPUT_TOPIC, 10)
        input_topic = INPUT_TOPIC.format(side=self.hand_type)
        self.quat_sub = self.create_subscription(
            Float32MultiArray, input_topic, self._quaternion_callback, 10
        )

        self.timer = self.create_timer(TIMER_SEC, self._control_loop)
        self.get_logger().info(
            f"Node Ready | hand={self.hand_type} | robot_config={config_name}"
        )

    def _create_config(self, name: str) -> HandConfig:
        cls = CONFIG_REGISTRY.get(name)
        if cls is None:
            available = list(CONFIG_REGISTRY.keys())
            raise ValueError(
                f"Unknown robot_config '{name}'. "
                f"Available: {available}"
            )
        return cls()

    def _init_robot(self, config: HandConfig):
        urdf_path = config.get_urdf_path(self.hand_type)
        self.robot = RobotWrapper(urdf_path)
        self.joint_names = list(self.robot.dof_joint_names)

        for i in range(len(self.robot.dof_joint_names)):
            lo, hi = self.robot.joint_limits[i]
            self.robot.joint_limits[i] = [max(lo, -0.1), hi]

    def _init_optimizer(self, target_link_names: List[str],
                        human_indices: np.ndarray,
                        weights: Optional[np.ndarray]):
        self.optimizer = PositionOptimizer(
            robot=self.robot,
            target_joint_names=self.joint_names,
            target_link_names=target_link_names,
            target_link_human_indices=human_indices,
            huber_delta=HUBER_DELTA,
            norm_delta=NORM_DELTA,
        )

        if weights is not None:
            self.optimizer.weights = weights

        try:
            target_opt = (getattr(self.optimizer, 'optimizer', None)
                          or getattr(self.optimizer, 'opt', None))
            if target_opt is not None:
                target_opt.set_maxtime(IK_MAX_TIME)
        except Exception as e:
            self.get_logger().warn(f"Optimizer 설정 오류 (무시 가능): {e}")

    def _quaternion_callback(self, msg: Float32MultiArray):
        if len(msg.data) == 68:
            self.latest_quats = np.array(msg.data).reshape(17, 4)

    def _control_loop(self):
        if self.latest_quats is None:
            return

        try:
            positions_quest    = self.fk.compute_positions(self.latest_quats)
            wrist_pos          = positions_quest[0].copy()
            positions_centered = positions_quest - wrist_pos
            positions_robot    = (self._coord_transform @ positions_centered.T).T

            if self._scale_factor != 1.0:
                positions_robot *= self._scale_factor

            target_positions = positions_robot[self._human_joint_indices]

            qpos_raw = self.optimizer.retarget(
                target_positions,
                np.array([], dtype=np.float32),
                self.last_qpos,
            )

            self.last_qpos = self.lp_filter.filter(qpos_raw)
            self._publish_joint_states()

        except Exception as e:
            self.get_logger().error(f"Processing error: {e}", throttle_duration_sec=1.0)

    def _publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = self.joint_names
        msg.position     = self.last_qpos.tolist()
        self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = QuestRetargetingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
