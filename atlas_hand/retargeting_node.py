#!/usr/bin/env python3
"""
retargeting.py
Hand Retargeting ROS 2 노드

파이프라인 (HandRetargeter 코어 사용):
  /quaternions → HandRetargeter.compute() → /joint_states
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from atlas_hand_core.config import CONTROL_TIMER_SEC, POSITION_WEIGHT, VECTOR_WEIGHT
from atlas_hand_core.retargeter import HandRetargeter


class RetargetingNode(Node):

    def __init__(self):
        super().__init__('retargeting_node')
        self.declare_parameter('hand_type',       'left')
        self.declare_parameter('robot_config',    'hand_rerun')
        self.declare_parameter('vector_weight',   VECTOR_WEIGHT)
        self.declare_parameter('position_weight', POSITION_WEIGHT)

        hand_type   = self.get_parameter('hand_type').value.lower()
        config_name = self.get_parameter('robot_config').value
        vw          = self.get_parameter('vector_weight').value
        pw          = self.get_parameter('position_weight').value

        self.retargeter   = HandRetargeter(hand_type, config_name, vw, pw)
        self.latest_quats = None

        self._pub = self.create_publisher(JointState, '/joint_states', 10)
        self._sub = self.create_subscription(
            Float32MultiArray,
            f'/{hand_type}_hand/quaternions',
            self._quat_callback,
            10,
        )
        self.create_timer(CONTROL_TIMER_SEC, self._control_loop)
        self.get_logger().info(f"Node Ready | hand={hand_type} | robot_config={config_name}")

    def _quat_callback(self, msg: Float32MultiArray):
        if len(msg.data) == 68:
            self.latest_quats = np.array(msg.data).reshape(17, 4)

    def _control_loop(self):
        if self.latest_quats is None:
            return
        try:
            self._publish(self.retargeter.compute(self.latest_quats))
        except Exception as e:
            self.get_logger().error(f"Processing error: {e}", throttle_duration_sec=1.0)

    def _publish(self, robot_qpos: np.ndarray):
        msg          = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = self.retargeter.joint_names
        msg.position = robot_qpos.tolist()
        self._pub.publish(msg)


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
