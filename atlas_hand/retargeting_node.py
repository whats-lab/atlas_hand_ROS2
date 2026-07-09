#!/usr/bin/env python3
"""
retargeting.py
Hand Retargeting ROS 2 노드

파이프라인 (HandRetargeter 코어 사용):
  /quaternions → HandRetargeter.compute() → /joint_states + TF
"""

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from atlas_hand_core.config import (
    AGA_SENSOR_COUNT,
    CONTROL_TIMER_SEC,
    POSITION_WEIGHT,
    VECTOR_WEIGHT,
)
from atlas_hand_core.retargeter import HandRetargeter

_HUMAN_JOINT_NAMES = [
    'wrist',
    'thumb_cmc0', 'thumb_cmc1', 'thumb_mcp', 'thumb_ip',   'thumb_tip',
    'index_mcp',  'index_pip',  'index_dip',  'index_tip',
    'middle_mcp', 'middle_pip', 'middle_dip', 'middle_tip',
    'ring_mcp',   'ring_pip',   'ring_dip',   'ring_tip',
    'pinky0',     'pinky_mcp',  'pinky_pip',  'pinky_dip',  'pinky_tip',
]


class RetargetingNode(Node):

    def __init__(self):
        super().__init__('retargeting_node')
        self.declare_parameter('hand_type',       'left')
        self.declare_parameter('robot_config',    'base_hand')
        self.declare_parameter('vector_weight',   VECTOR_WEIGHT)
        self.declare_parameter('position_weight', POSITION_WEIGHT)
        self.declare_parameter('tf_parent_frame', '')

        hand_type   = self.get_parameter('hand_type').value.lower()
        config_name = self.get_parameter('robot_config').value
        vw          = self.get_parameter('vector_weight').value
        pw          = self.get_parameter('position_weight').value
        tf_param    = self.get_parameter('tf_parent_frame').value

        self.retargeter   = HandRetargeter(hand_type, config_name, vw, pw)
        self.hand_type    = hand_type
        self.latest_quats = None

        from atlas_hand_core.hand_configs import CONFIG_REGISTRY
        self._tf_parent_frame = (
            tf_param if tf_param
            else CONFIG_REGISTRY[config_name]().get_wrist_link_name(hand_type)
        )

        self._pub            = self.create_publisher(JointState, '/joint_states', 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._sub = self.create_subscription(
            Float32MultiArray,
            f'/{hand_type}_hand/quaternions',
            self._quat_callback,
            10,
        )
        self.create_timer(CONTROL_TIMER_SEC, self._control_loop)
        self.get_logger().info(f"Node Ready | hand={hand_type} | robot_config={config_name}")

    def _quat_callback(self, msg: Float32MultiArray):
        expected = AGA_SENSOR_COUNT * 4
        if len(msg.data) == expected:
            self.latest_quats = np.array(msg.data).reshape(AGA_SENSOR_COUNT, 4)
        else:
            self.get_logger().warn(
                f"쿼터니언 길이 불일치: {len(msg.data)} (기대 {expected}) — 프레임 무시",
                throttle_duration_sec=1.0,
            )

    def _control_loop(self):
        if self.latest_quats is None:
            return
        try:
            robot_qpos = self.retargeter.compute(self.latest_quats)
            self._publish(robot_qpos)
            self._broadcast_human_tf(self.retargeter.last_human_positions)
        except Exception as e:
            self.get_logger().error(f"Processing error: {e}", throttle_duration_sec=1.0)

    def _publish(self, robot_qpos: np.ndarray):
        msg              = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = self.retargeter.joint_names
        msg.position     = robot_qpos.tolist()
        self._pub.publish(msg)

    def _broadcast_human_tf(self, positions: np.ndarray):
        # 사람 손과 로봇 손은 링크 길이가 달라 중간 관절은 로봇 링크에 얹히지 않는다.
        # IK가 실제로 위치를 맞추는 손가락 팁만 wrist 링크 기준으로 발행 → 로봇 팁과 직접 비교.
        now        = self.get_clock().now().to_msg()
        transforms = []
        # wrist(0) + 팁: wrist 마커로 앵커 위치(=wrist 링크 원점)를 눈으로 검증.
        for i in [0, *self.retargeter.tip_human_indices]:
            if i >= len(_HUMAN_JOINT_NAMES):
                continue
            pos                       = positions[i]
            t                         = TransformStamped()
            t.header.stamp            = now
            t.header.frame_id         = self._tf_parent_frame
            t.child_frame_id          = f'human_{self.hand_type}_{_HUMAN_JOINT_NAMES[i]}'
            t.transform.translation.x = float(pos[0])
            t.transform.translation.y = float(pos[1])
            t.transform.translation.z = float(pos[2])
            t.transform.rotation.w    = 1.0
            transforms.append(t)
        self._tf_broadcaster.sendTransform(transforms)


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
