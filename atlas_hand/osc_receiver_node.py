#!/usr/bin/env python3
"""
atlas_hand/osc_receiver_node.py
AGA 글러브 OSC 수신/송신 ROS 2 노드 — AGAOscHandler 래퍼

수신 토픽:
  /left_hand/haptic      (std_msgs/Int32MultiArray) — 왼손 햅틱 on 명령
  /right_hand/haptic     (std_msgs/Int32MultiArray) — 오른손 햅틱 on 명령
  /left_hand/haptic/off  (std_msgs/Empty)           — 왼손 햅틱 끄기 트리거
  /right_hand/haptic/off (std_msgs/Empty)           — 오른손 햅틱 끄기 트리거

발행 토픽:
  /left_hand/quaternions   (std_msgs/Float32MultiArray) — 왼손 관절 쿼터니언 (17×4)
  /right_hand/quaternions  (std_msgs/Float32MultiArray) — 오른손 관절 쿼터니언 (17×4)
  /left_hand/connected     (std_msgs/Bool)              — 왼손 연결 상태
  /right_hand/connected    (std_msgs/Bool)              — 오른손 연결 상태

파라미터:
  listen_ip   (str)  : OSC 수신 IP        (기본값: "0.0.0.0")
  server_port (int)  : OSC 수신 포트      (기본값: 4040)
  client_port (int)  : 하트비트 송신 포트  (기본값: 4042)
  target_ip   (str)  : 디바이스 IP        (기본값: "127.0.0.1")
  verbose     (bool) : 상세 로그 출력     (기본값: True)
"""

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Float32MultiArray, Int32MultiArray

from atlas_hand_core.config import (
    AGA_FINGER_COUNT,
    AGA_RAW_FLOAT_COUNT,
    HEARTBEAT_INTERVAL_SEC,
    OSC_CLIENT_PORT,
    OSC_LISTEN_IP,
    OSC_SERVER_PORT,
    OSC_TARGET_IP,
)
from atlas_hand_core.osc_handler import AGAOscHandler
from atlas_hand.config import (
    TOPIC_LEFT_CONNECTED,
    TOPIC_LEFT_HAPTIC,
    TOPIC_LEFT_HAPTIC_OFF,
    TOPIC_LEFT_QUAT,
    TOPIC_RIGHT_CONNECTED,
    TOPIC_RIGHT_HAPTIC,
    TOPIC_RIGHT_HAPTIC_OFF,
    TOPIC_RIGHT_QUAT,
)


class OSCReceiverNode(Node):
    """AGA 글러브 OSC 수신/송신 ROS 2 노드."""

    def __init__(self):
        super().__init__('osc_receiver')

        self.declare_parameter('listen_ip',   OSC_LISTEN_IP)
        self.declare_parameter('server_port', OSC_SERVER_PORT)
        self.declare_parameter('client_port', OSC_CLIENT_PORT)
        self.declare_parameter('target_ip',   OSC_TARGET_IP)
        self.declare_parameter('verbose',     True)

        listen_ip   = self.get_parameter('listen_ip').value
        server_port = self.get_parameter('server_port').value
        client_port = self.get_parameter('client_port').value
        target_ip   = self.get_parameter('target_ip').value
        verbose     = self.get_parameter('verbose').value

        # ── 쿼터니언 발행자 ──────────────────────────────────────────
        self._left_pub  = self.create_publisher(Float32MultiArray, TOPIC_LEFT_QUAT,  10)
        self._right_pub = self.create_publisher(Float32MultiArray, TOPIC_RIGHT_QUAT, 10)

        # ── 연결 상태 발행자 ─────────────────────────────────────────
        self._left_conn_pub  = self.create_publisher(Bool, TOPIC_LEFT_CONNECTED,  10)
        self._right_conn_pub = self.create_publisher(Bool, TOPIC_RIGHT_CONNECTED, 10)

        _haptic_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        _haptic_qos=10
        # ── 햅틱 구독자 ───────────────────────────────────────────────
        self.create_subscription(
            Int32MultiArray, TOPIC_LEFT_HAPTIC,     self._cb_left_haptic,    _haptic_qos)
        self.create_subscription(
            Int32MultiArray, TOPIC_RIGHT_HAPTIC,    self._cb_right_haptic,   _haptic_qos)
        self.create_subscription(
            Empty,           TOPIC_LEFT_HAPTIC_OFF, self._cb_left_hapt_off,  _haptic_qos)
        self.create_subscription(
            Empty,           TOPIC_RIGHT_HAPTIC_OFF,self._cb_right_hapt_off, _haptic_qos)

        # ── OSC 핸들러 ────────────────────────────────────────────────
        self._osc = AGAOscHandler(
            listen_ip=listen_ip,
            server_port=server_port,
            target_ip=target_ip,
            client_port=client_port,
            on_left_quat=self._pub_left_quat,
            on_right_quat=self._pub_right_quat,
            on_status_change=self._pub_status,
            verbose=verbose,
        )
        self._osc.start()

        self.get_logger().info(
            f'OSC 수신 대기: {listen_ip}:{server_port}  '
            f'(float x {AGA_RAW_FLOAT_COUNT} per hand)'
        )
        self.get_logger().info(
            f'하트비트 -> {target_ip}:{client_port} (간격 {HEARTBEAT_INTERVAL_SEC}s)'
        )

    # ── OSC 콜백 → ROS 2 발행 ─────────────────────────────────────────

    def _pub_left_quat(self, quats: np.ndarray) -> None:
        msg = Float32MultiArray()
        msg.data = quats.flatten().tolist()
        self._left_pub.publish(msg)

    def _pub_right_quat(self, quats: np.ndarray) -> None:
        msg = Float32MultiArray()
        msg.data = quats.flatten().tolist()
        self._right_pub.publish(msg)

    def _pub_status(self, left_ok: bool, right_ok: bool) -> None:
        self._left_conn_pub.publish(Bool(data=left_ok))
        self._right_conn_pub.publish(Bool(data=right_ok))
        self.get_logger().info(
            f'[OSC] 연결 상태 변경: left={left_ok}  right={right_ok}'
        )

    # ── 햅틱 ROS 2 콜백 ──────────────────────────────────────────────

    def _cb_left_haptic(self, msg: Int32MultiArray) -> None:
        if len(msg.data) < AGA_FINGER_COUNT:
            self.get_logger().warn(
                f'[ROS] /left_hand/haptic 데이터 부족 ({len(msg.data)}/{AGA_FINGER_COUNT})'
            )
            return
        self._osc.send_haptic('left', list(msg.data))

    def _cb_right_haptic(self, msg: Int32MultiArray) -> None:
        if len(msg.data) < AGA_FINGER_COUNT:
            self.get_logger().warn(
                f'[ROS] /right_hand/haptic 데이터 부족 ({len(msg.data)}/{AGA_FINGER_COUNT})'
            )
            return
        self._osc.send_haptic('right', list(msg.data))

    def _cb_left_hapt_off(self, _: Empty) -> None:
        self._osc.send_haptic('left', [0] * AGA_FINGER_COUNT)

    def _cb_right_hapt_off(self, _: Empty) -> None:
        self._osc.send_haptic('right', [0] * AGA_FINGER_COUNT)

    # ── 종료 ─────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self._osc.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OSCReceiverNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
