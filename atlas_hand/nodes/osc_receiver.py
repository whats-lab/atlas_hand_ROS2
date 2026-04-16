#!/usr/bin/env python3
"""
osc_receiver_node.py
AGA 글러브 OSC 수신 ROS 2 노드

발행 토픽:
  /left_hand/quaternions  (std_msgs/Float32MultiArray)
  /right_hand/quaternions (std_msgs/Float32MultiArray)
"""

import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from pythonosc import dispatcher as osc_dispatcher
from pythonosc import osc_server
from pythonosc.udp_client import SimpleUDPClient

from atlas_hand.config import (
    OSC_LISTEN_IP,
    OSC_SERVER_PORT,
    OSC_CLIENT_PORT,
    OSC_TARGET_IP,
    AGA_RAW_FLOAT_COUNT,
    AGA_SKIP_JOINT,
    HEARTBEAT_INTERVAL_SEC,
    TOPIC_LEFT_QUAT,
    TOPIC_RIGHT_QUAT,
)


class OSCReceiverNode(Node):
    """
    AGA 글러브 OSC 수신 ROS 2 노드.

    OSC 메시지를 별도 스레드에서 수신하여 ROS 2 토픽으로 발행.

    파라미터:
      listen_ip   (str)  : OSC 수신 IP (기본값: "0.0.0.0")
      server_port (int)  : OSC 수신 포트 (기본값: 4040)
      client_port (int)  : 하트비트 송신 포트 (기본값: 4042)
      target_ip   (str)  : 하트비트 대상 IP (기본값: "127.0.0.1")
      verbose     (bool) : 상세 로그 출력 (기본값: False)
    """

    def __init__(self):
        super().__init__('osc_receiver')

        # ── ROS 2 파라미터 선언 ──────────────────────────────────────
        self.declare_parameter('listen_ip',   OSC_LISTEN_IP)
        self.declare_parameter('server_port', OSC_SERVER_PORT)
        self.declare_parameter('client_port', OSC_CLIENT_PORT)
        self.declare_parameter('target_ip',   OSC_TARGET_IP)
        self.declare_parameter('verbose',     False)

        self._listen_ip   = self.get_parameter('listen_ip').value
        self._server_port = self.get_parameter('server_port').value
        self._client_port = self.get_parameter('client_port').value
        self._target_ip   = self.get_parameter('target_ip').value
        self._verbose     = self.get_parameter('verbose').value

        # ── 발행자 ──────────────────────────────────────────────────
        self._left_pub  = self.create_publisher(Float32MultiArray, TOPIC_LEFT_QUAT,  10)
        self._right_pub = self.create_publisher(Float32MultiArray, TOPIC_RIGHT_QUAT, 10)

        # ── OSC 서버 및 하트비트 시작 ────────────────────────────────
        self._running = True
        self._setup_osc()

        self.get_logger().info(
            f'OSC 수신 대기: {self._listen_ip}:{self._server_port}  '
            f'(float x {AGA_RAW_FLOAT_COUNT} per hand)'
        )
        self.get_logger().info(
            f'하트비트 -> {self._target_ip}:{self._client_port} '
            f'(간격 {HEARTBEAT_INTERVAL_SEC}s)'
        )

    # ── OSC 초기화 ───────────────────────────────────────────────────

    def _setup_osc(self) -> None:
        disp = osc_dispatcher.Dispatcher()
        disp.map('/left/quat/get',     self._h_left_quat)
        disp.map('/right/quat/get',    self._h_right_quat)
        disp.map('/device/status/get', self._h_device_status)
        disp.map('/heartbeat/get',     self._h_heartbeat)
        disp.set_default_handler(self._h_default)

        self._server = osc_server.ThreadingOSCUDPServer(
            (self._listen_ip, self._server_port), disp
        )
        self._srv_thread = threading.Thread(
            target=self._server.serve_forever,
            daemon=True,
            name='OSCServer',
        )
        self._srv_thread.start()

        self._udp_client = SimpleUDPClient(self._target_ip, self._client_port)
        self._hb_thread = threading.Thread(
            target=self._heartbeat_loop,
            daemon=True,
            name='OSCHeartbeat',
        )
        self._hb_thread.start()

    # ── OSC 파싱 헬퍼 ────────────────────────────────────────────────

    def _parse_floats(self, args: tuple, count: int):
        if len(args) < count + 1:
            return None
        try:
            return np.array(args[1:count + 1], dtype=np.float32)
        except (TypeError, ValueError):
            return None

    def _to_17_sensors(self, raw: np.ndarray) -> np.ndarray:
        """18 관절 쿼터니언 (72 floats) -> 17 센서 쿼터니언 (68 floats).
        AGA_SKIP_JOINT(pinky_0, index 14) 제거.
        """
        return np.delete(raw.reshape(18, 4), AGA_SKIP_JOINT, axis=0).flatten()

    def _publish_quat(self, pub, raw: np.ndarray, side: str) -> None:
        msg = Float32MultiArray()
        msg.data = self._to_17_sensors(raw).tolist()
        pub.publish(msg)
        if self._verbose:
            self.get_logger().debug(
                f'[OSC] <- {side}  raw[:4]={raw[:4].round(3)}'
            )

    # ── OSC 핸들러 ───────────────────────────────────────────────────

    def _h_left_quat(self, address: str, *args) -> None:
        raw = self._parse_floats(args, AGA_RAW_FLOAT_COUNT)
        if raw is None:
            self.get_logger().warn(
                f'[OSC] /left/quat/get 파싱 실패 (수신 args={len(args)}, '
                f'필요 {AGA_RAW_FLOAT_COUNT + 1})'
            )
            return
        self._publish_quat(self._left_pub, raw, 'left')

    def _h_right_quat(self, address: str, *args) -> None:
        raw = self._parse_floats(args, AGA_RAW_FLOAT_COUNT)
        if raw is None:
            self.get_logger().warn(
                f'[OSC] /right/quat/get 파싱 실패 (수신 args={len(args)}, '
                f'필요 {AGA_RAW_FLOAT_COUNT + 1})'
            )
            return
        self._publish_quat(self._right_pub, raw, 'right')

    def _h_device_status(self, address: str, *args) -> None:
        if len(args) < 3:
            return
        if self._verbose:
            self.get_logger().debug(
                f'[OSC] device status  left={args[1]}  right={args[2]}'
            )

    def _h_heartbeat(self, address: str, *args) -> None:
        if self._verbose:
            self.get_logger().debug('[OSC] <- heartbeat from device')

    def _h_default(self, address: str, *args) -> None:
        if self._verbose:
            self.get_logger().debug(f'[OSC] 미처리 주소: {address}')

    def _heartbeat_loop(self) -> None:
        while self._running:
            try:
                self._udp_client.send_message('/device/status/get', '4')
                if self._verbose:
                    self.get_logger().debug('[OSC] -> heartbeat sent')
            except Exception as e:
                self.get_logger().warn(f'[OSC] heartbeat 전송 실패: {e}')
            time.sleep(HEARTBEAT_INTERVAL_SEC)

    def destroy_node(self) -> None:
        self._running = False
        if hasattr(self, '_server'):
            self._server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OSCReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
