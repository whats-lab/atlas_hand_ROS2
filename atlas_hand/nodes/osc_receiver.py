#!/usr/bin/env python3
"""
osc_receiver.py
AGA 글러브 OSC 수신/송신 ROS 2 노드

수신 토픽:
  /left_hand/haptic      (std_msgs/Int32MultiArray) — 왼손 햅틱 on 명령  [thumb,index,middle,ring,pinky]
  /right_hand/haptic     (std_msgs/Int32MultiArray) — 오른손 햅틱 on 명령
  /left_hand/haptic/off  (std_msgs/Empty)           — 왼손 햅틱 끄기 트리거
  /right_hand/haptic/off (std_msgs/Empty)           — 오른손 햅틱 끄기 트리거

발행 토픽:
  /left_hand/quaternions   (std_msgs/Float32MultiArray) — 왼손 관절 쿼터니언
  /right_hand/quaternions  (std_msgs/Float32MultiArray) — 오른손 관절 쿼터니언
  /left_hand/connected     (std_msgs/Bool)              — 왼손 연결 상태
  /right_hand/connected    (std_msgs/Bool)              — 오른손 연결 상태

OSC 수신 주소 (디바이스 → 호스트):
  /left/quat/get       float×73  — 왼손 쿼터니언 (타입태그 + 72 floats)
  /right/quat/get      float×73  — 오른손 쿼터니언
  /device/status/get   bool×2    — 좌/우 연결 상태
  /left/hapt/get                 — 왼손 햅틱 에코 (디바이스 확인 응답)
  /right/hapt/get                — 오른손 햅틱 에코
  /left/hapt/ret/get             — 왼손 햅틱 처리 결과 반환
  /right/hapt/ret/get            — 오른손 햅틱 처리 결과 반환
  /device/alarm/get              — 디바이스 알람
  /heartbeat/get                 — 디바이스 하트비트

OSC 송신 주소 (호스트 → 디바이스):
  /device/status/get   "4"                            — 하트비트 요청
  /left/hapt/set       [str, int, int, int, int, ...] — 왼손 햅틱 (메시지 타입 "9")
  /right/hapt/set      [str, int, int, int, int, ...] — 오른손 햅틱 (메시지 타입 "10")
"""

import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32MultiArray, Int32MultiArray

from pythonosc import dispatcher as osc_dispatcher
from pythonosc import osc_server
from pythonosc.udp_client import SimpleUDPClient

from atlas_hand.config import (
    AGA_FINGER_COUNT,
    AGA_RAW_FLOAT_COUNT,
    AGA_SKIP_JOINT,
    HAPTIC_PING_INTERVAL_SEC,
    HEARTBEAT_INTERVAL_SEC,
    OSC_ADDR_LEFT_HAPT,
    OSC_ADDR_RIGHT_HAPT,
    OSC_CLIENT_PORT,
    OSC_LISTEN_IP,
    OSC_MSG_TYPE_LEFT_HAPT,
    OSC_MSG_TYPE_RIGHT_HAPT,
    OSC_SERVER_PORT,
    OSC_TARGET_IP,
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
    """
    AGA 글러브 OSC 수신/송신 ROS 2 노드.

    주요 기능:
      - 글러브 쿼터니언 수신 → ROS 2 토픽 발행
      - 디바이스 연결 상태 추적 → Bool 토픽 발행
      - ROS 2 int 배열 토픽 수신 → 햅틱 OSC 즉시 전송
      - /off 트리거 → 전체 0 전송으로 햅틱 끄기
      - 주기적 ROS 2 fake ping → 햅틱/off 토픽 DDS subscription 웜업

    파라미터:
      listen_ip   (str)  : OSC 수신 IP        (기본값: "0.0.0.0")
      server_port (int)  : OSC 수신 포트      (기본값: 4040)
      client_port (int)  : 하트비트 송신 포트  (기본값: 4042)
      target_ip   (str)  : 디바이스 IP        (기본값: "127.0.0.1")
      verbose     (bool) : 상세 로그 출력     (기본값: True)
    """

    def __init__(self):
        super().__init__('osc_receiver')

        # ── ROS 2 파라미터 선언 ──────────────────────────────────────
        self.declare_parameter('listen_ip',   OSC_LISTEN_IP)
        self.declare_parameter('server_port', OSC_SERVER_PORT)
        self.declare_parameter('client_port', OSC_CLIENT_PORT)
        self.declare_parameter('target_ip',   OSC_TARGET_IP)
        self.declare_parameter('verbose',     True)

        self._listen_ip   = self.get_parameter('listen_ip').value
        self._server_port = self.get_parameter('server_port').value
        self._client_port = self.get_parameter('client_port').value
        self._target_ip   = self.get_parameter('target_ip').value
        self._verbose     = self.get_parameter('verbose').value
        self._verbose = True

        # ── 쿼터니언 발행자 ──────────────────────────────────────────
        self._left_pub  = self.create_publisher(Float32MultiArray, TOPIC_LEFT_QUAT,  10)
        self._right_pub = self.create_publisher(Float32MultiArray, TOPIC_RIGHT_QUAT, 10)

        # ── 연결 상태 발행자 ─────────────────────────────────────────
        self._left_conn_pub  = self.create_publisher(Bool, TOPIC_LEFT_CONNECTED,  10)
        self._right_conn_pub = self.create_publisher(Bool, TOPIC_RIGHT_CONNECTED, 10)

        # ── 햅틱 토픽 ping 발행자 (DDS subscription 웜업용) ──────────
        # 이 노드가 직접 발행해 subscription을 살린다. ping 발행 시에는
        # _ping_active 플래그를 세워 자신의 콜백이 OSC 전송을 건너뛰게 한다.
        self._ping_pub_left_hapt  = self.create_publisher(Int32MultiArray, TOPIC_LEFT_HAPTIC,      10)
        self._ping_pub_right_hapt = self.create_publisher(Int32MultiArray, TOPIC_RIGHT_HAPTIC,     10)
        self._ping_pub_left_off   = self.create_publisher(Empty,           TOPIC_LEFT_HAPTIC_OFF,  10)
        self._ping_pub_right_off  = self.create_publisher(Empty,           TOPIC_RIGHT_HAPTIC_OFF, 10)

        # ping 발행 중임을 표시 — 자신의 콜백이 이 플래그를 소모해 OSC 생략
        self._ping_active = {
            'left_haptic':  False,
            'right_haptic': False,
            'left_off':     False,
            'right_off':    False,
        }

        # ── 연결 상태 캐시 ────────────────────────────────────────────
        self._left_connected  = False
        self._right_connected = False

        # ── 햅틱 on 구독자 (Int32MultiArray [thumb, index, middle, ring, pinky]) ──
        self._left_hapt_sub = self.create_subscription(
            Int32MultiArray, TOPIC_LEFT_HAPTIC, self._cb_left_haptic, 10
        )
        self._right_hapt_sub = self.create_subscription(
            Int32MultiArray, TOPIC_RIGHT_HAPTIC, self._cb_right_haptic, 10
        )

        # ── 햅틱 off 트리거 구독자 (Empty) ───────────────────────────
        self._left_hapt_off_sub = self.create_subscription(
            Empty, TOPIC_LEFT_HAPTIC_OFF, self._cb_left_haptic_off, 10
        )
        self._right_hapt_off_sub = self.create_subscription(
            Empty, TOPIC_RIGHT_HAPTIC_OFF, self._cb_right_haptic_off, 10
        )

        # ── OSC 서버 및 백그라운드 루프 시작 ─────────────────────────
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
        self.get_logger().info(
            f'햅틱 topic ping 간격: {HAPTIC_PING_INTERVAL_SEC}s'
        )

    # ── OSC 초기화 ───────────────────────────────────────────────────

    def _setup_osc(self) -> None:
        disp = osc_dispatcher.Dispatcher()

        disp.map('/left/quat/get',      self._h_left_quat)
        disp.map('/right/quat/get',     self._h_right_quat)
        disp.map('/device/status/get',  self._h_device_status)
        disp.map('/left/hapt/get',      self._h_left_hapt)
        disp.map('/right/hapt/get',     self._h_right_hapt)
        disp.map('/left/hapt/ret/get',  self._h_left_hapt_ret)
        disp.map('/right/hapt/ret/get', self._h_right_hapt_ret)
        disp.map('/device/alarm/get',   self._h_device_alarm)
        disp.map('/heartbeat/get',      self._h_heartbeat)
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

        threading.Thread(
            target=self._heartbeat_loop, daemon=True, name='OSCHeartbeat'
        ).start()
        threading.Thread(
            target=self._haptic_ping_loop, daemon=True, name='HapticPing'
        ).start()

    # ── OSC 파싱 헬퍼 ────────────────────────────────────────────────

    def _parse_floats(self, args: tuple, count: int):
        if len(args) < count + 1:
            return None
        try:
            return np.array(args[1:count + 1], dtype=np.float32)
        except (TypeError, ValueError):
            return None

    def _to_17_sensors(self, raw: np.ndarray) -> np.ndarray:
        """18 관절 쿼터니언 (72 floats) → 17 센서 쿼터니언 (68 floats).
        AGA_SKIP_JOINT (pinky_0, index 14) 제거.
        """
        return np.delete(raw.reshape(18, 4), AGA_SKIP_JOINT, axis=0).flatten()

    def _publish_quat(self, pub, raw: np.ndarray, side: str) -> None:
        msg = Float32MultiArray()
        msg.data = self._to_17_sensors(raw).tolist()
        pub.publish(msg)
        if self._verbose:
            self.get_logger().debug(
                f'[OSC] <- {side} quat  raw[:4]={raw[:4].round(3)}'
            )

    # ── 햅틱 피드백 송신 ─────────────────────────────────────────────

    def _build_haptic_packet(self, msg_type: str, values: list) -> list:
        """햅틱 OSC 패킷 생성. 형식: [msg_type, idx, val, idx, val, ...]"""
        packet: list = [msg_type]
        for i, v in enumerate(values[:AGA_FINGER_COUNT]):
            packet.append(i)
            packet.append(int(v))
        return packet

    def _send_haptic(self, side: str, values: list) -> None:
        """OSC 햅틱 명령을 즉시 1회 전송.

        패킷 구조: [msg_type_str, idx(int), val(int), ...]
          msg_type: "9" (왼손) / "10" (오른손), idx: 0~4, val: int
        """
        if side == 'left':
            address  = OSC_ADDR_LEFT_HAPT
            msg_type = OSC_MSG_TYPE_LEFT_HAPT
            connected = self._left_connected
        else:
            address  = OSC_ADDR_RIGHT_HAPT
            msg_type = OSC_MSG_TYPE_RIGHT_HAPT
            connected = self._right_connected

        if not connected:
            self.get_logger().warn(f'[OSC] {side} 손 미연결 — 햅틱 전송 생략')
            return

        packet = self._build_haptic_packet(msg_type, values)
        try:
            self._udp_client.send_message(address, packet)
            self.get_logger().info(
                f'[OSC] -> {side} hapt  values={[int(v) for v in values[:AGA_FINGER_COUNT]]}'
            )
        except Exception as e:
            self.get_logger().warn(f'[OSC] {side} 햅틱 전송 실패: {e}')

    # ── 햅틱 ROS 2 콜백 ──────────────────────────────────────────────

    def _cb_left_haptic(self, msg: Int32MultiArray) -> None:
        """/left_hand/haptic 수신 → 왼손 햅틱 on. ping이면 무시."""
        if self._ping_active['left_haptic']:
            self._ping_active['left_haptic'] = False
            return
        if len(msg.data) < AGA_FINGER_COUNT:
            self.get_logger().warn(
                f'[ROS] /left_hand/haptic 데이터 부족 '
                f'({len(msg.data)}/{AGA_FINGER_COUNT})'
            )
            return
        self._send_haptic('left', list(msg.data))

    def _cb_right_haptic(self, msg: Int32MultiArray) -> None:
        """/right_hand/haptic 수신 → 오른손 햅틱 on. ping이면 무시."""
        if self._ping_active['right_haptic']:
            self._ping_active['right_haptic'] = False
            return
        if len(msg.data) < AGA_FINGER_COUNT:
            self.get_logger().warn(
                f'[ROS] /right_hand/haptic 데이터 부족 '
                f'({len(msg.data)}/{AGA_FINGER_COUNT})'
            )
            return
        self._send_haptic('right', list(msg.data))

    def _cb_left_haptic_off(self, _: Empty) -> None:
        """/left_hand/haptic/off 수신 → 왼손 햅틱 끄기. ping이면 무시."""
        if self._ping_active['left_off']:
            self._ping_active['left_off'] = False
            return
        self._send_haptic('left', [0] * AGA_FINGER_COUNT)

    def _cb_right_haptic_off(self, _: Empty) -> None:
        """/right_hand/haptic/off 수신 → 오른손 햅틱 끄기. ping이면 무시."""
        if self._ping_active['right_off']:
            self._ping_active['right_off'] = False
            return
        self._send_haptic('right', [0] * AGA_FINGER_COUNT)

    # ── OSC 수신 핸들러 ──────────────────────────────────────────────

    def _h_left_quat(self, address: str, *args) -> None:
        raw = self._parse_floats(args, AGA_RAW_FLOAT_COUNT)
        if raw is None:
            self.get_logger().warn(
                f'[OSC] {address} 파싱 실패 (수신 args={len(args)}, '
                f'필요 {AGA_RAW_FLOAT_COUNT + 1})'
            )
            return
        if not self._left_connected:
            self._left_connected = True
            self._left_conn_pub.publish(Bool(data=True))
            self.get_logger().info('[OSC] 왼손 연결 상태 변경: True')
        self._publish_quat(self._left_pub, raw, 'left')

    def _h_right_quat(self, address: str, *args) -> None:
        raw = self._parse_floats(args, AGA_RAW_FLOAT_COUNT)
        if raw is None:
            self.get_logger().warn(
                f'[OSC] {address} 파싱 실패 (수신 args={len(args)}, '
                f'필요 {AGA_RAW_FLOAT_COUNT + 1})'
            )
            return
        if not self._right_connected:
            self._right_connected = True
            self._right_conn_pub.publish(Bool(data=True))
            self.get_logger().info('[OSC] 오른손 연결 상태 변경: True')
        self._publish_quat(self._right_pub, raw, 'right')

    def _h_device_status(self, address: str, *args) -> None:
        """디바이스 연결 상태 수신 및 발행.

        args[1]=left_connected(bool), args[2]=right_connected(bool)
        """
        if len(args) < 3:
            self.get_logger().debug(f'[OSC] {address} args 부족: {len(args)}')
            return

        left_ok  = bool(args[1])
        right_ok = bool(args[2])

        if left_ok != self._left_connected:
            self._left_connected = left_ok
            self.get_logger().info(f'[OSC] 왼손 연결 상태 변경: {left_ok}')
        if right_ok != self._right_connected:
            self._right_connected = right_ok
            self.get_logger().info(f'[OSC] 오른손 연결 상태 변경: {right_ok}')

        self._left_conn_pub.publish(Bool(data=left_ok))
        self._right_conn_pub.publish(Bool(data=right_ok))

        if self._verbose:
            self.get_logger().debug(
                f'[OSC] device status  left={left_ok}  right={right_ok}'
            )

    def _h_left_hapt(self, address: str, *args) -> None:
        if self._verbose:
            self.get_logger().debug(f'[OSC] <- {address}  args={args}')

    def _h_right_hapt(self, address: str, *args) -> None:
        if self._verbose:
            self.get_logger().debug(f'[OSC] <- {address}  args={args}')

    def _h_left_hapt_ret(self, address: str, *args) -> None:
        if self._verbose:
            self.get_logger().debug(f'[OSC] <- {address}  args={args}')

    def _h_right_hapt_ret(self, address: str, *args) -> None:
        if self._verbose:
            self.get_logger().debug(f'[OSC] <- {address}  args={args}')

    def _h_device_alarm(self, address: str, *args) -> None:
        self.get_logger().warn(f'[OSC] <- {address}  args={args}')

    def _h_heartbeat(self, address: str, *args) -> None:
        if self._verbose:
            self.get_logger().debug(f'[OSC] <- {address}  args={args}')

    def _h_default(self, address: str, *args) -> None:
        if self._verbose:
            self.get_logger().debug(f'[OSC] 미처리 주소: {address}  args={args}')

    # ── 하트비트 루프 ─────────────────────────────────────────────────

    def _heartbeat_loop(self) -> None:
        while self._running:
            try:
                self._udp_client.send_message('/device/status/get', '4')
                if self._verbose:
                    self.get_logger().debug('[OSC] -> heartbeat sent')
            except Exception as e:
                self.get_logger().warn(f'[OSC] heartbeat 전송 실패: {e}')
            time.sleep(HEARTBEAT_INTERVAL_SEC)

    # ── 햅틱 topic ping 루프 (DDS subscription 웜업) ─────────────────

    def _haptic_ping_loop(self) -> None:
        """햅틱/off 토픽에 주기적으로 fake 메시지를 발행해 DDS subscription을 유지한다.
        콜백에서 _ping_active 플래그를 확인해 OSC 전송을 건너뛴다.
        """
        zero_msg = Int32MultiArray(data=[0] * AGA_FINGER_COUNT)
        empty_msg = Empty()
        while self._running:
            self._ping_active['left_haptic']  = True
            self._ping_pub_left_hapt.publish(zero_msg)
            self._ping_active['right_haptic'] = True
            self._ping_pub_right_hapt.publish(zero_msg)
            self._ping_active['left_off']  = True
            self._ping_pub_left_off.publish(empty_msg)
            self._ping_active['right_off'] = True
            self._ping_pub_right_off.publish(empty_msg)
            time.sleep(HAPTIC_PING_INTERVAL_SEC)

    # ── 종료 ─────────────────────────────────────────────────────────

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
