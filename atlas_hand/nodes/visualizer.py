#!/usr/bin/env python3
"""
visualizer_node.py
Rerun 기반 Hand 3D 시각화 ROS 2 노드 (HandSphericalFK 사용)

사용법:
  ros2 run atlas_hand visualizer [hand_type] [mode]
  - mode:
    1. spawn   : 로컬에서 직접 Rerun Viewer 창을 띄움 (로컬 실행 시)
    2. connect : 외부(Mac 호스트) Viewer에 연결 (Docker 실행 시)
    3. off     : 시각화 기능을 끄고 연산만 수행
"""

import sys
import threading
import time
import os

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import rerun as rr
import rerun.blueprint as rrb
from ament_index_python.packages import get_package_share_directory

from atlas_hand.core.hand_spherical_fk import HandRerunViz

class HandVisualizerNode(Node):
    def __init__(self, hand_type: str = 'left', mode: str = 'spawn'):
        super().__init__('hand_visualizer_node')
        self.hand_type = hand_type.lower()
        self.mode = mode
        self.lock = threading.Lock()
        self.latest_quats = None
        self.msg_count = 0

        try:
            pkg_share = get_package_share_directory('atlas_hand')
        except Exception as e:
            self.get_logger().error(f"Failed to find atlas_hand package share: {e}")
            pkg_share = "."
            
        urdf_path = os.path.join(pkg_share, 'urdf', f'{self.hand_type}_hand', 'urdf', f'{self.hand_type}_hand_rerun.urdf')
        mesh_base_dir = os.path.join(pkg_share, 'urdf', f'{self.hand_type}_hand')

        if not os.path.exists(urdf_path):
            self.get_logger().warn(f"URDF path not found: {urdf_path}")

        self.viz = HandRerunViz(
            hand_type=self.hand_type,
            urdf_path=urdf_path,
            mesh_base_dir=mesh_base_dir,
        )

        if self.mode != 'off':
            self.viz.setup()

        self.get_logger().info(f"Hand Visualizer started. Hand: {self.hand_type.upper()}")
        self.create_subscription(
            Float32MultiArray,
            f'/{self.hand_type}_hand/quaternions',
            self._callback,
            10,
        )

    def _callback(self, msg: Float32MultiArray):
        if len(msg.data) != 68: return
        try:
            quats = np.array(msg.data).reshape(17, 4)
            with self.lock:
                self.latest_quats = quats
                self.msg_count += 1
        except Exception as e:
            self.get_logger().error(f"Callback Error: {e}")

def _ros_spin(node: Node):
    rclpy.spin(node)

def main(args=None):
    hand_type = 'left'
    mode = 'spawn' 

    if len(sys.argv) > 1:
        hand_type = 'right' if sys.argv[1].lower() in ['right', 'r'] else 'left'
    
    if len(sys.argv) > 2:
        arg_mode = sys.argv[2].lower()
        if arg_mode in ['spawn', 'true', 't']:
            mode = 'spawn'
        elif arg_mode in ['off', 'false', 'f', '0']:
            mode = 'off'
        else:
            mode = 'connect'

    # ROS 2 및 Rerun 초기화
    rclpy.init(args=args)

    if mode != 'off':
        print(f"Rerun Mode: {mode.upper()}")
        rr.init(f"AGA Hand ({hand_type.upper()})", spawn=(mode == 'spawn'))
        if mode == 'connect':
            rr.serve_grpc()

        # 뷰포트 설정
        blueprint = rrb.Blueprint(
            rrb.Horizontal(rrb.Spatial3DView(name="Hand Update View", origin="/world", contents=["/world/**", f"/hand/**"]))
        )
        rr.send_blueprint(blueprint)

    node = HandVisualizerNode(hand_type, mode)

    # ROS 스레드 시작
    thread = threading.Thread(target=_ros_spin, args=(node,), daemon=True)
    thread.start()

    # 메인 루프 (Rerun update 주기)
    try:
        while rclpy.ok():
            with node.lock:
                quats = node.latest_quats.copy() if node.latest_quats is not None else None

            if mode != 'off' and quats is not None:
                node.viz.update(quats)

            time.sleep(0.016) 

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()