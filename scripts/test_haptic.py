#!/usr/bin/env python3
"""
scripts/test_haptic_left.py
왼손 햅틱 수동 테스트

  1  — 전체 손가락 햅틱 ON
  2  — 햅틱 OFF
  q  — 종료
"""

import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Int32MultiArray

sys.path.insert(0, __import__('os').path.join(__import__('os').path.dirname(__file__), '..'))

from atlas_hand.config import TOPIC_LEFT_HAPTIC, TOPIC_LEFT_HAPTIC_OFF, TOPIC_RIGHT_HAPTIC_OFF,TOPIC_RIGHT_HAPTIC
from atlas_hand_core.config import AGA_FINGER_COUNT

HAPTIC_INTENSITY = 1  # 0~255


class HapticTestNode(Node):

    def __init__(self,side="left"):
        super().__init__('haptic_test_left')
        
        self._pub_on  = self.create_publisher(Int32MultiArray, TOPIC_LEFT_HAPTIC if side=="left" else TOPIC_RIGHT_HAPTIC,     10)
        self._pub_off = self.create_publisher(Empty,           TOPIC_LEFT_HAPTIC_OFF if side=="left" else TOPIC_RIGHT_HAPTIC_OFF, 10)

    def on(self, intensity: int = HAPTIC_INTENSITY):
        self._pub_on.publish(Int32MultiArray(data=[intensity] * AGA_FINGER_COUNT))
        self.get_logger().info(f"[ON]  {AGA_FINGER_COUNT}fingers × {intensity}")

    def off(self):
        self._pub_off.publish(Empty())
        self.get_logger().info("[OFF]")


def _getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main(args=None):
    rclpy.init(args=args)
    left_node = HapticTestNode()
    right_node = HapticTestNode("right")

    threading.Thread(target=rclpy.spin, args=(left_node,), daemon=True).start()
    threading.Thread(target=rclpy.spin, args=(right_node,), daemon=True).start()

    print("=" * 35)
    print("  햅틱 테스트")
    
    print(" left hand")
    print("1 → level 1 \n2 → level 2 \n3 → level3 \nq → 종료")
    print(" right hand")
    print("a → level 1 \ns → level 2 \nd → level3 \nq → 종료")
    print("=" * 35)

    try:
        while rclpy.ok():
            ch = _getch()
            if ch == '1':
                left_node.on(1)
            elif ch == '2':
                left_node.on(2)
            elif ch == '3':
                left_node.on(3)
            elif ch == 'a':
                right_node.on(1)
            elif ch == 's':
                right_node.on(2)
            elif ch == 'd':
                right_node.on(3)

            elif ch in ('q', 'Q', '\x03'):
                break
    finally:
        left_node.off()
        left_node.destroy_node()
        
        right_node.off()
        right_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
