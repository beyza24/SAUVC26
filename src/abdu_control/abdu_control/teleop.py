"""ABDÜ ROV için 6 serbestlik dereceli klavye teleop."""

from __future__ import annotations

import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
ABDÜ ROV teleop
----------------
  w/s     ileri / geri          (surge)
  a/d     sola / sağa sap       (yaw)
  r/f     yüksel / alçal        (heave)
  q/e     sola / sağa kay       (sway)
  t/g     burun yukarı / aşağı  (pitch)
  y/h     yatış                 (roll)
  boşluk  dur
  CTRL-C  çıkış

Hız çarpanı: 1-5 (1 yavaş, 5 hızlı)
"""


class RovTeleop(Node):
    def __init__(self) -> None:
        super().__init__("abdu_teleop")
        self.pub = self.create_publisher(Twist, "/abdu/cmd_vel", 10)
        self.scale = 2
        self.cmd = Twist()
        self.get_logger().info(HELP)

    def publish(self) -> None:
        s = 0.25 * self.scale
        out = Twist()
        out.linear.x = self.cmd.linear.x * s * 1.2
        out.linear.y = self.cmd.linear.y * s
        out.linear.z = self.cmd.linear.z * s * 0.5
        out.angular.x = self.cmd.angular.x * s * 0.4
        out.angular.y = self.cmd.angular.y * s * 0.4
        out.angular.z = self.cmd.angular.z * s * 0.9
        self.pub.publish(out)


def _getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def main() -> None:
    rclpy.init()
    node = RovTeleop()
    print(HELP)
    try:
        while rclpy.ok():
            key = _getch()
            if key == "\x03":
                break
            node.cmd = Twist()
            mapping = {
                "w": ("linear", "x", 1.0),
                "s": ("linear", "x", -1.0),
                "a": ("angular", "z", 1.0),
                "d": ("angular", "z", -1.0),
                "r": ("linear", "z", 1.0),
                "f": ("linear", "z", -1.0),
                "q": ("linear", "y", 1.0),
                "e": ("linear", "y", -1.0),
                "t": ("angular", "y", 1.0),
                "g": ("angular", "y", -1.0),
                "y": ("angular", "x", 1.0),
                "h": ("angular", "x", -1.0),
            }
            if key in mapping:
                kind, axis, value = mapping[key]
                setattr(getattr(node.cmd, kind), axis, value)
            elif key in "12345":
                node.scale = int(key)
                print(f"hız çarpanı: {node.scale}")
            node.publish()
    except KeyboardInterrupt:
        pass
    node.cmd = Twist()
    node.publish()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
