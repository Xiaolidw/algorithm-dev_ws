"""Minimal persistent keyboard teleoperation for ROS 2.

Keys:
  W - forward, S - backward, A - turn left, D - turn right
  SPACE - stop, Q - stop and quit
"""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
Simple robot control
--------------------
  W: forward       S: backward
  A: turn left     D: turn right
  SPACE: stop      Q: stop and quit

Press a key once; the command remains active until the next key.
"""


class SimpleTeleop(Node):
    def __init__(self):
        super().__init__("simple_teleop")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 0.30)
        self.declare_parameter("angular_speed", 0.80)
        self.declare_parameter("publish_rate", 10.0)

        topic = self.get_parameter("cmd_vel_topic").value
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        rate = max(float(self.get_parameter("publish_rate").value), 1.0)

        self.publisher = self.create_publisher(Twist, topic, 10)
        self.command = Twist()
        self.timer = self.create_timer(1.0 / rate, self.publish_command)
        self.get_logger().info(
            f"Ready: topic={topic}, linear={self.linear_speed:.2f} m/s, "
            f"angular={self.angular_speed:.2f} rad/s"
        )

    def publish_command(self):
        self.publisher.publish(self.command)

    def set_motion(self, linear=0.0, angular=0.0):
        self.command = Twist()
        self.command.linear.x = linear
        self.command.angular.z = angular
        self.publisher.publish(self.command)

    def stop(self):
        self.set_motion()


def read_key():
    if select.select([sys.stdin], [], [], 0.0)[0]:
        return sys.stdin.read(1).lower()
    return None


def main(args=None):
    if not sys.stdin.isatty():
        raise RuntimeError("simple_teleop must run in an interactive terminal")

    rclpy.init(args=args)
    node = SimpleTeleop()
    old_settings = termios.tcgetattr(sys.stdin)
    print(HELP)

    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            key = read_key()
            if key == "w":
                node.set_motion(linear=node.linear_speed)
                print("Forward")
            elif key == "s":
                node.set_motion(linear=-node.linear_speed)
                print("Backward")
            elif key == "a":
                node.set_motion(angular=node.angular_speed)
                print("Turn left")
            elif key == "d":
                node.set_motion(angular=-node.angular_speed)
                print("Turn right")
            elif key == " ":
                node.stop()
                print("Stop")
            elif key == "q":
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        for _ in range(3):
            node.publisher.publish(node.command)
            rclpy.spin_once(node, timeout_sec=0.03)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Stopped")


if __name__ == "__main__":
    main()
