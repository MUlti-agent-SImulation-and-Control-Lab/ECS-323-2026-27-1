import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')
        if os.path.exists('/dev/ttyUSB1'):
            self.arduino = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
        else:
            self.arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        time.sleep(2)

        self.last_cmd = None  # 🔥 store last command

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10)

    def cmd_callback(self, msg):

        linear = msg.linear.x
        angular = msg.angular.z

        if linear > 0.3:
            cmd = 'w'
        elif linear < -0.3:
            cmd = 's'
        elif angular > 0.3:
            cmd = 'a'
        elif angular < -0.3:
            cmd = 'd'
        else:
            cmd = 'x'

        # 🔥 Only send if changed
        if cmd != self.last_cmd:
            self.send_cmd(cmd)
            self.last_cmd = cmd

    def send_cmd(self, cmd):
        try:
            self.arduino.write((cmd + '\n').encode())
            self.get_logger().info(f"Sending {cmd}")
        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
