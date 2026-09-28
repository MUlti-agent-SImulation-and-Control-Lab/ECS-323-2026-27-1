#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleBotMover(Node):

    def __init__(self):
        super().__init__('turtlebot_mover')

        self.robot1_pub = self.create_publisher(
            Twist,
            '/robot1/diffdrive_controller/cmd_vel_unstamped',
            10
        )
        
        self.robot2_pub = self.create_publisher(
            Twist,
            '/robot2/diffdrive_controller/cmd_vel_unstamped',
            10
        )
        
        self.robot3_pub = self.create_publisher(
            Twist,
            '/robot3/diffdrive_controller/cmd_vel_unstamped',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.move_robot
        )

    def move_robot(self):

	    msg = Twist()
	    msg.linear.x = 0.2
	    msg.angular.z = 1.0

	    self.robot1_pub.publish(msg)
	    self.robot2_pub.publish(msg)
	    self.robot3_pub.publish(msg)
    	
def main():

    rclpy.init()

    node = TurtleBotMover()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
