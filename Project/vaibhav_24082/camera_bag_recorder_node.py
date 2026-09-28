#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

import subprocess
import os
import time
import signal

from datetime import datetime


class CameraBagRecorder(Node):

    def __init__(self):

        super().__init__('camera_bag_recorder')

        # ==================================
        # SETTINGS
        # ==================================

        # A button on Xbox controller
        self.record_button = 0

        self.save_directory = os.path.expanduser(
            '~/Desktop/ROS_Autonomous/realsense_bags'
        )

        # ==================================

        os.makedirs(self.save_directory, exist_ok=True)

        self.recording = False

        self.bag_process = None
        self.realsense_process = None

        self.last_button_state = 0

        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.get_logger().info(
            "Press A to Start/Stop RealSense Recording"
        )

    def joy_callback(self, msg):

        if len(msg.buttons) <= self.record_button:
            return

        button_state = msg.buttons[self.record_button]

        # Detect button press edge
        if button_state == 1 and self.last_button_state == 0:

            if not self.recording:
                self.start_recording()
            else:
                self.stop_recording()

        self.last_button_state = button_state

    def wait_for_camera_topics(self):

        self.get_logger().info(
            "Waiting for RealSense topics..."
        )

        timeout = 20
        start_time = time.time()

        while time.time() - start_time < timeout:

            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True
            )

            topics = result.stdout

            if (
                '/camera/camera/color/image_raw' in topics and
                '/camera/camera/depth/image_rect_raw' in topics
            ):
                self.get_logger().info(
                    "RealSense topics detected."
                )
                return True

            time.sleep(1)

        self.get_logger().error(
            "Timed out waiting for RealSense topics."
        )

        return False

    def start_recording(self):

        self.get_logger().info(
            "Launching RealSense Camera..."
        )

        self.realsense_process = subprocess.Popen(
            [
                'ros2',
                'launch',
                'realsense2_camera',
                'rs_launch.py',
                'enable_color:=true',
                'enable_depth:=true'
            ],
            preexec_fn=os.setsid
        )

        if not self.wait_for_camera_topics():

            self.get_logger().error(
                "Camera failed to start."
            )

            return

        timestamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )

        bag_name = f"realsense_record_{timestamp}"

        bag_path = os.path.join(
            self.save_directory,
            bag_name
        )

        self.get_logger().info(
            f"Recording to: {bag_path}"
        )

        self.bag_process = subprocess.Popen(
            [
                'ros2',
                'bag',
                'record',

                '/camera/camera/color/image_raw',
                '/camera/camera/depth/image_rect_raw',
                '/joy',

                '-o',
                bag_path
            ],
            preexec_fn=os.setsid
        )

        self.recording = True

        self.get_logger().info(
            "Recording Started"
        )

    def stop_recording(self):

        self.get_logger().info(
            "Stopping recording..."
        )

        # -------------------------
        # Stop rosbag
        # -------------------------

        if self.bag_process is not None:

            try:

                os.killpg(
                    os.getpgid(self.bag_process.pid),
                    signal.SIGINT
                )

                self.bag_process.wait(timeout=15)

            except Exception as e:

                self.get_logger().error(
                    f"Error stopping rosbag: {e}"
                )

            self.bag_process = None

        self.get_logger().info(
            "Rosbag saved successfully."
        )

        # -------------------------
        # Stop RealSense
        # -------------------------

        if self.realsense_process is not None:

            try:

                self.get_logger().info(
                    "Stopping RealSense camera..."
                )

                os.killpg(
                    os.getpgid(self.realsense_process.pid),
                    signal.SIGINT
                )

                self.realsense_process.wait(timeout=15)

            except Exception as e:

                self.get_logger().error(
                    f"Error stopping camera: {e}"
                )

            self.realsense_process = None

        self.recording = False

        self.get_logger().info(
            "RealSense camera stopped."
        )

    def destroy_node(self):

        if self.recording:
            self.stop_recording()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = CameraBagRecorder()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
