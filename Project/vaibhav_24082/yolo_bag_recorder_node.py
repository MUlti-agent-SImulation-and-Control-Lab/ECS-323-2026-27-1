
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, Joy
from cv_bridge import CvBridge

from ultralytics import YOLO

from rclpy.qos import qos_profile_sensor_data

import cv2
import os
import subprocess
import time
import signal

from datetime import datetime


class YoloBagRecorder(Node):

    def __init__(self):

        super().__init__('yolo_bag_recorder')

        # ==================================================
        # SETTINGS
        # ==================================================

        self.record_button = 1

        self.camera_topic = '/camera/camera/color/image_raw'

        self.video_directory = os.path.expanduser(
            '~/Desktop/ROS_Autonomous/yolo_videos'
        )

        self.bag_directory = os.path.expanduser(
            '~/Desktop/ROS_Autonomous/yolo_bags'
        )

        os.makedirs(self.video_directory, exist_ok=True)
        os.makedirs(self.bag_directory, exist_ok=True)

        # ==================================================
        # VARIABLES
        # ==================================================

        self.bridge = CvBridge()

        self.recording = False

        self.video_writer = None

        self.bag_process = None

        self.realsense_process = None

        self.model = None

        self.last_button_state = 0

        self.video_path = None

        # ==================================================
        # LOAD YOLO MODEL ONCE
        # ==================================================

        self.get_logger().info("Loading YOLO Model...")

        self.model = YOLO(
            '/home/cypher/Desktop/ROS_Autonomous/yolov8n.pt'
        )

        self.get_logger().info("YOLO Model Loaded")

        # ==================================================
        # PUBLISHER
        # ==================================================

        self.detection_publisher = self.create_publisher(
            Image,
            '/yolo/detection_image',
            10
        )

        # ==================================================
        # SUBSCRIBERS
        # ==================================================

        self.image_subscriber = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.get_logger().info("YOLO Bag Recorder Ready")

    # ==================================================
    # JOYSTICK CALLBACK
    # ==================================================

    def joy_callback(self, msg):

        button_state = msg.buttons[self.record_button]

        # SINGLE PRESS DETECTION
        if button_state == 1 and self.last_button_state == 0:

            if not self.recording:

                self.start_recording()

            else:

                self.stop_recording()

        self.last_button_state = button_state

    # ==================================================
    # START RECORDING
    # ==================================================

    def start_recording(self):

        if self.recording:
            return

        self.get_logger().info("Starting Recording System...")

        # ==================================================
        # START REALSENSE CAMERA
        # ==================================================

        self.get_logger().info("Starting RealSense Camera...")

        self.realsense_process = subprocess.Popen(
            [
                'ros2',
                'launch',
                'realsense2_camera',
                'rs_launch.py',
                'enable_color:=true'
            ]
        )

        # WAIT FOR CAMERA START
        time.sleep(5)

        self.get_logger().info("RealSense Camera Started")

        # ==================================================
        # TIMESTAMP
        # ==================================================

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # ==================================================
        # VIDEO PATH
        # ==================================================

        video_name = f"yolo_detection_{timestamp}.avi"

        self.video_path = os.path.join(
            self.video_directory,
            video_name
        )

        # ==================================================
        # BAG PATH
        # ==================================================

        bag_name = f"yolo_bag_{timestamp}"

        bag_path = os.path.join(
            self.bag_directory,
            bag_name
        )

        # ==================================================
        # START ROSBAG
        # ==================================================

        command = [
            'ros2',
            'bag',
            'record',

            self.camera_topic,

            '/yolo/detection_image',

            '/joy',

            '-o',
            bag_path
        ]

        self.bag_process = subprocess.Popen(command)

        self.recording = True

        self.get_logger().info(
            f"\nStarted Recording\n"
            f"Video: {self.video_path}\n"
            f"Bag: {bag_path}\n"
        )

    # ==================================================
    # STOP RECORDING
    # ==================================================

    def stop_recording(self):

        if not self.recording:
            return

        self.get_logger().info("Stopping Recording System...")

        self.recording = False

        # ==================================================
        # RELEASE VIDEO WRITER
        # ==================================================

        if self.video_writer is not None:

            self.video_writer.release()

            self.video_writer = None

            self.get_logger().info("Video Saved Successfully")

        # ==================================================
        # STOP ROSBAG
        # ==================================================

        if self.bag_process is not None:

            self.get_logger().info("Stopping Rosbag...")

            self.bag_process.terminate()

            self.bag_process.wait()

            self.bag_process = None

        # ==================================================
        # WAIT FOR BAG FLUSH
        # ==================================================

        time.sleep(1)

        # ==================================================
        # STOP REALSENSE
        # ==================================================

        if self.realsense_process is not None:

            self.get_logger().info("Stopping RealSense Camera...")

            self.realsense_process.send_signal(
                signal.SIGINT
            )

            self.realsense_process.wait()

            self.realsense_process = None

        # ==================================================
        # CLOSE WINDOWS
        # ==================================================

      #  cv2.destroyAllWindows()

        self.get_logger().info("Recording Stopped")

    # ==================================================
    # IMAGE CALLBACK
    # ==================================================

    def image_callback(self, msg):

        if not self.recording:
            return

        try:

            # ==================================================
            # ROS IMAGE -> CV2
            # ==================================================

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            # ==================================================
            # RESIZE FOR PERFORMANCE
            # ==================================================

            frame = cv2.resize(
                frame,
                (640, 480)
            )

            # ==================================================
            # YOLO INFERENCE
            # ==================================================

            results = self.model(
                frame,
                verbose=False
            )

            # ==================================================
            # DRAW DETECTIONS
            # ==================================================

            annotated_frame = results[0].plot()

            # ==================================================
            # DISPLAY WINDOW
            # ==================================================

           # cv2.imshow(
           #     "YOLO Detection",
           #     annotated_frame
           # )

           # cv2.waitKey(1)

            # ==================================================
            # PUBLISH DETECTION IMAGE
            # ==================================================

            detection_msg = self.bridge.cv2_to_imgmsg(
                annotated_frame,
                encoding='bgr8'
            )

            self.detection_publisher.publish(
                detection_msg
            )

            # ==================================================
            # CREATE VIDEO WRITER
            # ==================================================

            if self.video_writer is None:

                height, width, _ = annotated_frame.shape

                fourcc = cv2.VideoWriter_fourcc(*'XVID')

                self.video_writer = cv2.VideoWriter(
                    self.video_path,
                    fourcc,
                    20.0,
                    (width, height)
                )

                # VERIFY VIDEO WRITER

                if not self.video_writer.isOpened():

                    self.get_logger().error(
                        "FAILED TO OPEN VIDEO WRITER"
                    )

                else:

                    self.get_logger().info(
                        "Video Writer Opened"
                    )

            # ==================================================
            # WRITE VIDEO FRAME
            # ==================================================

            if self.video_writer is not None:

                self.video_writer.write(
                    annotated_frame
                )

        except Exception as e:

            self.get_logger().error(
                f"Image Callback Error: {e}"
            )

    # ==================================================
    # CLEAN SHUTDOWN
    # ==================================================

    def destroy_node(self):

        self.get_logger().info(
            "Shutting Down Node..."
        )

        self.stop_recording()

        #cv2.destroyAllWindows()

        super().destroy_node()


# ==================================================
# MAIN
# ==================================================

def main(args=None):

    rclpy.init(args=args)

    node = YoloBagRecorder()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()


