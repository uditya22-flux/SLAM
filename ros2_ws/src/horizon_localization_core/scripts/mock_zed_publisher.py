#!/usr/bin/env python3
"""
Mock ZED Camera Publisher for ROS 2.
Generates synthetic ZED 2 RGB images containing an ArUco marker and publishes CameraInfo.
Used for verification and hardware-free simulation.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class MockZedPublisher(Node):

    def __init__(self):
        super().__init__("mock_zed_publisher")

        self.declare_parameter("image_topic", "/zed/zed_node/rgb/image_rect_color")
        self.declare_parameter("camera_info_topic", "/zed/zed_node/rgb/camera_info")
        self.declare_parameter("frame_id", "zed_left_camera_frame")
        self.declare_parameter("tag_id", 42)

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.tag_id = self.get_parameter("tag_id").get_parameter_value().integer_value

        self.pub_image = self.create_publisher(Image, self.image_topic, 10)
        self.pub_info = self.create_publisher(CameraInfo, self.camera_info_topic, 10)
        self.bridge = CvBridge()

        # Timer to publish at 15 Hz
        self.timer = self.create_timer(1.0 / 15.0, self.publish_frame)

        # Image resolution & camera intrinsics (HD720 ZED parameters)
        self.width = 1280
        self.height = 720
        self.fx = 700.0
        self.fy = 700.0
        self.cx = 640.0
        self.cy = 360.0

        # Generate ArUco marker image
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        if hasattr(cv2.aruco, "generateImageMarker"):
            self.marker_img = cv2.aruco.generateImageMarker(dictionary, self.tag_id, 200)
        else:
            self.marker_img = cv2.aruco.drawMarker(dictionary, self.tag_id, 200)

        self.angle = 0.0
        self.get_logger().info(f"Mock ZED Publisher broadcasting on {self.image_topic} (Tag ID: {self.tag_id})")

    def publish_frame(self):
        stamp = self.get_clock().now().to_msg()

        # Build CameraInfo msg
        info_msg = CameraInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = self.frame_id
        info_msg.width = self.width
        info_msg.height = self.height
        info_msg.distortion_model = "plumb_bob"
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info_msg.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        info_msg.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        # Build Synthetic Canvas
        canvas = np.ones((self.height, self.width, 3), dtype=np.uint8) * 220

        # Oscillate marker position slightly to simulate movement
        self.angle += 0.05
        offset_x = int(50 * np.sin(self.angle))
        offset_y = int(30 * np.cos(self.angle * 0.7))

        x_start = 540 + offset_x
        y_start = 260 + offset_y

        marker_bgr = cv2.cvtColor(self.marker_img, cv2.COLOR_GRAY2BGR)
        canvas[y_start:y_start + 200, x_start:x_start + 200] = marker_bgr

        # Convert to ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.frame_id

        self.pub_info.publish(info_msg)
        self.pub_image.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockZedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
