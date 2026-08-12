#!/usr/bin/env python3
"""
ArUco / AprilTag Detection and 6-DOF Pose Estimation Node for ROS 2.
Author: Horizon Localization Team (Person 2 Deliverable)

Deliverable: Given a camera (e.g. ZED) image containing a tag, this node reliably
outputs tag_id and 3D 6-DOF pose relative to the camera frame, broadcasts TF transforms,
and publishes visualization markers for RViz.
"""

import json
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

import tf2_ros
from scipy.spatial.transform import Rotation as R


class ArucoTagDetector(Node):
    # Dictionary lookup map for standard ArUco and AprilTag dictionaries
    ARUCO_DICTIONARIES = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
        "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
        "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
        "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
        "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
        "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
        "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
        "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
        "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
    }

    def __init__(self):
        super().__init__("aruco_tag_detector")

        # Declare parameters
        self.declare_parameter("image_topic", "/zed/zed_node/rgb/image_rect_color")
        self.declare_parameter("camera_info_topic", "/zed/zed_node/rgb/camera_info")
        self.declare_parameter("dictionary_name", "DICT_5X5_100")
        self.declare_parameter("marker_size", 0.15)  # Marker side length in meters
        self.declare_parameter("camera_frame_override", "")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("tf_prefix", "tag_")

        # Read parameters
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.dictionary_name = self.get_parameter("dictionary_name").get_parameter_value().string_value
        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        self.camera_frame_override = self.get_parameter("camera_frame_override").get_parameter_value().string_value
        self.publish_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value
        self.tf_prefix = self.get_parameter("tf_prefix").get_parameter_value().string_value

        self.get_logger().info(f"Initializing ArUco Tag Detector node...")
        self.get_logger().info(f"Image Topic: {self.image_topic}")
        self.get_logger().info(f"Camera Info Topic: {self.camera_info_topic}")
        self.get_logger().info(f"Dictionary: {self.dictionary_name}, Marker Size: {self.marker_size}m")

        # OpenCV ArUco Dictionary & Parameters Setup
        dict_enum = self.ARUCO_DICTIONARIES.get(self.dictionary_name, cv2.aruco.DICT_5X5_100)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_enum)
        
        # Detector Parameters setup (compatible across OpenCV versions)
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()

        # Detector object if modern OpenCV is available
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.detector = None

        self.bridge = CvBridge()

        # Camera intrinsics storage
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame_id = None

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # QoS configuration
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        self.sub_info = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile
        )

        self.sub_image = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile
        )

        # Publishers
        self.pub_poses = self.create_publisher(PoseArray, "/localization/tag_detections", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/localization/tag_markers", 10)
        self.pub_debug_img = self.create_publisher(Image, "/localization/tag_debug_image", 10)
        self.pub_json_data = self.create_publisher(String, "/localization/tag_detections_json", 10)

    def camera_info_callback(self, msg: CameraInfo):
        """Extract camera intrinsic matrix and distortion coefficients."""
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        
        if self.camera_frame_override:
            self.camera_frame_id = self.camera_frame_override
        else:
            self.camera_frame_id = msg.header.frame_id if msg.header.frame_id else "zed_camera_frame"

    def image_callback(self, msg: Image):
        """Process incoming image, detect tags, estimate 6-DOF pose, and publish outputs."""
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warning("Waiting for CameraInfo intrinsics...", throttle_duration_sec=3.0)
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect markers
        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        pose_array = PoseArray()
        pose_array.header.stamp = msg.header.stamp
        pose_array.header.frame_id = self.camera_frame_id

        marker_array = MarkerArray()
        detections_payload = []

        if ids is not None and len(ids) > 0:
            # 3D object points of marker corners in marker local frame
            half_s = self.marker_size / 2.0
            obj_points = np.array([
                [-half_s,  half_s, 0.0],
                [ half_s,  half_s, 0.0],
                [ half_s, -half_s, 0.0],
                [-half_s, -half_s, 0.0]
            ], dtype=np.float64)

            # Draw detected markers on debug image
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            for i in range(len(ids)):
                tag_id = int(ids[i][0])
                corner = corners[i][0]

                # 6-DOF Pose Estimation via solvePnP
                success, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    corner,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if not success:
                    continue

                rvec = rvec.reshape((3,))
                tvec = tvec.reshape((3,))

                # Convert rotation vector to quaternion
                rot_matrix, _ = cv2.Rodrigues(rvec)
                scipy_rot = R.from_matrix(rot_matrix)
                quat = scipy_rot.as_quat()  # Returns [x, y, z, w]

                # Create Pose
                tag_pose = Pose()
                tag_pose.position.x = float(tvec[0])
                tag_pose.position.y = float(tvec[1])
                tag_pose.position.z = float(tvec[2])
                tag_pose.orientation.x = float(quat[0])
                tag_pose.orientation.y = float(quat[1])
                tag_pose.orientation.z = float(quat[2])
                tag_pose.orientation.w = float(quat[3])

                pose_array.poses.append(tag_pose)

                dist = math.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)

                # Build JSON Payload entry for Person 3 Handoff
                detections_payload.append({
                    "tag_id": tag_id,
                    "frame_id": self.camera_frame_id,
                    "distance_m": round(dist, 4),
                    "pose": {
                        "position": {"x": round(tvec[0], 4), "y": round(tvec[1], 4), "z": round(tvec[2], 4)},
                        "orientation": {"x": round(quat[0], 4), "y": round(quat[1], 4), "z": round(quat[2], 4), "w": round(quat[3], 4)}
                    }
                })

                # Log Tag ID + 6-DOF pose
                self.get_logger().info(
                    f"[TAG DETECTED] Tag ID: {tag_id} | "
                    f"Pos (x,y,z): [{tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f}] m | "
                    f"Dist: {dist:.3f} m"
                )

                # Draw 3D axis on debug image
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size * 0.7)

                # Broadcast TF frame if enabled
                if self.publish_tf:
                    tf_msg = TransformStamped()
                    tf_msg.header.stamp = msg.header.stamp
                    tf_msg.header.frame_id = self.camera_frame_id
                    tf_msg.child_frame_id = f"{self.tf_prefix}{tag_id}"
                    tf_msg.transform.translation.x = tag_pose.position.x
                    tf_msg.transform.translation.y = tag_pose.position.y
                    tf_msg.transform.translation.z = tag_pose.position.z
                    tf_msg.transform.rotation = tag_pose.orientation
                    self.tf_broadcaster.sendTransform(tf_msg)

                # RViz Visualization Marker (Text ID + Box)
                text_marker = Marker()
                text_marker.header.stamp = msg.header.stamp
                text_marker.header.frame_id = self.camera_frame_id
                text_marker.ns = "aruco_tags_text"
                text_marker.id = tag_id
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose = tag_pose
                text_marker.pose.position.z += self.marker_size * 0.6
                text_marker.scale.z = 0.1
                text_marker.color.r = 0.0
                text_marker.color.g = 1.0
                text_marker.color.b = 0.0
                text_marker.color.a = 1.0
                text_marker.text = f"Tag {tag_id}"
                marker_array.markers.append(text_marker)

                cube_marker = Marker()
                cube_marker.header.stamp = msg.header.stamp
                cube_marker.header.frame_id = self.camera_frame_id
                cube_marker.ns = "aruco_tags_cube"
                cube_marker.id = tag_id + 10000
                cube_marker.type = Marker.CUBE
                cube_marker.action = Marker.ADD
                cube_marker.pose = tag_pose
                cube_marker.scale.x = float(self.marker_size)
                cube_marker.scale.y = float(self.marker_size)
                cube_marker.scale.z = 0.01
                cube_marker.color.r = 1.0
                cube_marker.color.g = 0.5
                cube_marker.color.b = 0.0
                cube_marker.color.a = 0.8
                marker_array.markers.append(cube_marker)

        # Publish PoseArray, MarkerArray, JSON Data, and Debug Image
        self.pub_poses.publish(pose_array)
        self.pub_markers.publish(marker_array)

        json_msg = String()
        json_msg.data = json.dumps(detections_payload)
        self.pub_json_data.publish(json_msg)

        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            debug_msg.header = msg.header
            self.pub_debug_img.publish(debug_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge publish error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
