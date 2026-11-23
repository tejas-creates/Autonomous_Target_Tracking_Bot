#!/usr/bin/env python3
"""Detect the target checkerboard and publish its position relative to the AGV."""

import math
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraModule(Node):
    def __init__(self):
        super().__init__('camera_module')

        # Values that may be changed from a launch file without editing this file.
        self.image_topic = self.declare_parameter('image_topic', '/AGV/camera/img').value
        self.pattern = (
            self.declare_parameter('pattern_columns', 5).value,
            self.declare_parameter('pattern_rows', 3).value,
        )
        self.square_size = self.declare_parameter('square_size', 0.015).value
        self.horizontal_fov = self.declare_parameter('horizontal_fov', 1.78).value
        self.vertical_fov = self.declare_parameter('vertical_fov', 1.169).value
        self.camera_offset = np.array(
            self.declare_parameter('camera_offset', [0.0, 0.01, 0.114]).value
        )
        self.camera_pitch = self.declare_parameter('camera_pitch', 0.5236).value
        self.max_rmse = self.declare_parameter('max_reprojection_error', 5.0).value
        self.detection_rate = self.declare_parameter('detection_rate', 10.0).value
        self.show_image = self.declare_parameter('show_image', False).value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value

        # Known 3-D positions of the checkerboard's inner corners, centred at (0, 0, 0).
        columns, rows = self.pattern
        self.object_points = np.array([
            [(column - (columns - 1) / 2) * self.square_size,
             (row - (rows - 1) / 2) * self.square_size, 0.0]
            for row in range(rows) for column in range(columns)
        ], dtype=np.float32)

        # Convert OpenCV optical axes (right, down, forward) to AGV axes
        # (forward, left, up), including the camera's upward mounting pitch.
        cosine, sine = math.cos(self.camera_pitch), math.sin(self.camera_pitch)
        pitch_rotation = np.array([[cosine, 0, sine], [0, 1, 0], [-sine, 0, cosine]])
        optical_to_body = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        self.optical_to_base = pitch_rotation @ optical_to_body

        self.bridge = CvBridge()
        self.last_detection_time = 0.0
        self.image_subscriber = self.create_subscription(
            Image, self.image_topic, self.process_image, qos_profile_sensor_data
        )
        self.polar_publisher = self.create_publisher(
            PointStamped, '/target/relative_polar', 10
        )
        self.pose_publisher = self.create_publisher(
            PoseStamped, '/target/relative_pose', 10
        )
        self.get_logger().info(f'Listening for camera images on {self.image_topic}')

    def process_image(self, message):
        """Find the checkerboard, estimate its pose, and publish the result."""
        now = time.monotonic()
        if now - self.last_detection_time < 1.0 / max(self.detection_rate, 0.1):
            return
        self.last_detection_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'Could not convert camera image: {error}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # The "SB" detector also works without a white border around the board
        # (our board sits on a dark backing, which the classic detector cannot
        # handle). It already returns sub-pixel accurate corners.
        found, corners = cv2.findChessboardCornersSB(gray, self.pattern)
        if not found:
            self.display(frame)
            return
        height, width = gray.shape
        camera_matrix = np.array([
            [width / (2 * math.tan(self.horizontal_fov / 2)), 0, width / 2],
            [0, height / (2 * math.tan(self.vertical_fov / 2)), height / 2],
            [0, 0, 1],
        ], dtype=np.float64)
        distortion = np.zeros(5)

        success, rotation, translation = cv2.solvePnP(
            self.object_points, corners, camera_matrix, distortion
        )
        if not success or translation[2, 0] <= 0:
            return

        projected, _ = cv2.projectPoints(
            self.object_points, rotation, translation, camera_matrix, distortion
        )
        rmse = float(np.sqrt(np.mean((corners - projected) ** 2)))
        if rmse > self.max_rmse:
            return

        position = self.camera_offset + self.optical_to_base @ translation.flatten()
        distance = math.hypot(position[0], position[1])
        bearing = math.atan2(position[1], position[0])

        polar = PointStamped()
        polar.header.stamp = message.header.stamp
        polar.header.frame_id = self.base_frame
        polar.point.x = distance
        polar.point.y = bearing
        polar.point.z = float(np.linalg.norm(position))
        self.polar_publisher.publish(polar)

        normal = self.optical_to_base @ cv2.Rodrigues(rotation)[0][:, 2]
        if np.dot(normal[:2], position[:2]) < 0:
            normal = -normal
        yaw = math.atan2(normal[1], normal[0])
        pose = PoseStamped()
        pose.header = polar.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, position)
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        self.pose_publisher.publish(pose)

        cv2.drawChessboardCorners(frame, self.pattern, corners, True)
        cv2.putText(frame, f'{distance:.2f} m, {math.degrees(bearing):+.1f} deg',
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        self.display(frame)

    def display(self, frame):
        if self.show_image:
            cv2.imshow('Camera Module', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_image:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraModule()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
