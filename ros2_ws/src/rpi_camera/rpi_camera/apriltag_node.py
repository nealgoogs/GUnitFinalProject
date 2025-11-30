#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped

from cv_bridge import CvBridge
import cv2
import numpy as np
from pupil_apriltags import Detector


class AprilTagNode(Node):
    def __init__(self):
        super().__init__('apriltag_node')

        # Parameters
        self.declare_parameter('tag_size', 0.05)  # meters
        self.tag_size = self.get_parameter('tag_size').value

        # Focal length guesses (tune later if needed)
        self.declare_parameter('fx', 600.0)
        self.declare_parameter('fy', 600.0)
        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)

        self.bridge = CvBridge()
        self.image_width = None
        self.image_height = None

        # AprilTag detector
        self.detector = Detector(
            families='tag36h11',
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25,
            debug=False,
        )

        # Sub & pubs
        self.sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.distance_pub = self.create_publisher(Float32, 'apriltag/distance', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'apriltag/pose', 10)
        self.info_pub = self.create_publisher(String, 'apriltag/info', 10)

        self.get_logger().info(
            f"AprilTagNode started. tag_size={self.tag_size} m, fx={self.fx}, fy={self.fy}"
        )

    def image_callback(self, msg: Image):
        # Convert ROS image → OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        if frame is None:
            self.get_logger().warning("Empty frame received!")
            return

        # Initialize image shape
        if self.image_width is None:
            self.image_height, self.image_width = frame.shape[:2]
            self.get_logger().info(f"Image size: {self.image_width} x {self.image_height}")

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Camera intrinsics
        cx = self.image_width / 2.0
        cy = self.image_height / 2.0
        camera_params = (self.fx, self.fy, cx, cy)

        # Detect + pose
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=self.tag_size,
        )

        if len(detections) == 0:
            info = String()
            info.data = "no tags detected"
            self.info_pub.publish(info)
            return

        # Find the closest tag
        closest = None
        closest_dist = None

        for det in detections:
            if det.pose_t is None:
                continue

            # det.pose_t is 3x1 numpy array
            tx, ty, tz = float(det.pose_t[0]), float(det.pose_t[1]), float(det.pose_t[2])
            dist = math.sqrt(tx * tx + ty * ty + tz * tz)

            if closest is None or dist < closest_dist:
                closest = det
                closest_dist = dist

        if closest is None:
            info = String()
            info.data = "detections found but no valid pose"
            self.info_pub.publish(info)
            return

        # Extract translation as floats
        tx, ty, tz = float(closest.pose_t[0]), float(closest.pose_t[1]), float(closest.pose_t[2])

        # Publish distance
        dist_msg = Float32()
        dist_msg.data = float(closest_dist)
        self.distance_pub.publish(dist_msg)

        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header = msg.header  # keep timestamp/frame_id

        pose_msg.pose.position.x = tx
        pose_msg.pose.position.y = ty
        pose_msg.pose.position.z = tz

        # Convert rotation matrix → quaternion
        qx, qy, qz, qw = rotation_matrix_to_quaternion(closest.pose_R)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_pub.publish(pose_msg)

        # Publish readable summary
        info = String()
        info.data = (
            f"tag_id={closest.tag_id}, "
            f"distance={closest_dist:.3f} m, "
            f"position=({tx:.3f}, {ty:.3f}, {tz:.3f})"
        )
        self.info_pub.publish(info)
        self.get_logger().info(info.data)


def rotation_matrix_to_quaternion(R: np.ndarray):
    """Convert 3×3 rotation matrix into quaternion (x,y,z,w)."""
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]

    tr = m00 + m11 + m22

    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif m00 > m11 and m00 > m22:
        S = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S

    return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
