#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Parameters (you can override via ros2 param)
        self.declare_parameter('device_id', 0)          # /dev/video0
        self.declare_parameter('frame_rate', 10.0)      # Hz
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        device_id = self.get_parameter('device_id').get_parameter_value().integer_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value

        # OpenCV video capture
        self.cap = cv2.VideoCapture(device_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open video device {device_id}')
            raise RuntimeError(f'Could not open video device {device_id}')

        # Optionally set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        # ROS2 publisher
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)

        # Bridge OpenCV <-> ROS Image
        self.bridge = CvBridge()

        # Timer to grab & publish frames
        timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'CameraPublisher started: device={device_id}, '
            f'{width}x{height} @ {self.frame_rate} Hz'
        )

    def timer_callback(self):
        """Grab a frame from the camera and publish as ROS2 Image."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Failed to read frame from camera')
            return

        # OpenCV is BGR; encode as BGR8 in ROS
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'

        self.publisher_.publish(msg)
        # self.get_logger().debug('Published image frame')  # uncomment if you want spam

    def destroy_node(self):
        # Make sure to release the camera
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
