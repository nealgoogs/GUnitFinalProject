#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO
import numpy as np


class YoloV8Node(Node):
    def __init__(self):
        super().__init__('yolov8_node')

        # Parameters
        self.declare_parameter('model_path', '/home/nealgoogs/GUnitFinalProject/epoch150_nano_sgd/weights/best.pt')
        self.declare_parameter('conf_thres', 0.5)
        self.declare_parameter('publish_annotated', True)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_thres = self.get_parameter('conf_thres').get_parameter_value().double_value
        self.publish_annotated = self.get_parameter('publish_annotated').get_parameter_value().bool_value

        # Load YOLO model (once)
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            raise

        # CV bridge
        self.bridge = CvBridge()

        # Subscriber: camera frames
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher: detection summary (simple text)
        self.detections_pub = self.create_publisher(
            String,
            'yolo/detections',
            10
        )

        # Publisher: annotated image
        if self.publish_annotated:
            self.annotated_image_pub = self.create_publisher(
                Image,
                'camera/image_yolo',
                10
            )
        else:
            self.annotated_image_pub = None

        self.get_logger().info(
            f'YoloV8Node started. Listening on /camera/image_raw, '
            f'conf_thres={self.conf_thres}, publish_annotated={self.publish_annotated}'
        )

    def image_callback(self, msg: Image):
        # Convert ROS Image -> OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Run YOLO inference
        try:
            # results is a list; take first
            results = self.model(frame, verbose=False)[0]
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed: {e}')
            return

        boxes = results.boxes
        if boxes is None or len(boxes) == 0:
            # No detections – still publish empty summary if you want
            summary = String()
            summary.data = 'no detections'
            self.detections_pub.publish(summary)
            return

        names = self.model.names  # dict: class_id -> class_name
        det_strings = []
        annotated = frame.copy()

        for box in boxes:
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])

            if conf < self.conf_thres:
                continue

            class_name = names.get(cls_id, str(cls_id))

            # xyxy bounding box
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            det_strings.append(
                f'{class_name} ({conf:.2f}) '
                f'[{x1},{y1},{x2},{y2}]'
            )

            # Draw box + label on annotated image
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f'{class_name} {conf:.2f}'
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(annotated, (x1, y1 - th - 4), (x1 + tw, y1), (0, 255, 0), -1)
            cv2.putText(
                annotated,
                label,
                (x1, y1 - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                1,
                cv2.LINE_AA
            )

        # Publish text summary
        summary = String()
        summary.data = '; '.join(det_strings) if det_strings else 'no detections (below conf threshold)'
        self.detections_pub.publish(summary)

        # Publish annotated image if enabled
        if self.annotated_image_pub is not None and det_strings:
            try:
                out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                out_msg.header = msg.header  # preserve original timestamp/frame_id
                self.annotated_image_pub.publish(out_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish annotated image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloV8Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
