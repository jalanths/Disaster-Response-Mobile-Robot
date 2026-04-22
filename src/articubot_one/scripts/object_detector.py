#!/usr/bin/env python3
"""
Disaster Rescue Rover — Object Detector
Uses YOLOv8 on the rover's camera feed to detect survivors, hazards, and debris.
Publishes detections and displays annotated video window.

Usage: python3 object_detector.py [--model yolov8n.pt] [--conf 0.5] [--show]
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import json
import argparse
import os
import sys


class ObjectDetector(Node):
    def __init__(self, model_path, conf_threshold, show_window):
        super().__init__('object_detector')
        self.bridge = CvBridge()
        self.conf = conf_threshold
        self.show_window = show_window

        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('Model loaded.')

        # Subscribe to camera
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publish annotated image
        self.pub_image = self.create_publisher(Image, '/object_detection/image', 10)

        # Publish detections as JSON string
        self.pub_detections = self.create_publisher(String, '/object_detection/detections', 10)

        self.frame_count = 0
        self.get_logger().info('Object detector ready. Waiting for camera feed...')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        results = self.model(frame, conf=self.conf, verbose=False)
        annotated = results[0].plot()

        detections = []
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            label = self.model.names[cls_id]
            conf = float(box.conf[0])
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
            detections.append({
                'label': label,
                'confidence': round(conf, 2),
                'bbox': [x1, y1, x2, y2]
            })

        if detections:
            self.frame_count += 1
            if self.frame_count % 10 == 1:
                for d in detections:
                    self.get_logger().info(
                        f"Detected: {d['label']} ({d['confidence']*100:.0f}%)"
                    )

        # Publish detections JSON
        det_msg = String()
        det_msg.data = json.dumps(detections)
        self.pub_detections.publish(det_msg)

        # Publish annotated image
        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header = msg.header
            self.pub_image.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f'Publish error: {e}')

        # Show window
        if self.show_window:
            cv2.imshow('Object Detection', annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='YOLOv8 Object Detector')
    parser.add_argument('--model', default='yolov8n.pt', help='YOLO model file')
    parser.add_argument('--conf', type=float, default=0.5, help='Confidence threshold (0-1)')
    parser.add_argument('--show', action='store_true', help='Show detection window')
    args = parser.parse_args()

    rclpy.init()
    node = ObjectDetector(args.model, args.conf, args.show)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down object detector.')
    finally:
        if args.show:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
