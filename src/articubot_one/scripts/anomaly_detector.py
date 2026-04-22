#!/usr/bin/env python3
"""
Disaster Rescue Rover — Hazard & Survivor Detector
Combines color-based detection (for Gazebo primitives) + YOLOv8 (for realism)
+ LiDAR distance + TF map coordinates to classify and report rescue alerts.

Usage: python3 anomaly_detector.py [--conf 0.4] [--show]
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from ultralytics import YOLO
import cv2
import numpy as np
import json
import math
import os
import time
import argparse
from datetime import datetime


# ─── Severity classification by YOLO class label ───────────────────────────
SEVERITY_MAP = {
    'person':      ('SURVIVOR',  'CRITICAL', 'STOP — Survivor detected, mark location'),
    'bed':         ('SURVIVOR',  'CRITICAL', 'STOP — Survivor lying down, mark location'),
    'fire':        ('FIRE',      'HIGH',     'Avoid area — Active fire hazard'),
    'smoke':       ('SMOKE',     'HIGH',     'Caution — Smoke/gas detected'),
    'car':         ('WRECKAGE',  'MEDIUM',   'Navigate around — Vehicle wreckage'),
    'truck':       ('WRECKAGE',  'MEDIUM',   'Navigate around — Vehicle wreckage'),
    'bottle':      ('HAZMAT',    'HIGH',     'Caution — Potential hazmat spill'),
    'suitcase':    ('DEBRIS',    'MEDIUM',   'Navigate around — Debris blockage'),
    'backpack':    ('DEBRIS',    'MEDIUM',   'Navigate around — Debris blockage'),
    'chair':       ('DEBRIS',    'MEDIUM',   'Navigate around — Structural debris'),
    'dining table':('DEBRIS',    'MEDIUM',   'Navigate around — Structural debris'),
    'default':     ('UNKNOWN',   'MEDIUM',   'Approach cautiously — Unknown hazard'),
}

# ─── Color-based detection rules for Gazebo primitives ──────────────────────
# Each entry: (label, HSV_lower, HSV_upper, obs_type, severity, action, min_area)
COLOR_RULES = [
    # Blue survivor torso (ambient 0.2 0.4 0.8) → hue ~210° → HSV ~105-125
    ('survivor',    (100, 80,  50),  (130, 255, 255), 'SURVIVOR', 'CRITICAL', 'STOP — Survivor detected, mark location',      800),
    # Red-orange fire zone (ambient 0.9 0.2 0.0) → hue ~0-10° → HSV ~0-10
    ('fire',        (0,   150, 100), (10,  255, 255), 'FIRE',     'HIGH',     'Avoid area — Active fire hazard',              400),
    # Yellow hazmat barrel (ambient 0.95 0.85 0.0) → hue ~45° → HSV ~20-35
    ('hazmat',      (18,  150, 150), (35,  255, 255), 'HAZMAT',   'HIGH',     'Caution — Hazmat container detected',          300),
    # Grey concrete debris (low saturation, medium value) → HSV broad grey
    ('debris',      (0,   0,   60),  (180, 50,  180), 'DEBRIS',   'MEDIUM',   'Navigate around — Concrete debris',            500),
]

SEVERITY_ORDER = {'CRITICAL': 4, 'HIGH': 3, 'MEDIUM': 2, 'LOW': 1}
REPORT_DIR = os.path.expanduser('~/Documents/MAR_miniproject/reports')
COOLDOWN_SEC = {
    'CRITICAL': 5.0,   # survivors — alert frequently
    'HIGH':     30.0,  # fire/hazmat — alert every 30s
    'MEDIUM':   60.0,  # debris — alert every 60s
}


class AnomalyDetector(Node):
    def __init__(self, conf_threshold, show_window):
        super().__init__('anomaly_detector')
        self.bridge = CvBridge()
        self.conf = conf_threshold
        self.show_window = show_window
        self.latest_scan = None
        self.last_alert_time = {}
        self.session_reports = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Loading YOLOv8 model...')
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('Model ready.')

        self.create_subscription(Image,     '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(LaserScan, '/scan',             self.scan_callback,  10)

        self.pub_image = self.create_publisher(Image,  '/rescue_detection/image',  10)
        self.pub_alert = self.create_publisher(String, '/rescue_detection/alerts', 10)

        os.makedirs(REPORT_DIR, exist_ok=True)
        self.get_logger().info(f'Rescue detector active (YOLO + color). Reports → {REPORT_DIR}')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def get_robot_position(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return round(t.transform.translation.x, 2), round(t.transform.translation.y, 2)
        except Exception:
            return None, None

    def get_distance(self, cx, img_width):
        if self.latest_scan is None:
            return None
        scan = self.latest_scan
        angle_fraction = (cx / img_width) - 0.5
        ray_angle = scan.angle_min + (0.5 + angle_fraction) * (scan.angle_max - scan.angle_min)
        idx = int((ray_angle - scan.angle_min) / scan.angle_increment)
        idx = max(0, min(idx, len(scan.ranges) - 1))
        dist = scan.ranges[idx]
        return None if (math.isnan(dist) or math.isinf(dist)) else round(dist, 2)

    # YOLO classes that are Gazebo false positives — ignore them
    YOLO_IGNORE = {'stop sign', 'tv', 'laptop', 'cell phone', 'remote', 'clock',
                   'vase', 'book', 'scissors', 'toothbrush', 'hair drier'}

    def classify_yolo(self, label):
        if label.lower() in self.YOLO_IGNORE:
            return None
        label_lower = label.lower()
        for key, val in SEVERITY_MAP.items():
            if key in label_lower:
                return val
        return SEVERITY_MAP['default']

    def color_detections(self, frame):
        """Return list of (label, obs_type, severity, action, bbox, confidence) from color rules."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = []
        for label, lower, upper, obs_type, severity, action, min_area in COLOR_RULES:
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((5, 5), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((10, 10), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < min_area:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                conf = min(0.95, area / (frame.shape[0] * frame.shape[1]))
                detections.append((label, obs_type, severity, action, [x, y, x+w, y+h], round(conf, 2)))
        return detections

    def fire_alert(self, label, obs_type, severity, action, bbox, confidence, frame, annotated, img_width):
        alert_key = f'{label}_{severity}'
        now = time.time()
        cooldown = COOLDOWN_SEC.get(severity, 30.0)
        if now - self.last_alert_time.get(alert_key, 0) < cooldown:
            return
        self.last_alert_time[alert_key] = now

        cx = (bbox[0] + bbox[2]) / 2
        distance = self.get_distance(cx, img_width)
        robot_x, robot_y = self.get_robot_position()
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        report = {
            'timestamp':      timestamp,
            'type':           obs_type,
            'label':          label,
            'severity':       severity,
            'confidence':     confidence,
            'distance_m':     distance,
            'robot_position': {'x': robot_x, 'y': robot_y},
            'action':         action,
        }
        self.session_reports.append(report)

        sep = '=' * 55
        self.get_logger().info(f'\n{sep}')
        self.get_logger().info(f'  RESCUE ALERT DETECTED')
        self.get_logger().info(f'  Type      : {obs_type}')
        self.get_logger().info(f'  Severity  : {severity}')
        self.get_logger().info(f'  Label     : {label} ({confidence*100:.0f}%)')
        self.get_logger().info(f'  Distance  : {distance}m')
        self.get_logger().info(f'  Position  : x={robot_x}, y={robot_y}')
        self.get_logger().info(f'  Action    : {action}')
        self.get_logger().info(f'{sep}\n')

        alert_msg = String()
        alert_msg.data = json.dumps(report)
        self.pub_alert.publish(alert_msg)

        if severity in ('CRITICAL', 'HIGH'):
            fname = os.path.join(REPORT_DIR,
                f'{severity}_{obs_type}_{datetime.now().strftime("%H%M%S")}.json')
            with open(fname, 'w') as f:
                json.dump(report, f, indent=2)
            self.get_logger().info(f'  Report saved: {fname}')

        color = {'CRITICAL': (0, 0, 255), 'HIGH': (0, 100, 255),
                 'MEDIUM': (0, 165, 255), 'LOW': (0, 255, 0)}.get(severity, (255, 255, 255))
        x1, y1, x2, y2 = bbox
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
        cv2.putText(annotated, f'{severity}:{label}', (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return

        img_width = frame.shape[1]

        # ── YOLO detections ──────────────────────────────────────────────────
        results = self.model(frame, conf=self.conf, verbose=False)
        annotated = results[0].plot()

        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            label = self.model.names[cls_id]
            confidence = float(box.conf[0])
            bbox = [int(v) for v in box.xyxy[0]]
            result = self.classify_yolo(label)
            if result is None:
                continue
            obs_type, severity, action = result
            self.fire_alert(label, obs_type, severity, action, bbox, confidence,
                            frame, annotated, img_width)

        # ── Color-based detections ───────────────────────────────────────────
        for label, obs_type, severity, action, bbox, confidence in self.color_detections(frame):
            self.fire_alert(label, obs_type, severity, action, bbox, confidence,
                            frame, annotated, img_width)

        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header = msg.header
            self.pub_image.publish(out_msg)
        except Exception:
            pass

        if self.show_window:
            cv2.imshow('Rescue Detection', annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.save_session_report()
                rclpy.shutdown()

    def save_session_report(self):
        if not self.session_reports:
            self.get_logger().info('No hazards detected this session.')
            return
        fname = os.path.join(REPORT_DIR,
            f'rescue_session_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json')
        survivors = sum(1 for r in self.session_reports if r['type'] == 'SURVIVOR')
        summary = {
            'session_end':       datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'total_detections':  len(self.session_reports),
            'survivors_found':   survivors,
            'critical':  sum(1 for r in self.session_reports if r['severity'] == 'CRITICAL'),
            'high':      sum(1 for r in self.session_reports if r['severity'] == 'HIGH'),
            'medium':    sum(1 for r in self.session_reports if r['severity'] == 'MEDIUM'),
            'detections': self.session_reports,
        }
        with open(fname, 'w') as f:
            json.dump(summary, f, indent=2)
        self.get_logger().info(f'\nRescue session report saved: {fname}')
        self.get_logger().info(f'Total: {len(self.session_reports)}  '
                               f'Survivors: {survivors}  '
                               f'CRITICAL:{summary["critical"]}  '
                               f'HIGH:{summary["high"]}  '
                               f'MEDIUM:{summary["medium"]}')


def main():
    parser = argparse.ArgumentParser(description='Disaster Rescue Rover — Hazard & Survivor Detector')
    parser.add_argument('--conf', type=float, default=0.4, help='YOLO confidence threshold')
    parser.add_argument('--show', action='store_true', help='Show detection window')
    args = parser.parse_args()

    rclpy.init()
    node = AnomalyDetector(args.conf, args.show)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down — saving rescue session report...')
        node.save_session_report()
    finally:
        if args.show:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
