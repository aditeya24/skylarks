#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from interfaces.msg import TargetDeviation

class QRDetector(Node):
    def __init__(self):
        super().__init__('qr_detector')
        self.publisher_ = self.create_publisher(TargetDeviation, '/vision/target_deviation', 10)
        self.timer = self.create_timer(0.5, self.publish_mock_data)
        self.get_logger().info('QR Detector Node Started')

    def publish_mock_data(self):
        msg = TargetDeviation()
        msg.detected = True
        msg.x_error = 0.1
        msg.y_error = -0.05
        msg.qr_data = "PAYLOAD_TARGET_A"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QRDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
