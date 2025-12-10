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
        self.publisher_ = self.create_publisher(TargetDeviation, '/vision/target_deviation', 1)
        self.cap = cv2.VideoCapture(11)

        if not self.cap.isOpened():
            self.get_logger().info('ERROR: Failed to access video feed. Check GStreamer splitter.')
            return

        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info('QR Detector Node Started')

    def publish_mock_data(self):
        msg = TargetDeviation()
        msg.detected = True
        msg.x_error = 0.1
        msg.y_error = -0.05
        msg.qr_data = "PAYLOAD_TARGET_A"
        self.publisher_.publish(msg)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        qr_codes = decode(gray)

        msg = TargetDeviation()
        
        if qr_codes:
            qr = qr_codes[0]
            msg.detected = True
            msg.qr_data = qr.data.decode("utf-8")
            
            self.get_logger().info(f'QR Code: {msg.qr_data}')
             

        else:
            msg.detected = False
            msg.x_error = 0.0
            msg.y_error = 0.0
            msg.qr_data = ""

        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = QRDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()