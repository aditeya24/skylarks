#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import os
import numpy as np
from pyzbar.pyzbar import decode
from interfaces.msg import TargetDeviation

class QRDetector(Node):
    def __init__(self):
        super().__init__('qr_detector')
        self.publisher_ = self.create_publisher(TargetDeviation, '/vision/target_deviation', 1)
        
        self.cap = None
        self.device_path = '/dev/video11'

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
        if not os.path.exists(self.device_path):
            self.get_logger().warn(f"Waiting for {self.device_path}...", throttle_duration_sec=1.0)
            if self.cap is not None:
                self.cap.release()
                self.cap = None
            return

        if self.cap is None:
            self.cap = cv2.VideoCapture(self.device_path, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn(f"Video Feed lost. Waiting...", throttle_duration_sec=1.0)
            self.cap.release()
            self.cap = None
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        qr_codes = decode(gray)

        msg = TargetDeviation()
        
        if qr_codes:
            qr = qr_codes[0]
            msg.detected = True
            msg.qr_data = qr.data.decode("utf-8")
            
            self.get_logger().info(f'QR Code: {msg.qr_data}', throttle_duration_sec=0.25)
             

        else:
            msg.detected = False
            msg.x_error = 0.0
            msg.y_error = 0.0
            msg.qr_data = ""
            self.get_logger().info(f"QR not detected. Retrying...", throttle_duration_sec=0.5)

        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.cap is not None and self.cap.isOpened():
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