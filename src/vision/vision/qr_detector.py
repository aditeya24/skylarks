#!/usr/bin/env python3
"""
Skylarks QR Detector Node

This node interfaces with a v4l2loopback virtual camera stream to capture video frames,
convert them to grayscale, and use `pyzbar` to decode any detected QR codes. 
If a QR code is detected, it calculates the 'x' and 'y' pixel offset from the true 
center of the 640x400 camera feed, resulting in a normalized Cartesian (-1.0 to 1.0) 
error vector published to `/vision/target_deviation`.
"""

import rclpy
from rclpy.node import Node
import cv2
import os
import numpy as np
from pyzbar.pyzbar import decode
from interfaces.msg import TargetDeviation

class QRDetector(Node):
    """
    ROS 2 Node responsible for visual payload localization.
    Publishes to:
        - /vision/target_deviation (interfaces.msg.TargetDeviation)
    """
    def __init__(self):
        super().__init__('qr_detector')
        self.publisher_ = self.create_publisher(TargetDeviation, '/vision/target_deviation', 1)
        
        self.cap = None
        self.device_path = '/dev/video41'

        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info('QR Detector Node Started')

    def timer_callback(self):
        """
        Main processing loop executed every 33ms (~30 FPS).
        Reads frame, decodes QR, calculates offset, and publishes the normalized error matrix.
        """
        # Ensure virtual camera device exists before capturing
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
            # We assume the first detected QR is our designated drop zone target
            qr = qr_codes[0]
            msg.detected = True
            msg.qr_data = qr.data.decode("utf-8")
            
            # Optical Center Tracking:
            # The camera outputs a 1280x800 resolution stream natively to /dev/video41.
            # Thus, the exact optical center of the image is (640, 400).
            rect = qr.rect
            qr_cx = rect.left + rect.width / 2.0
            qr_cy = rect.top + rect.height / 2.0
            
            # Normalizing the error (-1.0 to 1.0) prevents the drone controller's PID loop 
            # from being strictly coupled to the 1280x800 resolution.
            msg.x_error = (qr_cx - 640.0) / 640.0
            msg.y_error = (qr_cy - 400.0) / 400.0
            
            self.get_logger().info(f'QR Code: {msg.qr_data} | Error: X={msg.x_error:.2f}, Y={msg.y_error:.2f}', throttle_duration_sec=0.25)
             
        else:
            # If no QR is present, zero out the offset errors to prevent errant movement commands
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