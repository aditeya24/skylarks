#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyzbar.pyzbar as pyzbar
from std_msgs.msg import String

def qr_callback(msg):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    qr_codes = pyzbar.decode(frame)

    for qr in qr_codes:
        qr_data = qr.data.decode("utf-8")
        rospy.loginfo(f"QR Code: {qr_data}")
        qr_pub.publish(qr_data)

def qr_detector():
    rospy.init_node('qr_detector', anonymous=True)
    global qr_pub
    qr_pub = rospy.Publisher('/qr_detected', String, queue_size=10)
    rospy.Subscriber('/camera/image_raw', Image, qr_callback)
    rospy.spin()

if __name__ == '__main__':
    qr_detector()

