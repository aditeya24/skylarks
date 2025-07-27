#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def camera_publisher():
    rospy.init_node('camera_publisher', anonymous=True)
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    rate = rospy.Rate(30)  # 30 FPS
    cap = cv2.VideoCapture(0)  # /dev/video0

    if not cap.isOpened():
        rospy.logerr("Failed to open camera")
        return

    bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            pub.publish(ros_image)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass

