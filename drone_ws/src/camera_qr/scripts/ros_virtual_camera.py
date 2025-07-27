#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import pyfakewebcam
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Initialize ROS node
rospy.init_node("ros_virtual_camera")

# Initialize virtual webcam
virtual_cam = pyfakewebcam.FakeWebcam('/dev/video3', 640, 480)

# Bridge between ROS and OpenCV
bridge = CvBridge()

def callback(msg):
    try:
        # Convert ROS image message to OpenCV
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Resize to match virtual webcam resolution
        frame = cv2.resize(frame, (640, 480))

        # Convert to RGB for pyfakewebcam
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Send frame to virtual webcam
        virtual_cam.schedule_frame(frame)

    except Exception as e:
        rospy.logerr(f"Error processing frame: {e}")

# Subscribe to ROS camera topic
rospy.Subscriber("/camera/image_raw", Image, callback)

rospy.loginfo("ROS Virtual Camera Node Started")
rospy.spin()
