#!/usr/bin/env python3
import rospy
import subprocess
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

# Initialize ROS node
rospy.init_node("gstreamer_publisher")
bridge = CvBridge()

# GStreamer pipeline (now using stdin for appsrc)
GST_PIPELINE = (
    "gst-launch-1.0 -v appsrc format=time is-live=true block=true "
    "caps=video/x-raw,format=BGR,width=640,height=480,framerate=30/1 "
    "! videoconvert ! video/x-raw,format=I420 "
    "! x264enc tune=zerolatency bitrate=500 speed-preset=ultrafast "
    "! rtph264pay config-interval=1 pt=96 "
    "! udpsink host=100.67.58.107 port=5000"
)

process = subprocess.Popen(GST_PIPELINE, shell=True, stdin=subprocess.PIPE, stderr=sys.stderr)

def callback(msg):
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        process.stdin.write(frame.tobytes())
        process.stdin.flush()
    except Exception as e:
        rospy.logerr(f"GStreamer error: {e}")

rospy.Subscriber("/camera/image_raw", Image, callback)
rospy.spin()

process.stdin.close()
process.wait()

