#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import String

CONTROL_PIN = 17  # GPIO pin connected to ESP32

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(CONTROL_PIN, GPIO.OUT)

def led_callback(msg):
    """Callback function to handle incoming messages."""
    if msg.data == "ON":
        GPIO.output(CONTROL_PIN, GPIO.HIGH)  # Send HIGH signal to ESP32
        rospy.loginfo("Signal sent: LED ON")
    elif msg.data == "OFF":
        GPIO.output(CONTROL_PIN, GPIO.LOW)   # Send LOW signal to ESP32
        rospy.loginfo("Signal sent: LED OFF")

def motor_control_node():
    """Initialize and run the ROS node."""
    rospy.init_node('motor_control', anonymous=True)
    rospy.Subscriber('/payload', String, led_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        motor_control_node()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()