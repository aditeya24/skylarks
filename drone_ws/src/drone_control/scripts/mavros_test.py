#!/usr/bin/env python3

import rospy
import time
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool

# def state_callback(msg):
#     rospy.loginfo("Connection Status: %r", msg.connected)
#     rospy.loginfo("Flight Mode: %s", msg.mode)
#     rospy.loginfo("Armed: %r", msg.armed)

def arm():
    rospy.wait_for_service('mavros/cmd/arming')
    try:
        arm_service = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
        arm_status = arm_service(True)
        if arm_status.success:
            rospy.loginfo("Skylarks Drone Successfully ARMED")
        else:
            rospy.logwarn("Failed to ARM  the Drone")

        rospy.sleep(5)
        arm_status = arm_service(False)
        if arm_status.success:
            rospy.loginfo("DISARMED") 
        else:
            rospy.logwarn("Failed to DISARM")       
    except rospy.ServiceException as error:
        rospy.logerr("Arming service FAILED") 




if __name__ == '__main__':
    rospy.init_node("skylarks_drone",anonymous = True)
    rospy.loginfo("Initialized node skylarks_drone")
    try:
        arm()
        rospy.signal_shutdown("Shutting down the node")
    except rospy.ROSInterruptException:
        rospy.signal_shutdown("Shutting down the node")
