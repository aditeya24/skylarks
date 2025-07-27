#!/usr/bin/env python3

import rospy
import time
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped


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
    except rospy.ServiceException as error:
        rospy.logerr("Arming service FAILED") 

def disarm():
    rospy.wait_for_service('mavros/cmd/arming')
    arm_service = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
    arm_status = arm_service(False)
    if arm_status.success:
        rospy.loginfo("DISARMED") 
    else:
        rospy.logwarn("Failed to DISARM")       
        

def set_guided_mode():
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode",SetMode)
    mode_status = set_mode_client(base_mode = 0,custom_mode = "GUIDED")

    if(mode_status.mode_sent):
        rospy.loginfo("Now in GUIDED MODE")
    else:
        rospy.logwarn("Could not switch to GUIDED mode")    

def takeoff_hold():
    rospy.wait_for_service("/mavros/cmd/takeoff")
    position = rospy.Publisher("/mavros/setpoint_position/local",PoseStamped,queue_size=10)
    takeoff_service_client = rospy.ServiceProxy("/mavros/cmd/takeoff",CommandTOL)
    takeoff_success = takeoff_service_client(altitude=2,latitude=0, longitude=0, min_pitch=0, yaw=0)
    if takeoff_success.success:
        rospy.loginfo("Mission Takeoff SUCCESSFUL")
        current_pos = PoseStamped()
        current_pos.pose.position.x = 0
        current_pos.pose.position.y = 0
        current_pos.pose.position.z = 2
        rospy.loginfo("Holding")
        rate = rospy.Rate(10)
        for tm in range(40):
            position.publish(current_pos)
            rate.sleep()

    else:
        rospy.logerr("Couldnot takeoff")
    


            
    
def land():
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode",SetMode)
    mode_status = set_mode_client(base_mode = 0,custom_mode = "LAND")

    if(mode_status.mode_sent):
        rospy.loginfo("Now in LAND MODE")
    else:
        rospy.logwarn("Could not switch to LAND mode")    


    

if __name__ == '__main__':
    rospy.init_node("skylarks_drone_takeoff",anonymous = True)
    rospy.loginfo("Initialized node skylarks_drone")  
  
    try:
        set_guided_mode()
        rospy.sleep(2)
        arm()
        rospy.sleep(5)
        takeoff_hold()
        # rospy.sleep(2)
        land()  
        disarm()
        rospy.signal_shutdown("Shutting down the node")
    except rospy.ROSInterruptException:
        rospy.signal_shutdown("Shutting down the node")
