#!/usr/bin/env python3

import rospy
import time
from mavros_msgs.msg import State, WaypointReached
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
# from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64

waypoint_reached_status = False
current_altitude = 0.0  


def altitude_callback(msg):
   
    global current_altitude
    current_altitude = msg.data  

def waypoint_reached(msg):
    
    rospy.loginfo(f"Waypoint reached: {msg.wp_seq}")
    global waypoint_reached_status
    
    if(msg.wp_seq == 2):
        waypoint_reached_status = True
        rospy.loginfo("Descending to 2 meters!!!!!!")
          

def position_callback(msg1):
    global current_pose
    current_pose = msg1

def hover_at_altitude(altitude, duration):
    """
    Function to hover at a specific altitude for a given duration.
    """
    rospy.loginfo(f"Hovering at {altitude} meters for {duration} seconds.")
    position_pub = rospy.Publisher("/mavros/setpoint_position/local",PoseStamped, queue_size=10)
    hover_pose = PoseStamped()
    hover_pose.header.stamp = rospy.Time.now()
    hover_pose.pose.position.x =  0
    hover_pose.pose.position.y = 0
    hover_pose.pose.position.z = altitude

    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
        position_pub.publish(hover_pose)
        rate.sleep()
    rospy.loginfo("Hovering complete.")    

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

# def set_loiter_mode():
#     rospy.wait_for_service("/mavros/set_mode")
#     set_mode_client = rospy.ServiceProxy("/mavros/set_mode",SetMode)
#     mode_status = set_mode_client(base_mode = 0,custom_mode = "LOITER")

#     if(mode_status.mode_sent):
#         rospy.loginfo("Now in LOITER MODE")
#     else:
#         rospy.logwarn("Could not switch to LOITER mode")    


def takeoff():
    rospy.wait_for_service("/mavros/cmd/takeoff")
    rospy.Subscriber("/mavros/global_position/rel_alt", Float64, altitude_callback)
    # position = rospy.Publisher("/mavros/setpoint_position/local",PoseStamped,queue_size=10)
    takeoff_service_client = rospy.ServiceProxy("/mavros/cmd/takeoff",CommandTOL)
    takeoff_success = takeoff_service_client(altitude=4,latitude=0, longitude=0, min_pitch=0, yaw=0)
    if takeoff_success.success:
        rospy.loginfo("Mission Takeoff SUCCESSFUL")
    else:
        rospy.logerr("Couldnot takeoff")

     
    


def set_auto_mode():
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode",SetMode)
    mode_status = set_mode_client(base_mode = 0,custom_mode = "AUTO")

    if(mode_status.mode_sent):
        rospy.loginfo("Now in AUTO MODE")
    else:
        rospy.logwarn("Could not switch to AUTO mode")    


def set_rtl_mode():
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode",SetMode)
    mode_status = set_mode_client(base_mode = 0,custom_mode = "RTL")

    if(mode_status.mode_sent):
        rospy.loginfo("Now in RTL MODE")
    else:
        rospy.logwarn("Could not switch to RTL mode")    


    
def land():
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode",SetMode)
    mode_status = set_mode_client(base_mode = 0,custom_mode = "LAND")

    if(mode_status.mode_sent):
        rospy.loginfo("Now in LAND MODE")
    else:
        rospy.logwarn("Could not switch to LAND mode")    


    

if __name__ == '__main__':
    rospy.init_node("skylarks_drone_takeoff",anonymous = True)
    rospy.Subscriber("/mavros/global_position/rel_alt", Float64, altitude_callback)
    waypoint_reached_sub = rospy.Subscriber("/mavros/mission/reached", WaypointReached, waypoint_reached)
    rospy.loginfo("Initialized node skylarks_drone") 
     
  
    try:
        set_guided_mode()
        rospy.sleep(2)
        arm()
        rospy.sleep(3)
        takeoff()
        rospy.loginfo("Outside takeoff (edit1)")
        rospy.sleep(10)
        rospy.loginfo("Hold OVER")
        set_auto_mode()
        rate = rospy.Rate(5)
        while ((not waypoint_reached_status) and (not rospy.is_shutdown())):
            rospy.logwarn("Waiting to reach the WAYPOINT")
            rate.sleep()
        rospy.sleep(5)    
        set_guided_mode()
        rospy.sleep(3)
        #rospy.loginfo("Entering RTL Mode")
        #set_rtl_mode()    
        land()
        rospy.signal_shutdown("Shutting down the node")
    except rospy.ROSInterruptException:
        rospy.signal_shutdown("Shutting down the node")
