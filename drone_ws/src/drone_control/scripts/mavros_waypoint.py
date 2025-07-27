#!/usr/bin/env python3

import rospy
import time
from mavros_msgs.msg import State, WaypointReached
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong
# from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64,String

waypoint_reached_status = False
current_altitude = 0.0  
qr_detected = False
qr_data = ""


def altitude_callback(msg):
   
    global current_altitude
    current_altitude = msg.data  

def qr_callback(qr_msg):
    global qr_detected
    global qr_data
    qr_detected = True
    qr_data = qr_msg.data
    rospy.loginfo(f"QR Code detected: {qr_data}")
    
def waypoint_reached(msg):
    
    rospy.loginfo(f"Waypoint reached: {msg.wp_seq}")
    global waypoint_reached_status
    
    if(msg.wp_seq == 3):
        waypoint_reached_status = True
        rospy.loginfo("Descending to 1 meters!!!!!!")
          

def position_callback(msg1):
    global current_pose
    current_pose = msg1

# def hover_at_altitude(altitude, duration):
#     """
#     Function to hover at a specific altitude for a given duration.
#     """
#     rospy.loginfo(f"Hovering at {altitude} meters for {duration} seconds.")
#     position_pub = rospy.Publisher("/mavros/setpoint_position/local",PoseStamped, queue_size=10)
#     hover_pose = PoseStamped()
#     hover_pose.header.stamp = rospy.Time.now()
#     hover_pose.pose.position.x =  0
#     hover_pose.pose.position.y = 0
#     hover_pose.pose.position.z = altitude

#     rate = rospy.Rate(10)  # 10 Hz
#     start_time = rospy.Time.now()
#     while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
#         position_pub.publish(hover_pose)
#         rate.sleep()
#     rospy.loginfo("Hovering complete.")    
def set_home_position(current_gps, latitude=0.0, longitude=0.0, altitude=0.0):
    """
    Update home position dynamically during flight.
    :param current_gps: True to use current GPS location, False for custom coordinates.
    :param latitude: Latitude of new home position (used if current_gps=False).
    :param longitude: Longitude of new home position (used if current_gps=False).
    :param altitude: Altitude of new home position (used if current_gps=False).
    """
    rospy.wait_for_service('/mavros/cmd/command')
    try:
        set_home_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        response = set_home_service(
            broadcast=False,
            command=179,  # MAV_CMD_DO_SET_HOME
            param1=1 if not current_gps else 0,  # Use custom coordinates or current GPS
            param2=0,
            param3=0,
            param4=0,
            param5=latitude,
            param6=longitude,
            param7=altitude
        )
        if response.success:
            rospy.loginfo("Home position updated successfully!")
        else:
            rospy.logerr("Failed to update home position.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
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

def set_loiter_mode():
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode",SetMode)
    mode_status = set_mode_client(base_mode = 0,custom_mode = "LOITER")

    if(mode_status.mode_sent):
        rospy.loginfo("Now in LOITER MODE")
    else:
        rospy.logwarn("Could not switch to LOITER mode")    


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
    # rospy.Subscriber("/mavros/global_position/rel_alt", Float64, altitude_callback)
    waypoint_reached_sub = rospy.Subscriber("/mavros/mission/reached", WaypointReached, waypoint_reached)
    rospy.Subscriber("/qr_detected",String,qr_callback)
    payload_pub = rospy.Publisher('/payload', String, queue_size=10)
    rospy.loginfo("Initialized node skylarks_drone") 
     
  
    try:
        set_guided_mode()
        rospy.sleep(2)
        arm()
        rospy.sleep(4)
        takeoff()
        rospy.loginfo("Outside takeoff (edit1)")
        rospy.sleep(7)
        rospy.loginfo("Hold OVER")
        set_auto_mode()
        rate = rospy.Rate(5)
        while ((not waypoint_reached_status) and (not rospy.is_shutdown())):
            rospy.logwarn("Waiting to reach the WAYPOINT")
            rate.sleep()
        rospy.sleep(5)    
        set_guided_mode()
        
        rospy.loginfo("Waiting for QR code detection")
        rate2 = rospy.Rate(5)
        start_time = time.time()
        while ((not qr_detected) and (not rospy.is_shutdown())):
            if time.time() - start_time > 30:
                rospy.logwarn("QR code detection timed out")
                break
            rospy.loginfo("Waiting for QR code detection")
            rate2.sleep()
        if qr_detected:
            rospy.loginfo("QR code detection Completed. Activating payload.")
            payload_pub.publish('ON')
            rospy.sleep(5)
            payload_pub.publish('OFF')
        else:
            rospy.logwarn("QR code detection timed out")
 
        rospy.sleep(5)
        rospy.loginfo("Entering RTL Mode")
        set_rtl_mode()    
        rospy.signal_shutdown("Shutting down the node")
    except rospy.ROSInterruptException:
        rospy.signal_shutdown("Shutting down the node")
