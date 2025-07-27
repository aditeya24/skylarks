# ...existing code...

# Add these imports if not already present
import time
import rospy
from mavros_msgs.srv import SetMode

# ...existing code...

def reach_waypoint(self, waypoint, tolerance=0.2):
    # ...existing code...
    
    # After reaching the waypoint, add hover behavior
    if self.distance_to_target(target_pose) < tolerance:
        rospy.loginfo("Waypoint reached, starting hover at 2 meters...")
        
        # Create a hover pose at 2 meters height (keeping x,y position)
        hover_pose = PoseStamped()
        hover_pose.header.stamp = rospy.Time.now()
        hover_pose.header.frame_id = "map"
        hover_pose.pose.position.x = self.current_pose.pose.position.x
        hover_pose.pose.position.y = self.current_pose.pose.position.y
        hover_pose.pose.position.z = 2.0  # Hover at exactly 2 meters
        
        # Keep orientation the same
        hover_pose.pose.orientation = self.current_pose.pose.orientation
        
        # Hover for 10 seconds
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            self.local_pos_pub.publish(hover_pose)
            self.rate.sleep()
            
            # Check if 10 seconds have passed
            if (rospy.Time.now() - start_time).to_sec() >= 10.0:
                rospy.loginfo("Hover complete after 10 seconds, switching to RTL")
                # Switch to RTL mode
                set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
                set_mode_client(custom_mode="AUTO.RTL")
                return True
        
        return True
    
    return False

# ...existing code...
