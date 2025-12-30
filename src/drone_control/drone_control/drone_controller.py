#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from enum import Enum, auto
from sensor_msgs.msg import NavSatFix
import pymap3d as pm
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from interfaces.msg import TargetDeviation
from interfaces.action import DropPayload


class MissionState(Enum):
    INIT = auto()
    TAKEOFF = auto()
    TRANSIT = auto()
    SEARCH = auto()
    ALIGN = auto()
    LAND = auto()
    DROP = auto()
    RTL = auto()

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Variables

        # state variables
        self.mission_state = MissionState.INIT
        self.drop_flag = False
        self.command_sent = False

        # navigation variables
        self.current_gps = None
        self.home_gps = None
        self.target_location = [0.0, 0.0] # need to obtain this from config/cmd line
        
        # timing/counter variables
        self.state_start_time = 0.0
        self.search_last_update = 0.0
        self.search_index = 0

        # subcriber variables
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.vision_data = TargetDeviation()

        # Subscribers
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.local_pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pos_cb, 10)
        self.vision_sub = self.create_subscription(TargetDeviation, '/vision/target_deviation', self.vision_cb, 1)
        self.global_pos_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_cb, 10)

        # Publishers
        self.local_pos_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self.vel_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # Action Client
        self._payload_client = ActionClient(self, DropPayload, '/payload/drop')

        # Service Clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        # ROS Parameters
        self.declare_parameter('target_lat', 360.0)
        self.declare_parameter('target_lon', 360.0)

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Drone Controller Node Started')

    # Callbacks
    def state_cb(self, msg): 
        self.current_state = msg

    def pos_cb(self, msg): 
        self.current_pose = msg

    def vision_cb(self, msg): 
        self.vision_data = msg

    def gps_cb(self, msg):
        self.current_gps = msg

    # State Change Helper
    def change_state(self, new_state):
        self.mission_state = new_state
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9
        self.command_sent = False
        self.get_logger().info(f"State Changed to: {new_state.name}")

    # Main loop
    def control_loop(self):
        # Check MAVROS connection
        if not self.current_state.connected:
            self.get_logger().info('Waiting for FCU Connection...', throttle_duration_sec=2.0)
            return


        if self.mission_state == MissionState.INIT:
            if self.current_gps is None or self.current_gps.status.status < 0:
                self.get_logger().info("Waiting for GPS Fix...", throttle_duration_sec=2.0)
                return

            if self.target_location == [0.0, 0.0]:
                lat = self.get_parameter('target_lat').value
                lon = self.get_parameter('target_lon').value

                if lat == 360.0 or lon == 360.0:
                    self.get_logger().error('NO TARGET COORDINATES PROVIDED. Waiting for parameters...', throttle_duration_sec=2.0)
                    return

                self.target_location = [lat, lon]
                self.get_logger().info(f"Target Location Obtained: {self.target_location}")

            if self.home_gps is None:
                self.home_gps = self.current_gps
                self.get_logger().info(f"Home Location Saved: {self.home_gps.latitude}, {self.home_gps.longitude}")

            if self.current_state.mode != "GUIDED":
                if not self.command_sent:
                    req = SetMode.Request()
                    req.custom_mode = "GUIDED"
                    self.mode_client.call_async(req)
                    self.command_sent = True
                    self.get_logger().info("Requesting GUIDED mode...")

            elif not self.current_state.armed:
                if not self.command_sent:
                    req = CommandBool.Request()
                    req.value = True
                    self.arming_client.call_async(req)
                    self.command_sent = True
                    self.get_logger().info("Requesting ARMing...")

                else:
                    self.command_sent = False
                
            else:
                self.change_state(MissionState.TAKEOFF)


        elif self.mission_state == MissionState.TAKEOFF:
            if not self.command_sent:
                req = CommandTOL.Request()
                req.min_pitch = 0.0
                req.yaw = 0.0
                req.latitude = 0.0
                req.longitude = 0.0
                req.altitude = 4.5
                self.takeoff_client.call_async(req)
                self.command_sent = True
                self.get_logger().info("Requesting TAKEOFF...")

            current_altitude = self.current_pose.pose.position.z

            if current_altitude >= 4.0:
                self.get_logger().info("Target Altitude Reached")
                self.change_state(MissionState.LAND) # this should switch to TRANSIT


        elif self.mission_state == MissionState.TRANSIT:
            pass

        elif self.mission_state == MissionState.SEARCH:
            pass

        elif self.mission_state == MissionState.ALIGN:
            pass


        elif self.mission_state == MissionState.LAND:
            if self.current_state.mode != "LAND":
                if not self.command_sent:
                    req = SetMode.Request()
                    req.custom_mode = "LAND"
                    self.mode_client.call_async(req)
                    self.command_sent = True
                    self.get_logger().info("Requesting LAND Mode...")

            if not self.current_state.armed:
                self.get_logger().info("Drone Disarmed. Landing Completed Successfully.")
                self.get_logger().info("TEST PASSED: Shutting down.")

                import sys
                sys.exit(0)


        elif self.mission_state == MissionState.DROP:
            pass

        elif self.mission_state == MissionState.RTL:
            pass

    """
    control_loop pseudocode:
        if init:
            save current location as home waypoint
            if guided:
                then arm
            else:
                set guided
            if guided and armed:
                set Takeoff
        if takeoff:
            if current_altitude < 4:
                take off till reach altitude
            else:
                set transit
        if transit:
            if qr detected:
                set align
            else if waypoint not reached:
                goto waypoint
            else:
                set search
        if search:
            if qr not detected:
                go in circles
            else:
                set align
        if align:
            if center of qr near center of camera:
                set land
            else:
                move drone to towards qr center
        if land:
            set mode land
            if disarmed and drop_flag is 0:
                drop_flag = 1
                set drop
            else if drop_flag = 1:
                log mission completed succesfully
        if drop:
            call payload action
            if action finished:
                set rtl
        if rtl:
            set home waypoint as destination waypoint
            set as init
    """

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
