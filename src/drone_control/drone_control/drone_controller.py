#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from interfaces.msg import TargetDeviation
from interfaces.action import DropPayload

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Subscribers
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, 10)
        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pos_cb, 10)
        self.vision_sub = self.create_subscription(
            TargetDeviation, '/vision/target_deviation', self.vision_cb, 1)

        # Publishers
        self.local_pos_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self.vel_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # Action Client
        self._payload_client = ActionClient(self, DropPayload, '/payload/drop')

        self.get_logger().info('Drone Controller Node Started')

    def state_cb(self, msg): pass
    def pos_cb(self, msg): pass
    def vision_cb(self, msg): pass

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
