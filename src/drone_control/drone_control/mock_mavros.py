#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State

class MockMavros(Node):
    def __init__(self):
        super().__init__('mavros_node') # Emulate the real node name
        
        self.state_pub = self.create_publisher(State, '/mavros/state', 10)
        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/local_position/pose', 10)
        
        # Subscribe to commands so arrows appear in graph
        self.create_subscription(PoseStamped, '/mavros/setpoint_position/local', self.dummy_cb, 10)
        self.create_subscription(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', self.dummy_cb, 10)

    def dummy_cb(self, msg): pass

def main(args=None):
    rclpy.init(args=args)
    node = MockMavros()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
