#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from interfaces.action import DropPayload

class PayloadDriver(Node):
    def __init__(self):
        super().__init__('payload_driver')
        self._action_server = ActionServer(
            self,
            DropPayload,
            '/payload/drop',
            self.execute_callback)
        self.get_logger().info('Payload Driver Node Started')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing payload drop...')
        goal_handle.succeed()
        result = DropPayload.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = PayloadDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
