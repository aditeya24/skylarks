#!/usr/bin/env python3
"""
Skylarks Payload Driver Node

Hardware interface node that controls an MG90S servo via RPi.GPIO software PWM.
This node exposes a ROS 2 Action Server (`/payload/drop`) that the mission controller
calls to physically open the bay door, wait for gravity drop, and then close it.
Multithreaded execution prevents the `time.sleep` drops from blocking system cancellations.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import RPi.GPIO as GPIO
from interfaces.action import DropPayload

SERVO_PIN = 18       # BCM Header pin assigned to the Servo signal wire
PWM_FREQ = 50        # Standard analog servo frequency (50 Hz = 20ms period)

# Duty Cycles
# 2.5  = ~0 Degrees
# 7.5  = ~90 Degrees
# 12.5 = ~180 Degrees
CLOSED_DUTY = 6
OPEN_DUTY = 11   

class PayloadDriver(Node):
    """
    Action Server Node for managing the Physical Payload Bay.
    Initializes hardware on startup to the Closed/Safe state.
    """
    def __init__(self):
        super().__init__('payload_driver')
        
        try:
            # Configure GPIO for BCM numbering and set pin to output
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(SERVO_PIN, GPIO.OUT)
            
            self.pwm = GPIO.PWM(SERVO_PIN, PWM_FREQ)
            self.pwm.start(CLOSED_DUTY)
            
            time.sleep(0.5)
            self.pwm.ChangeDutyCycle(0) 
            
            self.hardware_ready = True
        except Exception as e:
            self.get_logger().error(f"GPIO Setup Failed: {e}")
            self.hardware_ready = False

        self.cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            DropPayload,
            '/payload/drop',
            self.execute_callback,
            callback_group=self.cb_group)
            
        self.get_logger().info('Payload Driver Ready (Snap Mode)')

    def execute_callback(self, goal_handle):
        """
        Action callback that triggers exactly when '/payload/drop' is called.
        Note: The 5-second sleep does NOT freeze the ROS Node because `callback_group` utilizes 
        the `ReentrantCallbackGroup` combined with a `MultiThreadedExecutor` in `main()`.
        """
        self.get_logger().info('Received Drop Command.')
        
        if not self.hardware_ready:
            goal_handle.abort()
            return DropPayload.Result(success=False)

        # 1. SNAP OPEN: Energize servo to drop position
        self.get_logger().info('Opening Mechanism...')
        self.pwm.ChangeDutyCycle(OPEN_DUTY)
        time.sleep(0.5) # Wait for servo to physically travel
        self.pwm.ChangeDutyCycle(0) # Cut PWM signal to stop motor jitter and save power
        
        # 2. WAIT (Drop Time): Give gravity time to pull payload entirely out of the bay
        # Provides feedback to the mission controller
        feedback_msg = DropPayload.Feedback()
        feedback_msg.status = "Mechanism Open - Dropping"
        goal_handle.publish_feedback(feedback_msg)
        
        time.sleep(5.0) 
        
        # 3. SNAP CLOSE: Re-energize servo to default locked position
        self.get_logger().info('Closing Mechanism...')
        self.pwm.ChangeDutyCycle(CLOSED_DUTY)
        time.sleep(0.5) # Wait for travel
        self.pwm.ChangeDutyCycle(0) # Cut PWM signal

        
        # 4. FINISH
        goal_handle.succeed()
        result = DropPayload.Result()
        result.success = True
        self.get_logger().info('Drop Sequence Complete.')
        return result

    def destroy_node(self):
        if self.hardware_ready:
            self.pwm.stop()
            GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PayloadDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()