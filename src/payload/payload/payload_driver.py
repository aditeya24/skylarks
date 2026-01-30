#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import time
import RPi.GPIO as GPIO
from interfaces.action import DropPayload

SERVO_PIN = 18
PWM_FREQ = 50   

# Duty Cycles
# 2.5  = ~0 Degrees
# 7.5  = ~90 Degrees
# 12.5 = ~180 Degrees
CLOSED_DUTY = 6
OPEN_DUTY = 11   

class PayloadDriver(Node):
    def __init__(self):
        super().__init__('payload_driver')
        
        try:
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

        self._action_server = ActionServer(
            self,
            DropPayload,
            '/payload/drop',
            self.execute_callback)
            
        self.get_logger().info('Payload Driver Ready (Snap Mode)')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received Drop Command.')
        
        if not self.hardware_ready:
            goal_handle.abort()
            return DropPayload.Result(success=False)

        # 1. SNAP OPEN
        self.get_logger().info('Opening Mechanism...')
        self.pwm.ChangeDutyCycle(OPEN_DUTY)
        time.sleep(0.5) # Wait for servo to travel
        self.pwm.ChangeDutyCycle(0) # Cut signal to stop jitter
        
        # 2. WAIT (Drop Time)
        # Increased to 5 seconds to ensure payload falls clear
        feedback_msg = DropPayload.Feedback()
        feedback_msg.status = "Mechanism Open - Dropping"
        goal_handle.publish_feedback(feedback_msg)
        
        time.sleep(5.0) 
        
        # 3. SNAP CLOSE
        self.get_logger().info('Closing Mechanism...')
        self.pwm.ChangeDutyCycle(CLOSED_DUTY)
        time.sleep(0.5) # Wait for travel
        self.pwm.ChangeDutyCycle(0) # Cut signal
        
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()