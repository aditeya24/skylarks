#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import time
import RPi.GPIO as GPIO
from interfaces.action import DropPayload


SERVO_PIN = 18
PWM_FREQ = 50   

# Duty Cycles for SG90/MG90S Servos:
# 2.5  = ~0 Degrees
# 7.5  = ~90 Degrees
# 12.5 = ~180 Degrees
CLOSED_DUTY = 3.6  # Adjust this if it doesn't close fully
OPEN_DUTY = 10.6    # Adjust this if it doesn't open enough

STEP_SIZE = 0.1
STEP_DELAY = 0.02

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
            
        self.get_logger().info('Payload Driver Ready (GPIO Mode)')

    def slow_move(self, start_duty, end_duty):
        self.get_logger().info(f"Moving servo from {start_duty} to {end_duty}")

        if start_duty < end_duty:
            current = start_duty
            while current < end_duty:
                current += STEP_SIZE
                if current > end_duty: current = end_duty
                self.pwm.ChangeDutyCycle(current)
                time.sleep(STEP_DELAY)
        else:
            current = start_duty
            while current > end_duty:
                current -= STEP_SIZE
                if current < end_duty: current = end_duty
                self.pwm.ChangeDutyCycle(current)
                time.sleep(STEP_DELAY)

        self.pwm.ChangeDutyCycle(end_duty)
        time.sleep(0.2)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received Drop Command. Actuating Servo...')
        
        if not self.hardware_ready:
            self.get_logger().error("Hardware not ready, aborting action.")
            goal_handle.abort()
            return DropPayload.Result(success=False)

        self.slow_move(CLOSED_DUTY, OPEN_DUTY)

        self.pwm.ChangeDutyCycle(0)
        
        feedback_msg = DropPayload.Feedback()
        feedback_msg.status = "Dropping"
        goal_handle.publish_feedback(feedback_msg)
        
        time.sleep(3.0) 
        
        self.slow_move(OPEN_DUTY, CLOSED_DUTY)
        self.pwm.ChangeDutyCycle(0)
        
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