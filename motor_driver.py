import argparse
import sys
import time

import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


ESC_PIN_L = 32      # BOARD numbering; hardware-PWM capable
ESC_PIN_R = 33      # BOARD numbering; hardware-PWM capable
FREQ = 500          # 500 Hz -> 2 ms period
NEUTRAL_DUTY = 75.0 # 1.5 ms pulse
ARM_TIME_S = 3.0    # hold neutral so the ESC can arm
COMMAND_TIMEOUT_S = 1.0


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver')

        self.shutdown_requested = False
        self.last_left_command_time = self.get_clock().now()
        self.last_right_command_time = self.get_clock().now()

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(ESC_PIN_L, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(ESC_PIN_R, GPIO.OUT, initial=GPIO.LOW)

        self.pwm_left = GPIO.PWM(ESC_PIN_L, FREQ)
        self.pwm_right = GPIO.PWM(ESC_PIN_R, FREQ)

        self.pwm_left.start(NEUTRAL_DUTY)
        self.pwm_right.start(NEUTRAL_DUTY)

        self.get_logger().info(
            f'Arming ESCs at {FREQ} Hz with {NEUTRAL_DUTY}% duty for {ARM_TIME_S} seconds.'
        )
        time.sleep(ARM_TIME_S)
        self.get_logger().info('ESCs armed. Listening for motor percentage commands.')

        self.left_percent = 0.0
        self.right_percent = 0.0

        self.create_subscription(
            Float64,
            'jetson/actuator/motor_l',
            self.handle_left_command,
            10,
        )
        self.create_subscription(
            Float64,
            'jetson/actuator/motor_r',
            self.handle_right_command,
            10,
        )

        self.create_timer(0.1, self.enforce_command_timeout)

    def clamp_percentage(self, value):
        return max(-100.0, min(100.0, float(value)))

    def percentage_to_duty(self, percentage):
        percentage = self.clamp_percentage(percentage)
        return NEUTRAL_DUTY + (percentage * 0.25)

    def set_neutral(self, pwm, motor_name):
        pwm.ChangeDutyCycle(NEUTRAL_DUTY)
        self.get_logger().info(f'[{motor_name}] No recent input, holding neutral duty: {NEUTRAL_DUTY:.2f}%')

    def apply_motor_command(self, pwm, motor_name, percentage):
        duty_cycle = self.percentage_to_duty(percentage)
        pwm.ChangeDutyCycle(duty_cycle)
        self.get_logger().info(
            f'[{motor_name}] Command: {percentage:.2f}% -> PWM duty: {duty_cycle:.2f}%'
        )

    def handle_left_command(self, msg):
        self.left_percent = self.clamp_percentage(msg.data)
        self.last_left_command_time = self.get_clock().now()
        self.apply_motor_command(self.pwm_left, 'motor_l', self.left_percent)

    def handle_right_command(self, msg):
        self.right_percent = self.clamp_percentage(msg.data)
        self.last_right_command_time = self.get_clock().now()
        self.apply_motor_command(self.pwm_right, 'motor_r', self.right_percent)

    def enforce_command_timeout(self):
        now = self.get_clock().now()
        timeout_ns = int(COMMAND_TIMEOUT_S * 1_000_000_000)

        if (now - self.last_left_command_time).nanoseconds > timeout_ns:
            if self.left_percent != 0.0:
                self.left_percent = 0.0
                self.set_neutral(self.pwm_left, 'motor_l')
            self.last_left_command_time = now

        if (now - self.last_right_command_time).nanoseconds > timeout_ns:
            if self.right_percent != 0.0:
                self.right_percent = 0.0
                self.set_neutral(self.pwm_right, 'motor_r')
            self.last_right_command_time = now

    def destroy_node(self):
        if not self.shutdown_requested:
            self.shutdown_requested = True
            try:
                self.pwm_left.ChangeDutyCycle(NEUTRAL_DUTY)
                self.pwm_right.ChangeDutyCycle(NEUTRAL_DUTY)
                time.sleep(0.1)
            except Exception:
                pass

            try:
                self.pwm_left.stop()
                self.pwm_right.stop()
            except Exception:
                pass

            GPIO.cleanup()
            self.get_logger().info('PWM stopped and GPIO cleaned up.')

        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='ROS 2 motor driver node')
    parser.parse_known_args(args=sys.argv[1:])

    rclpy.init(args=args)
    node = MotorDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Motor driver stopped by the user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()