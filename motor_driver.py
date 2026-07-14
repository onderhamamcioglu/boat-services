import argparse
import sys
import time

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float64


DEFAULT_PORT = '/dev/ttyTHS1'  # Jetson Nano 40-pin header UART (TX pin 8, RX pin 10)
DEFAULT_BAUD = 115200
SEND_RATE_HZ = 20.0            # keepalive rate; Arduino failsafes to neutral after 1 s of silence
COMMAND_TIMEOUT_S = 1.0


class MotorDriverNode(Node):
    def __init__(self, port, baud):
        super().__init__('motor_driver')

        self.shutdown_requested = False
        self.last_left_command_time = self.get_clock().now()
        self.last_right_command_time = self.get_clock().now()

        try:
            self.arduino = serial.Serial(port=port, baudrate=baud, timeout=0, write_timeout=0.1)
        except serial.SerialException as error:
            self.get_logger().fatal(f'Cannot open serial port {port}: {error}')
            raise

        self.get_logger().info(
            f'Connected to Arduino on {port} at {baud} baud. '
            'Listening for motor percentage commands.'
        )

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

        self.create_timer(1.0 / SEND_RATE_HZ, self.send_motor_commands)
        self.create_timer(0.1, self.enforce_command_timeout)

    def clamp_percentage(self, value):
        return max(-100.0, min(100.0, float(value)))

    def handle_left_command(self, msg):
        self.left_percent = self.clamp_percentage(msg.data)
        self.last_left_command_time = self.get_clock().now()
        self.get_logger().info(
            f'[motor_l] Input: {self.left_percent:.2f}%',
            throttle_duration_sec=1.0,
        )

    def handle_right_command(self, msg):
        self.right_percent = self.clamp_percentage(msg.data)
        self.last_right_command_time = self.get_clock().now()
        self.get_logger().info(
            f'[motor_r] Input: {self.right_percent:.2f}%',
            throttle_duration_sec=1.0,
        )

    def send_motor_commands(self):
        line = f'L:{self.left_percent:.2f},R:{self.right_percent:.2f}\n'
        try:
            self.arduino.write(line.encode('ascii'))
        except serial.SerialException as error:
            self.get_logger().error(f'Serial write failed: {error}')

    def enforce_command_timeout(self):
        now = self.get_clock().now()
        timeout_ns = int(COMMAND_TIMEOUT_S * 1_000_000_000)

        if (now - self.last_left_command_time).nanoseconds > timeout_ns:
            if self.left_percent != 0.0:
                self.left_percent = 0.0
                self.get_logger().info('[motor_l] No recent input, commanding neutral.')
            self.last_left_command_time = now

        if (now - self.last_right_command_time).nanoseconds > timeout_ns:
            if self.right_percent != 0.0:
                self.right_percent = 0.0
                self.get_logger().info('[motor_r] No recent input, commanding neutral.')
            self.last_right_command_time = now

    def destroy_node(self):
        if not self.shutdown_requested:
            self.shutdown_requested = True
            try:
                self.arduino.write(b'L:0.00,R:0.00\n')
                self.arduino.flush()
                time.sleep(0.1)
            except serial.SerialException:
                pass

            try:
                self.arduino.close()
            except serial.SerialException:
                pass

            self.get_logger().info('Neutral sent and serial port closed.')

        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='ROS 2 motor driver node')
    parser.add_argument('--port', default=DEFAULT_PORT, help='Serial port of the Arduino')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD, help='Serial baud rate')
    parsed_args, _ = parser.parse_known_args(args=sys.argv[1:])

    rclpy.init(args=args)
    node = MotorDriverNode(parsed_args.port, parsed_args.baud)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Motor driver stopped by the user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
