import argparse
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


# 0 -> 100 -> 0 in 25% steps, then repeats
RAMP_STEPS = [0.0, 25.0, 50.0, 75.0, 100.0, 75.0, 50.0, 25.0]
STEP_PERIOD_S = 1.0
PUBLISH_RATE_HZ = 10.0  # keep publishing between steps so the driver's 1 s timeout never trips


class MotorTesterNode(Node):
    def __init__(self, motors):
        super().__init__('motor_tester')

        self.motor_publishers = {}
        if motors in ('l', 'both'):
            self.motor_publishers['motor_l'] = self.create_publisher(
                Float64, 'jetson/actuator/motor_l', 10
            )
        if motors in ('r', 'both'):
            self.motor_publishers['motor_r'] = self.create_publisher(
                Float64, 'jetson/actuator/motor_r', 10
            )

        self.step_index = 0
        self.current_percent = RAMP_STEPS[0]

        targets = ', '.join(self.motor_publishers.keys())
        self.get_logger().info(
            f'Ramping {targets}: 0 -> 100 -> 0 in 25% steps, {STEP_PERIOD_S:.0f} s per step.'
        )
        self.get_logger().info('Press Ctrl+C to stop (motors return to neutral).')
        self.log_current_step()

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_current)
        self.create_timer(STEP_PERIOD_S, self.advance_step)

    def log_current_step(self):
        self.get_logger().info(
            f'Step {self.step_index + 1}/{len(RAMP_STEPS)}: '
            f'commanding {self.current_percent:.2f}%'
        )

    def publish_current(self):
        msg = Float64()
        msg.data = self.current_percent
        for publisher in self.motor_publishers.values():
            publisher.publish(msg)

    def advance_step(self):
        self.step_index = (self.step_index + 1) % len(RAMP_STEPS)
        self.current_percent = RAMP_STEPS[self.step_index]
        self.log_current_step()

    def send_neutral(self):
        msg = Float64()
        msg.data = 0.0
        for name, publisher in self.motor_publishers.items():
            publisher.publish(msg)
            self.get_logger().info(f'[{name}] Neutral (0.00%) sent.')


def main(args=None):
    parser = argparse.ArgumentParser(description='ROS 2 motor ramp tester')
    parser.add_argument(
        '--motor',
        choices=['l', 'r', 'both'],
        default='l',
        help='Which motor topic(s) to drive (default: l)',
    )
    parsed_args, ros_args = parser.parse_known_args(args=sys.argv[1:])

    rclpy.init(args=ros_args)
    node = MotorTesterNode(parsed_args.motor)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Motor tester stopped by the user.')
    finally:
        node.send_neutral()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
