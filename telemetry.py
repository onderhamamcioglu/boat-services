import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float64
import smbus2
import argparse
import sys

# Global interval variable
GLOBAL_PUBLISH_INTERVAL = 2.0

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry')
        
        global GLOBAL_PUBLISH_INTERVAL
        
        self.temp_publisher = self.create_publisher(Temperature, 'jetson/sensor/internal/temp', 10)
        self.voltage_publisher = self.create_publisher(Float64, 'jetson/sensor/i2c/battery/voltage', 10)
        self.percentage_publisher = self.create_publisher(Float64, 'jetson/sensor/i2c/battery/percentage', 10)
        
        # I2C Setup
        self.i2c_bus_number = 7
        self.i2c_address = 0x40
        self.voltage_register = 0x02
        
        try:
            self.bus = smbus2.SMBus(self.i2c_bus_number)
            self.get_logger().info(f'I2C bus {self.i2c_bus_number} opened successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to open I2C bus: {e}')
            self.bus = None
        
        # Timer for publishing all telemetry data
        self.timer = self.create_timer(GLOBAL_PUBLISH_INTERVAL, self.publish_telemetry)
        
        self.get_logger().info(f'Telemetry node initialized. Publishing interval: {GLOBAL_PUBLISH_INTERVAL} seconds.')

    def get_battery_percentage(self, voltage):
        max_v = 16.8
        min_v = 12.8

        if voltage >= max_v:
            return 100.0
        if voltage <= min_v:
            return 0.0

        percentage = ((voltage - min_v) / (max_v - min_v)) * 100
        return round(percentage, 2)

    def read_voltage(self):
        if self.bus is None:
            return 0.0
            
        try:
            raw_data = self.bus.read_i2c_block_data(self.i2c_address, self.voltage_register, 2)
            reg_value = (raw_data[0] << 8) | raw_data[1]
            bus_voltage = reg_value * 0.00125
            return bus_voltage
        except Exception as e:
            self.get_logger().warn(f'I2C read error: {e}')
            return 0.0

    def read_internal_temp(self):
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp_millicelsius = int(f.read().strip())
                return temp_millicelsius / 1000.0
        except FileNotFoundError:
            self.get_logger().error('Thermal zone file not found.')
            return None
        except Exception as e:
            self.get_logger().error(f'Error reading temperature: {e}')
            return None

    def publish_telemetry(self):
        # 1. Internal Temperature
        temp_celsius = self.read_internal_temp()
        if temp_celsius is not None:
            temp_msg = Temperature()
            temp_msg.header.stamp = self.get_clock().now().to_msg()
            temp_msg.header.frame_id = 'jetson_nano'
            temp_msg.temperature = float(temp_celsius)
            temp_msg.variance = 0.0  
            
            self.temp_publisher.publish(temp_msg)
            self.get_logger().info(f'[Temperature] Published: {temp_celsius:.2f} C')

        # 2. Battery Voltage and Percentage
        v_bus = self.read_voltage()
        if v_bus > 0:
            pct = self.get_battery_percentage(v_bus)
            
            # Publish Voltage as Float64
            voltage_msg = Float64()
            voltage_msg.data = float(round(v_bus, 2))
            self.voltage_publisher.publish(voltage_msg)
            
            # Publish Percentage as Float64
            pct_msg = Float64()
            pct_msg.data = float(pct)
            self.percentage_publisher.publish(pct_msg)
            
            self.get_logger().info(f'[Battery] Published: {v_bus:.2f}V | {pct}%')

    def destroy_node(self):
        if self.bus is not None:
            self.bus.close()
            self.get_logger().info('I2C bus closed.')
        super().destroy_node()

def main(args=None):
    parser = argparse.ArgumentParser(description='ROS2 Telemetry Node')
    parser.add_argument(
        '-i', '--interval', 
        type=float, 
        default=2.0, 
        help='Publishing interval in seconds (default: 2.0)'
    )
    
    parsed_args, ros_args = parser.parse_known_args(args=sys.argv[1:])
    
    global GLOBAL_PUBLISH_INTERVAL
    GLOBAL_PUBLISH_INTERVAL = parsed_args.interval
    
    rclpy.init(args=ros_args)
    node = TelemetryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Telemetry node stopped by the user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()