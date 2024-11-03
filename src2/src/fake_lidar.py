import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import csv
import os


class FakeLidar(Node):
    def __init__(self):
        super().__init__('fake_lidar')
        self.declare_parameter('csv_file', '/path/to/lidar_data.csv')
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('gaussian_noise_stddev', 0.0)
        self.declare_parameter('publish_rate', 10)

        self.csv_file = self.get_parameter('csv_file').get_parameter_value().string_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.noise_stddev = self.get_parameter('gaussian_noise_stddev').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().integer_value

        self.data = self.load_csv_data(self.csv_file)

        self.publisher = self.create_publisher(LaserScan, 'fake_lidar_scan', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_scan)

    def load_csv_data(self, csv_file):
        data = []
        if not os.path.isfile(csv_file):
            self.get_logger().error(f"CSV file {csv_file} not found.")
            return data

        with open(csv_file, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                data.append([float(x) for x in row])
        return data

    def publish_scan(self):
        if not self.data:
            self.get_logger().error("No data loaded.")
            return

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'fake_lidar'

        scan.angle_min = 0.0
        scan.angle_max = 2.0 * np.pi
        scan.angle_increment = 2.0 * np.pi / len(self.data[0])
        scan.range_min = 0.0
        scan.range_max = self.max_range

        for measurement in self.data:
            noisy_measurement = []
            for distance in measurement:
                if distance > self.max_range:
                    self.get_logger().error(f"Measurement {distance} exceeds max range {self.max_range}")
                    continue
                noise = np.random.normal(0, self.noise_stddev) if self.noise_stddev > 0 else 0
                noisy_measurement.append(distance + noise)
            scan.ranges = noisy_measurement

        self.publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    fake_lidar = FakeLidar()
    rclpy.spin(fake_lidar)
    fake_lidar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
