import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import numpy as np
from sensor_msgs_py import point_cloud2
import struct


class PointCloudToLaserScan(Node):
    def __init__(self):
        super().__init__("pointcloud_to_laserscan")

        # Parameters
        self.declare_parameter("min_height", -0.5)
        self.declare_parameter("max_height", 0.5)
        self.declare_parameter("angle_min", -np.pi)
        self.declare_parameter("angle_max", np.pi)
        self.declare_parameter("angle_increment", np.pi / 180.0)  # 1 degree
        self.declare_parameter("scan_time", 0.1)
        self.declare_parameter("range_min", 0.1)
        self.declare_parameter("range_max", 30.0)

        # Get parameters
        self.min_height = (
            self.get_parameter("min_height").get_parameter_value().double_value
        )
        self.max_height = (
            self.get_parameter("max_height").get_parameter_value().double_value
        )
        self.angle_min = (
            self.get_parameter("angle_min").get_parameter_value().double_value
        )
        self.angle_max = (
            self.get_parameter("angle_max").get_parameter_value().double_value
        )
        self.angle_increment = (
            self.get_parameter("angle_increment").get_parameter_value().double_value
        )
        self.scan_time = (
            self.get_parameter("scan_time").get_parameter_value().double_value
        )
        self.range_min = (
            self.get_parameter("range_min").get_parameter_value().double_value
        )
        self.range_max = (
            self.get_parameter("range_max").get_parameter_value().double_value
        )

        # Calculate number of beams
        self.num_beams = int((self.angle_max - self.angle_min) / self.angle_increment)

        # Publishers and subscribers
        self.subscription = self.create_subscription(
            PointCloud2, "/cloud_in", self.pointcloud_callback, 10
        )

        self.publisher = self.create_publisher(LaserScan, "/scan", 10)

        self.get_logger().info(f"PointCloud to LaserScan converter started")
        self.get_logger().info(f"Listening on: /cloud_in")
        self.get_logger().info(f"Publishing to: /scan")
        self.get_logger().info(f"Height filter: {self.min_height} to {self.max_height}")
        self.get_logger().info(f"Range: {self.range_min} to {self.range_max}")

    def pointcloud_callback(self, cloud_msg):
        try:
            # Create laser scan message
            scan = LaserScan()
            scan.header = cloud_msg.header
            scan.angle_min = self.angle_min
            scan.angle_max = self.angle_max
            scan.angle_increment = self.angle_increment
            scan.time_increment = 0.0
            scan.scan_time = self.scan_time
            scan.range_min = self.range_min
            scan.range_max = self.range_max

            # Initialize ranges with max range
            ranges = [self.range_max] * self.num_beams

            # Convert point cloud to list of points
            points_list = list(
                point_cloud2.read_points(
                    cloud_msg, skip_nans=True, field_names=("x", "y", "z")
                )
            )

            if not points_list:
                self.get_logger().warn("No valid points in point cloud")
                return

            # Process each point
            valid_points = 0
            for point in points_list:
                x, y, z = float(point[0]), float(point[1]), float(point[2])

                # Filter by height
                if not (self.min_height <= z <= self.max_height):
                    continue

                # Calculate range and angle
                range_val = float(np.sqrt(x * x + y * y))

                # Skip if out of range limits
                if not (self.range_min <= range_val <= self.range_max):
                    continue

                angle = float(np.arctan2(y, x))

                # Check if angle is within scan range
                if not (self.angle_min <= angle <= self.angle_max):
                    continue

                # Calculate beam index
                beam_index = int((angle - self.angle_min) / self.angle_increment)

                # Ensure beam index is valid
                if 0 <= beam_index < self.num_beams:
                    # Take the closest range for each beam
                    if range_val < ranges[beam_index]:
                        ranges[beam_index] = float(range_val)
                        valid_points += 1

            # Ensure all ranges are proper Python floats
            ranges = [float(r) for r in ranges]

            scan.ranges = ranges
            self.publisher.publish(scan)

            if valid_points > 0:
                self.get_logger().info(
                    f"Converted {valid_points} points to laser scan",
                    throttle_duration_sec=2.0,
                )
            else:
                self.get_logger().warn(
                    "No valid points found in height range", throttle_duration_sec=5.0
                )

        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScan()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
