import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from tf_transformations import euler_from_quaternion
import math

GRID_METERS = 5.0
GRID_RESOLUTION = 0.05
HIT = 0.8
MISS = 0.2

class SLAMNode(Node):
    def __init__(self):
        super().__init__('mini_pupper_slam_node')
        self.get_logger().info("SLAM Node Created")

        # Initialize your SLAM components
        from .occupancy_grid import OccupancyGrid
        from .sensor_model import LiDARSensorModel

        self.grid = OccupancyGrid(
            width_meters=GRID_METERS,
            height_meters=GRID_METERS,
            resolution=GRID_RESOLUTION)
        
        self.sensor_model = LiDARSensorModel(prob_hit=HIT, prob_miss=MISS)

        # Robot state
        self.current_yaw = 0.0
        self.robot_x = 0.0  # Fixed at origin since robot doesn't translate
        self.robot_y = 0.0

        # Subscriptions
        self.lidar_subscriber = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, 10)
        self.imu_subscriber = self.create_subscription(
            Imu, "imu/data_filtered_madgwick", self.imu_callback, 10)
        
        # Publishers
        self.grid_publisher = self.create_publisher(
            Float32MultiArray, "/occupancy_grid_raw", 10)
        self.grid_timer = self.create_timer(0.2, self.publish_grid_data)

        # Status logging
        self.scan_count = 0
        self.log_timer = self.create_timer(2.0, self.log_status)
        
    def publish_grid_data(self):
        msg = Float32MultiArray()
        msg.data = self.grid.grid.flatten().tolist()

        msg.layout.dim = [
            MultiArrayDimension(label="height", size=self.grid.height_cells, stride=self.grid.height_cells * self.grid.width_cells),
            MultiArrayDimension(label="width", size=self.grid.width_cells, stride=self.grid.width_cells)
        ]

        self.grid_publisher.publish(msg)

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.current_yaw = yaw

    def lidar_callback(self, msg: LaserScan):
        if self.current_yaw is None:
            return
            
        # Debug coordinate systems on first few scans
        if self.scan_count < 5:
            self.get_logger().info(f"=== SCAN {self.scan_count} DEBUG ===")
            self.get_logger().info(f"Robot yaw: {self.current_yaw:.3f} rad ({math.degrees(self.current_yaw):.1f}°)")
            self.get_logger().info(f"LiDAR angle range: {msg.angle_min:.3f} to {msg.angle_max:.3f} rad")
            self.get_logger().info(f"LiDAR angle range: {math.degrees(msg.angle_min):.1f}° to {math.degrees(msg.angle_max):.1f}°")
            
            # Check a few key rays
            for i in [0, len(msg.ranges)//4, len(msg.ranges)//2, 3*len(msg.ranges)//4]:
                if i < len(msg.ranges):
                    ray_angle = msg.angle_min + i * msg.angle_increment
                    distance = msg.ranges[i]
                    self.get_logger().info(f"Ray {i}: angle={math.degrees(ray_angle):.1f}°, distance={distance:.2f}m")
        
        self.scan_count += 1

        # Process each ray in the LiDAR scan
        for i, distance in enumerate(msg.ranges):
            ray_angle = msg.angle_min + i * msg.angle_increment

            self.sensor_model.process_lidar_ray(
                self.grid,
                self.robot_x, self.robot_y, self.current_yaw,
                ray_angle, distance, msg.range_max
            )

    def log_status(self):
        """Log SLAM progress"""
        occupied_cells = (self.grid.grid > 0.7).sum()
        free_cells = (self.grid.grid < 0.3).sum()
        unknown_cells = ((self.grid.grid >= 0.3) & (self.grid.grid <= 0.7)).sum()

        self.get_logger().info(
            f"SLAM: {self.scan_count} scans, "
            f"Occupied: {occupied_cells}, Free: {free_cells}, Unknown: {unknown_cells}, "
            f"Yaw: {self.current_yaw:.2f}rad"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SLAMNode()
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