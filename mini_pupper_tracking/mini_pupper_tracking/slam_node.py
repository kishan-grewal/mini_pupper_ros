import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from mini_pupper_interfaces.msg import TrackingArray
from tf_transformations import euler_from_quaternion
import math
import numpy as np
from .lidar_processor import get_distance

GRID_METERS = 5.0
GRID_RESOLUTION = 0.05
HIT = 0.8
MISS = 0.2


def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


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

        self.scan_count = 0

        # Robot state
        self.current_yaw = 0.0
        self.robot_x = 0.0  # Fixed at origin since robot doesn't translate
        self.robot_y = 0.0

        self.fov_deg = 62.2
        self.fov_rad = math.radians(self.fov_deg)

        # Subscriptions
        self.lidar_subscriber = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, 10)
        self.lidar = None
        
        self.imu_subscriber = self.create_subscription(
            Imu, "imu/data_filtered_madgwick", self.imu_callback, 10)
        
        self.tracking_array_subscriber = self.create_subscription(
            TrackingArray, "/tracking_array", self.tracking_callback, 10)
        self.detected = False
        self.left_x = None
        self.center_x = None
        self.right_x = None
        
        # Publishers
        self.grid_publisher = self.create_publisher(
            Float32MultiArray, "/occupancy_grid_raw", 10)
        self.grid_timer = self.create_timer(0.2, self.publish_grid_data)

        self.left_lidar_angle = 0.0
        self.right_lidar_angle = 0.0
        self.angle_debug_timer = self.create_timer(0.5, self.angle_debug_callback)

    def angle_debug_callback(self):
        try:
            result = self.get_angle_vectorised(self.left_x)
            if result is not None:
                lidar_angle, camera_angle = result
                self.left_lidar_angle = lidar_angle
                distance = get_distance(self.left_lidar_angle, self.lidar)
                if distance is not None:
                    self.get_logger().info(f"LEFT lidar:{self.left_lidar_angle:.2f},distance:{distance:.2f}")
                else:
                    self.get_logger().info(f"LEFT lidar:{self.left_lidar_angle:.2f},distance:None")
            else:
                self.get_logger().warning("LEFT: get_angle_vectorised returned None")
        except Exception as e:
            self.get_logger().error(f"LEFT error:{e}")
        
        try:
            result = self.get_angle_vectorised(self.right_x)
            if result is not None:
                lidar_angle, camera_angle = result
                self.right_lidar_angle = lidar_angle
                distance = get_distance(self.right_lidar_angle, self.lidar)
                if distance is not None:
                    self.get_logger().info(f"RIGHT lidar:{self.right_lidar_angle:.2f},distance:{distance:.2f}")
                else:
                    self.get_logger().info(f"RIGHT lidar:{self.right_lidar_angle:.2f},distance:None")
            else:
                self.get_logger().warning("RIGHT: get_angle_vectorised returned None")
        except Exception as e:
            self.get_logger().error(f"RIGHT error:{e}")

    def get_angle_vectorised(self, x_value) -> float:
        if not self.detected or self.center_x is None:
            self.get_logger().warning("NOT DETECTED")
            return None
        theta_c = (x_value - 0.5) * self.fov_rad
        v_c = np.array([math.cos(theta_c), math.sin(theta_c)])
        ranges = np.array(self.lidar.ranges)
        angles = self.lidar.angle_min + np.arange(len(ranges)) * self.lidar.angle_increment       
        hit_x = ranges * np.cos(angles)
        hit_y = ranges * np.sin(angles)
        diff_x = hit_x - 0.14 # 14 cm
        diff_y = hit_y - 0.00
        perp_distances = np.abs(diff_x * v_c[1] - diff_y * v_c[0])
        valid_mask = (~np.isinf(ranges)) & (~np.isnan(ranges)) & \
                    (ranges >= self.lidar.range_min) & (ranges <= self.lidar.range_max)
        if not np.any(valid_mask):
            return None
        valid_distances = perp_distances[valid_mask]
        valid_indices = np.where(valid_mask)[0]
        best_idx = valid_indices[np.argmin(valid_distances)]
        return float(normalize_angle(angles[best_idx])), theta_c
    
    def tracking_callback(self, msg: TrackingArray):
        if not msg.tracks:
            self.detected = False
            return

        # Pick detection with highest area
        choice = max(msg.tracks, key=lambda t: t.bounding_area)

        self.left_x = choice.left_x
        self.center_x = choice.center_x
        self.right_x = choice.right_x
        self.detected = True
        
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

        # Store initial yaw as offset on first IMU message
        if not hasattr(self, 'initial_yaw'):
            self.initial_yaw = yaw
            self.get_logger().info(f"Set initial yaw offset: {yaw:.3f} rad ({math.degrees(yaw):.1f}°)")

        # Use relative yaw (current - initial)
        self.current_yaw = yaw - self.initial_yaw

    def lidar_callback(self, msg: LaserScan):
        if self.current_yaw is None:
            return
        
        self.scan_count += 1

        self.lidar = msg

        # Process each ray in the LiDAR scan
        for i, distance in enumerate(msg.ranges):
            ray_angle = msg.angle_min + i * msg.angle_increment

            self.sensor_model.process_lidar_ray(
                self.grid,
                self.robot_x, self.robot_y, self.current_yaw,
                ray_angle - math.pi, distance, msg.range_max
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