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

        # Initialize SLAM components
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
        self.robot_x = 0.0
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
        self.detected_people = []

        # Publishers
        self.grid_publisher = self.create_publisher(
            Float32MultiArray, "/occupancy_grid_raw", 10)
        self.grid_timer = self.create_timer(0.2, self.publish_grid_data)

        # Person angle logging
        self.angle_debug_timer = self.create_timer(
            0.5, self.log_person_angles)

        # Working arrays
        self.ranges = None
        self.angles = None

    def lidar_ray_cleaner(self, best_idx: int, right_lidar_angle: bool,
                          center_x: float):
        """Clean lidar ray by searching for matching distance to center"""
        step = 1 if right_lidar_angle else -1
        threshold = 0.5
        center_distance = get_distance(center_x, self.lidar)
        if center_distance is None:
            return None

        lower = center_distance - threshold
        upper = center_distance + threshold

        while 0 <= best_idx < len(self.ranges):
            distance = self.ranges[best_idx]
            if lower <= distance <= upper:
                return best_idx
            best_idx += step

        return None

    def camera_to_lidar_ray_converter(self, x_value: float) -> int:
        """Convert camera x position to lidar ray index"""
        theta_c = (x_value - 0.5) * self.fov_rad
        v_c = np.array([math.cos(theta_c), math.sin(theta_c)])
        self.ranges = np.array(self.lidar.ranges)
        self.angles = (self.lidar.angle_min +
                      np.arange(len(self.ranges)) *
                      self.lidar.angle_increment)
        hit_x = self.ranges * np.cos(self.angles)
        hit_y = self.ranges * np.sin(self.angles)
        diff_x = hit_x - 0.14  # 14 cm camera offset
        diff_y = hit_y - 0.00
        perp_distances = np.abs(diff_x * v_c[1] - diff_y * v_c[0])
        valid_mask = ((~np.isinf(self.ranges)) & (~np.isnan(self.ranges)) &
                      (self.ranges >= self.lidar.range_min) &
                      (self.ranges <= self.lidar.range_max))
        if not np.any(valid_mask):
            return None
        valid_distances = perp_distances[valid_mask]
        valid_indices = np.where(valid_mask)[0]
        return valid_indices[np.argmin(valid_distances)]

    def get_person_angles(self, person):
        """Get cleaned left and right angles for a single person"""
        if not self.lidar:
            self.get_logger().warning("No lidar data")
            return None

        # Debug: Log person data
        self.get_logger().warning(
            f"----------- "
            f"Person data: left_x={person.left_x:.3f}, "
            f"center_x={person.center_x:.3f}, right_x={person.right_x:.3f}")

        # Step 1: Get raw lidar ray indices
        left_idx = self.camera_to_lidar_ray_converter(person.left_x)
        right_idx = self.camera_to_lidar_ray_converter(person.right_x)

        if left_idx is None:
            self.get_logger().warning("Failed: left_idx is None")
            return None
        if right_idx is None:
            self.get_logger().warning("Failed: right_idx is None")
            return None

        self.get_logger().warning(f"Raw indices: left={left_idx}, right={right_idx}")

        # Step 2: Get raw angles
        left_angle_raw = self.angles[left_idx]
        right_angle_raw = self.angles[right_idx]

        # Step 3: Check 1 radian consistency with camera angles
        left_camera_angle = (person.left_x - 0.5) * self.fov_rad
        right_camera_angle = (person.right_x - 0.5) * self.fov_rad

        left_diff = abs(normalize_angle(left_angle_raw - left_camera_angle))
        right_diff = abs(normalize_angle(right_angle_raw - right_camera_angle))

        self.get_logger().warning(
            f"Angle diffs: left={left_diff:.3f}, right={right_diff:.3f}")

        if left_diff > 1.0 or right_diff > 1.0:
            self.get_logger().warning(
                f"Failed: angle consistency check "
                f"(left_diff={left_diff:.3f}, right_diff={right_diff:.3f})")
            return None

        # Step 4: Clean the indices
        left_clean_idx = self.lidar_ray_cleaner(
            left_idx, right_lidar_angle=False, center_x=person.center_x)
        right_clean_idx = self.lidar_ray_cleaner(
            right_idx, right_lidar_angle=True, center_x=person.center_x)

        if left_clean_idx is None:
            self.get_logger().warning("Failed: left_clean_idx is None")
            return None
        if right_clean_idx is None:
            self.get_logger().warning("Failed: right_clean_idx is None")
            return None

        self.get_logger().warning(
            f"Clean indices: left={left_clean_idx}, right={right_clean_idx}")

        # Step 5: Return final cleaned angles
        left_angle_final = self.angles[left_clean_idx]
        right_angle_final = self.angles[right_clean_idx]

        return (left_angle_final, right_angle_final)

    def log_person_angles(self):
        """Log angles for all detected people every 0.5 seconds"""
        if not self.detected_people:
            self.get_logger().warning("No detected people")
            return

        self.get_logger().info("=== PERSON ANGLES ===")
        for i, person in enumerate(self.detected_people):
            angles = self.get_person_angles(person)
            if angles:
                left_angle, right_angle = angles
                self.get_logger().info(
                    f"Person {i+1}: left={left_angle:.3f}, "
                    f"right={right_angle:.3f}, "
                    f"track_id={person.track_id}")
            else:
                self.get_logger().info(
                    f"Person {i+1}: No valid angles, "
                    f"track_id={person.track_id}")

    def tracking_callback(self, msg: TrackingArray):
        """Store all detected people"""
        self.detected_people = msg.tracks

    def publish_grid_data(self):
        """Publish occupancy grid data"""
        msg = Float32MultiArray()
        msg.data = self.grid.grid.flatten().tolist()

        msg.layout.dim = [
            MultiArrayDimension(
                label="height",
                size=self.grid.height_cells,
                stride=self.grid.height_cells * self.grid.width_cells),
            MultiArrayDimension(
                label="width",
                size=self.grid.width_cells,
                stride=self.grid.width_cells)
        ]

        self.grid_publisher.publish(msg)

    def imu_callback(self, msg: Imu):
        """Process IMU data for robot orientation"""
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        # Store initial yaw as offset on first IMU message
        if not hasattr(self, 'initial_yaw'):
            self.initial_yaw = yaw
            self.get_logger().info(
                f"Set initial yaw offset: {yaw:.3f} rad "
                f"({math.degrees(yaw):.1f}°)")

        # Use relative yaw (current - initial)
        self.current_yaw = yaw - self.initial_yaw

    def lidar_callback(self, msg: LaserScan):
        """Process LiDAR scan for SLAM mapping"""
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