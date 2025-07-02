#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from mini_pupper_interfaces.msg import Tracking
from tf_transformations import euler_from_quaternion
import math
import numpy as np
import subprocess
from enum import Enum


def stop_rover_manually():
    """
    Publishes a zero-velocity Twist message once to stop the robot.
    """
    cmd = [
        "ros2", "topic", "pub", "/cmd_vel", "geometry_msgs/msg/Twist",
        "{linear: {x: 0.0}, angular: {z: 0.0}}", "--once"
    ]
    try:
        subprocess.run(cmd, check=True)
        print("Stop command sent from Python script.")
    except subprocess.CalledProcessError as e:
        print(f"Failed to run ros2 stop command: {e}")


class PID:
    def __init__(self, Kp, Ki, Kd):
        self.K = [Kp, Ki, Kd]
        self.prev_error = 0
        self.integral = 0
        # Add storage for component values
        self.last_p = 0
        self.last_i = 0
        self.last_d = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        # Store individual components
        self.last_p = self.K[0] * error
        self.last_i = self.K[1] * self.integral
        self.last_d = self.K[2] * derivative
        
        self.prev_error = error
        return self.last_p + self.last_i + self.last_d
    

class MovementNode(Node):
    def __init__(self):
        super().__init__('mini_pupper_movement_node')
        self.get_logger().info("Movement Node Created")

        self.velpub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.tracksub = self.create_subscription(Tracking, "/tracking", self.tracking_callback, 10)
        self.imusub = self.create_subscription(Imu, "/imu/qdata", self.imu_callback, 10)
        
        # Tracking variables
        self.detected = False
        self.center_x = 0.0
        self.center_y = 0.0
        self.bounding_area = 0.0

        self.turn_pid = PID(1.5, 0.0, 0.05)
        # Average derivative is around 5.0 (0-10)
        self.turn_dt = 1 / 30.0
        self.turn_timer = self.create_timer(self.turn_dt, self.turn_callback)
        self.last_turn = 0.0

        self.pid_log_timer = self.create_timer(0.2, self.log_pid_data)

        self.current_yaw = 0.0
        self.fov_deg = 62.2
        self.fov_rad = math.radians(self.fov_deg)
        self.last_target_yaw = None
        self.turn_decay = 0.5
        self.turn_clamp = 2.0
        self.angle_deadzone = 0.1 # radians
        self.dead = False
    
    def imu_callback(self, msg: Imu):
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.current_yaw = yaw

    def log_pid_data(self):
        if self.detected:
            if not self.dead:
                self.get_logger().info(
                    f"P={self.turn_pid.last_p:.3f} "
                    f"I={self.turn_pid.last_i:.3f} "
                    f"D={self.turn_pid.last_d:.3f} "
                    f"Total={self.turn_pid.last_p + self.turn_pid.last_i + self.turn_pid.last_d:.3f} "
                    f"Yaw={self.current_yaw:.3f}"
                    f"\n"
                )
            else:
                self.get_logger().info("DEAD")

    def tracking_callback(self, msg):
        self.detected = msg.detected
        if self.detected:
            self.center_x = msg.center_x
            self.center_y = msg.center_y
            self.bounding_area = msg.bounding_area
    

    def turn_callback(self):
        twist = Twist()
        twist.linear.x = 0.0

        if self.detected:
            offset_angle = (self.center_x - 0.5) * self.fov_rad

            if abs(offset_angle) < self.angle_deadzone:
                self.dead = True
                twist.angular.z = 0.0
                self.velpub.publish(twist)
                return
            else:
                self.dead = False
                desired_yaw = self.current_yaw + offset_angle
                self.last_target_yaw = desired_yaw

                yaw_error = math.atan2(math.sin(desired_yaw - self.current_yaw),
                                        math.cos(desired_yaw - self.current_yaw))
                twist.angular.z = self.turn_pid.compute(-yaw_error, self.turn_dt)

        elif self.last_target_yaw is not None:
            self.dead = False
            yaw_error = math.atan2(math.sin(self.last_target_yaw - self.current_yaw),
                                    math.cos(self.last_target_yaw - self.current_yaw))
            twist.angular.z = self.turn_pid.compute(-yaw_error, self.turn_dt)

        else:
            self.dead = False
            self.last_turn = self.last_turn * self.turn_decay
            twist.angular.z = self.last_turn

        twist.angular.z = max(-self.turn_clamp, min(self.turn_clamp, twist.angular.z))
        self.last_turn = twist.angular.z
        self.velpub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    stop_rover_manually()


if __name__ == '__main__':
    main()