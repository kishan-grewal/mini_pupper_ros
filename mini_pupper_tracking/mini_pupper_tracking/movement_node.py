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
        """
        Initialise PID parameters and internal state variables.
        """
        # TODO: store Kp, Ki, Kd
        self.K = [Kp, Ki, Kd]
        # TODO: initialise prev_error and integral to 0
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        """
        Calculate the PID output based on current error and time delta.
        """
        # TODO:
        # - update integral
        self.integral += error * dt
        # - compute derivative
        derivative = (error - self.prev_error) / dt
        # - compute and return output: Kp*error + Ki*integral + Kd*derivative
        vec = [error, self.integral, derivative]
        self.prev_error = error
        #print(f"[PID] error={error:.2f}, dt={dt:.3f}, output={np.dot(vec, self.K):.2f}")
        return np.dot(vec, self.K)
    

class MovementNode(Node):
    def __init__(self):
        super().__init__('mini_pupper_movement_node')
        self.get_logger().info("Movement Node Created")

        self.velpub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.tracksub = self.create_subscription(Tracking, "/tracking", self.tracking_callback, 10)
        
        # Tracking variables
        self.detected = False
        self.center_x = 0.0
        self.center_y = 0.0
        self.bounding_area = 0.0
        
        # Create timer for logging (1 second interval)
        self.log_timer = None
        self.create_timer(1.0, 
            lambda: self.get_logger().info(
            f"Tracking Data [1s]: Detected={self.detected}, "
            f"X={self.center_x:.3f}, Y={self.center_y:.3f}, "
            f"Area={self.bounding_area:.3f}") if hasattr(self, 'detected') else None)

    def tracking_callback(self, msg):
        self.detected = msg.detected
        if msg.detected:
            self.center_x = msg.center_x
            self.center_y = msg.center_y
            self.bounding_area = msg.bounding_area


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