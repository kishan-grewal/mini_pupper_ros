# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2025 MangDang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
LiDAR processing utilities for person distance measurement.
"""

import math
from typing import Optional, List
from sensor_msgs.msg import LaserScan

# Camera specifications
FOV_DEG = 62.2
FOV_RAD = math.radians(FOV_DEG)


def _camera_to_lidar_angle(center_x: float) -> float:
    """Convert camera center_x coordinate to LiDAR angle."""
    return (0.5 - center_x) * FOV_RAD + 3*math.pi/2


def _angle_to_index(angle: float, laser_scan: LaserScan) -> int:
    """Convert LiDAR angle to array index."""
    raw_index = (angle - laser_scan.angle_min) / laser_scan.angle_increment
    return max(0, min(int(round(raw_index)), len(laser_scan.ranges) - 1))


def _is_valid_distance(distance: float, laser_scan: LaserScan) -> bool:
    """Check if a distance measurement is valid."""
    return (not math.isinf(distance) and
            not math.isnan(distance) and
            laser_scan.range_min <= distance <= laser_scan.range_max)


def _find_nearest_valid_distance(center_index: int, ranges: List[float],
                                 laser_scan: LaserScan, max_search: int = 10) -> Optional[float]:
    """Search for nearest valid distance around center index."""
    for offset in range(1, max_search + 1):
        for index in [center_index - offset, center_index + offset]:
            if 0 <= index < len(ranges):
                distance = ranges[index]
                if _is_valid_distance(distance, laser_scan):
                    return distance
    return None


def get_distance(center_x: float, laser_scan: LaserScan, max_search: int = 10) -> Optional[float]:
    """
    Get LiDAR distance measurement for a camera detection.

    Args:
        center_x: Normalized horizontal position in camera frame (0.0-1.0)
        laser_scan: ROS LaserScan message from 2D LiDAR
        max_search: Maximum number of neighboring rays to search

    Returns:
        Distance in meters, or None if no valid measurement found
    """
    if not isinstance(center_x, (float, int)) or not (0.0 <= center_x <= 1.0):
        return None

    if not laser_scan.ranges:
        return None

    # Convert camera position to LiDAR angle
    lidar_angle = _camera_to_lidar_angle(center_x)

    # Check if angle is within LiDAR scanning range
    if not (laser_scan.angle_min <= lidar_angle <= laser_scan.angle_max):
        return None

    # Find corresponding LiDAR ray index
    desired_index = _angle_to_index(lidar_angle, laser_scan)

    # Try direct measurement first
    distance = laser_scan.ranges[desired_index]
    if _is_valid_distance(distance, laser_scan):
        return distance

    # Fallback: search neighboring rays
    return _find_nearest_valid_distance(desired_index, laser_scan.ranges, laser_scan, max_search)
