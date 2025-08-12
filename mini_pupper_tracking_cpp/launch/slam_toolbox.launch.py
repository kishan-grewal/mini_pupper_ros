# !/usr/bin/env python3
#
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

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[{
                'use_sim_time': False,
                'mode': 'mapping',
                'map_file_name': 'my_map'
            }]
        )
    ])

# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='slam_toolbox',
#             executable='sync_slam_toolbox_node',
#             name='slam_toolbox',
#             parameters=[{
#                 'use_sim_time': False,
#                 'mode': 'mapping',
#                 'map_file_name': 'my_map',

#                 # scan matching: tuned for indoor spaces
#                 'minimum_travel_distance': 0.12,
#                 'minimum_travel_heading': 0.087,
#                 'scan_buffer_size': 30,
#                 'transform_tolerance': 0.4,

#                 # range limits: indoor optimized
#                 'max_laser_range': 3.5,
#                 'minimum_range': 0.15,

#                 'map_update_interval': 1.5,        # Update map every 1.5s instead of constantly
#                 'processor_type': 'scan_buffer',   # Use buffered processing
#                 'transform_tolerance': 1.0,        # Increase from 0.4 to 1.0
#                 'tf_buffer_duration': 30.0,        # Much longer buffer
#                 'message_filter_tolerance': 1.0,   # Allow 1 second timing mismatch
#                 'scan_queue_size': 50,             # Bigger queue

#                 # loop closure: aggressive for indoor
#                 'loop_search_maximum_distance': 2.5,
#                 'do_loop_closing': True,
#                 'loop_match_minimum_chain_size': 8,
#                 'loop_search_space_dimension': 0.3,
#                 'loop_match_maximum_variance': 0.3,

#                 # optimisation: more aggressive indoors
#                 'optimization_scale': 1.0,
#                 'minimum_angle_penalty': 0.6,
#                 'minimum_time_interval': 0.08,

#                 # motion model: indoor specific
#                 'linear_update_distance': 0.08,
#                 'angular_update_distance': 0.04,
#                 'tf_buffer_duration': 8.0,

#                 # correlation: higher resolution for detail
#                 'correlation_search_space_dimension': 0.3,
#                 'correlation_search_space_resolution': 0.01,
#                 'correlation_search_space_smear_deviation': 0.1,

#                 # frames: match urdf
#                 'base_frame': 'base_link',
#                 'map_frame': 'map',
#                 'odom_frame': 'odom',
#             }]
#         )
#     ])
