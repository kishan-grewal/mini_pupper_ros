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

from flask import Flask, Response
import cv2
import time
import numpy as np


def convert_grid_to_image(grid_data, target_size=640):
    """Convert occupancy grid to displayable image"""
    if grid_data is None:
        return None

    # Convert probabilities to grayscale (0-255)
    grid_img = (grid_data * 255).astype(np.uint8)

    # Flip vertically for proper display orientation
    grid_img = cv2.flip(grid_img, 0)

    # Resize for display
    grid_img = cv2.resize(
        grid_img, (target_size, target_size), interpolation=cv2.INTER_NEAREST)

    return grid_img


def create_flask_app(node, flask_config):
    app = Flask(__name__)

    @app.route('/')
    def index():
        image_display_size = flask_config.get('image_display_size', 640)
        return f'''
        <h2>Mini Pupper Tracking & SLAM</h2>
        <div style="display: flex; gap: 20px;">
            <div>
                <h3>Camera Feed</h3>
                <img src='/video_feed' width='{image_display_size}'>
            </div>
            <div>
                <h3>Occupancy Grid</h3>
                <img src='/occupancy_grid' width='640' style="border: 1px solid #ccc;">
                <p style="font-size: 12px;">Black=Free, Gray=Unknown, White=Occupied</p>
            </div>
        </div>
        '''

    @app.route('/video_feed')
    def video_feed():
        def generate():
            frame_rate = flask_config.get('frame_rate', 15)
            while True:
                time.sleep(1 / frame_rate)

                try:
                    # Non-blocking frame access with timeout
                    if node.frame_lock.acquire(timeout=0.1):
                        try:
                            frame = (node.latest_frame.copy()
                                     if node.latest_frame is not None else None)
                        finally:
                            node.frame_lock.release()
                    else:
                        # Skip this frame if lock can't be acquired
                        continue

                    if frame is None:
                        continue

                    success, buffer = cv2.imencode('.jpg', frame)
                    if not success:
                        continue

                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

                except Exception as e:
                    node.get_logger().error(f"Flask video feed error: {e}")
                    continue

        return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/occupancy_grid')
    def occupancy_grid():
        def generate_grid():
            while True:
                time.sleep(0.2)
                try:
                    # Get raw grid data
                    if node.grid_lock.acquire(timeout=0.2):
                        try:
                            grid_data = node.latest_grid_data.copy() if node.latest_grid_data is not None else None
                        finally:
                            node.grid_lock.release()
                    else:
                        continue
                    
                    # Convert to image
                    grid_img = convert_grid_to_image(grid_data)
                    if grid_img is not None:
                        success, buffer = cv2.imencode('.png', grid_img)
                        if success:
                            yield (b'--frame\r\n'
                                   b'Content-Type: image/png\r\n\r\n' + buffer.tobytes() + b'\r\n')
                except Exception as e:
                    node.get_logger().error(f"Grid streaming error: {e}")
                    continue

        return Response(generate_grid(), mimetype='multipart/x-mixed-replace; boundary=frame')

    return app
