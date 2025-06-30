import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import onnxruntime as ort
import cv2
import time
import os
from threading import Lock
from ament_index_python.packages import get_package_share_directory

IMAGE_SIZE = 320
CONFIDENCE_THRESHOLD = 0.8
ROS_IMAGE_TOPIC = "/image_raw"
MODEL_NAME = "yolo11n.onnx"
MODEL_PATH = os.path.join(
    get_package_share_directory('mini_pupper_tracking'), 'models', MODEL_NAME)

class TrackingNode(Node):
    def __init__(self):
        super().__init__('mini_pupper_tracking_node')

        self.subscription = self.create_subscription(Image, ROS_IMAGE_TOPIC, self.image_callback, 10)
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = Lock()
        self.last_processed = 0
        self.frame_counter = 0
        self.frame_skip = 2
        self.min_interval = 1.0 / 15

        self.sess = ort.InferenceSession(
            MODEL_PATH,
            providers=['CPUExecutionProvider']
        )
        self.get_logger().info("Model loaded")

    def image_callback(self, msg):
        now = time.time()
        self.frame_counter += 1

        if self.frame_counter % self.frame_skip != 0:
            return
        if (now - self.last_processed) < self.min_interval:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            processed = self.process_frame(cv_image)
            with self.frame_lock:
                self.latest_frame = processed
            self.last_processed = now
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")

    def process_frame(self, frame):
        img = cv2.resize(frame, (IMAGE_SIZE, IMAGE_SIZE))
        img = img.transpose(2, 0, 1)[np.newaxis].astype(np.float32) / 255.0
        outputs = self.sess.run(None, {"images": img})[0]
        predictions = np.squeeze(outputs).T
        scores = np.max(predictions[:, 4:], axis=1)
        class_ids = np.argmax(predictions[:, 4:], axis=1)
        boxes = predictions[:, :4]

        for i in range(len(class_ids)):
            if class_ids[i] == 0 and scores[i] > CONFIDENCE_THRESHOLD:
                cx, cy, w, h = boxes[i]
                x1 = int((cx - w/2) * frame.shape[1] / IMAGE_SIZE)
                y1 = int((cy - h/2) * frame.shape[0] / IMAGE_SIZE)
                x2 = int((cx + w/2) * frame.shape[1] / IMAGE_SIZE)
                y2 = int((cy + h/2) * frame.shape[0] / IMAGE_SIZE)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"Person: {scores[i]:.2f}", (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return frame
