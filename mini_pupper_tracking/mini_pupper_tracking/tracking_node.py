import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from mini_pupper_interfaces.msg import Tracking
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
IOU_THRESHOLD = 0.4 # For NMS
MODEL_NAME = "yolo11n.onnx" 
MODEL_PATH = os.path.join(
    get_package_share_directory('mini_pupper_tracking'), 'models', MODEL_NAME)

class TrackingNode(Node):
    def __init__(self):
        super().__init__('mini_pupper_tracking_node')
        self.get_logger().info("Tracking Node Created")

        self.subscription = self.create_subscription(Image, "/image_raw", self.image_callback, 10)
        self.publisher = self.create_publisher(Tracking, "/tracking", 10)
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = Lock()
        self.last_processed = 0
        self.frame_counter = 0
        self.frame_skip = 1
        self.min_interval = 1.0 / 30

        # Variables for publisher
        self.center_x = 0.0        # Normalized x-position [0-1] (0=left, 1=right)
        self.center_y = 0.0        # Normalized y-position [0-1] (0=top, 1=bottom)
        self.bounding_area = 0.0   # Normalized area [0-1]

        self.sess = ort.InferenceSession(
            MODEL_PATH,
            providers=['CPUExecutionProvider']
        )
        self.get_logger().info(f"Loaded model: {MODEL_NAME}")

    def _apply_nms(self, boxes, scores, iou_threshold):
        """Non-Maximum Suppression to remove overlapping boxes"""
        # Convert from [cx, cy, w, h] to [x1, y1, x2, y2]
        x1 = boxes[:, 0] - boxes[:, 2] / 2
        y1 = boxes[:, 1] - boxes[:, 3] / 2
        x2 = boxes[:, 0] + boxes[:, 2] / 2
        y2 = boxes[:, 1] + boxes[:, 3] / 2
        boxes = np.column_stack([x1, y1, x2, y2])
        
        # Sort by descending confidence
        order = scores.argsort()[::-1]
        keep = []
        
        while order.size > 0:
            i = order[0]
            keep.append(i)
            
            # Compute IoU between current box and remaining
            xx1 = np.maximum(boxes[i, 0], boxes[order[1:], 0])
            yy1 = np.maximum(boxes[i, 1], boxes[order[1:], 1])
            xx2 = np.minimum(boxes[i, 2], boxes[order[1:], 2])
            yy2 = np.minimum(boxes[i, 3], boxes[order[1:], 3])
            
            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)
            intersection = w * h
            
            area_i = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
            area_j = (boxes[order[1:], 2] - boxes[order[1:], 0]) * (boxes[order[1:], 3] - boxes[order[1:], 1])
            union = area_i + area_j - intersection
            
            iou = intersection / (union + 1e-7)  # Avoid division by zero
            
            # Keep boxes with IoU < threshold
            inds = np.where(iou <= iou_threshold)[0]
            order = order[inds + 1]
        
        return keep

    def _preprocess_frame(self, frame):
        """Resize while preserving aspect ratio and pad to square"""
        h, w = frame.shape[:2]
        scale = IMAGE_SIZE / max(h, w)
        new_h, new_w = int(h * scale), int(w * scale)
        resized = cv2.resize(frame, (new_w, new_h))
        
        # Pad to IMAGE_SIZE x IMAGE_SIZE
        top = (IMAGE_SIZE - new_h) // 2
        bottom = IMAGE_SIZE - new_h - top
        left = (IMAGE_SIZE - new_w) // 2
        right = IMAGE_SIZE - new_w - left
        padded = cv2.copyMakeBorder(resized, top, bottom, left, right, 
                                   cv2.BORDER_CONSTANT, value=(114, 114, 114))
        return padded, (scale, left, top)

    def image_callback(self, msg):
        now = time.time()
        self.frame_counter += 1

        if self.frame_counter % self.frame_skip != 0:
            return
        if (now - self.last_processed) < self.min_interval:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            processed, has_detection = self.process_frame(cv_image)
            
            with self.frame_lock:
                self.latest_frame = processed
                
            track_msg = Tracking()
            track_msg.detected = False
            track_msg.center_x = 0.0
            track_msg.center_y = 0.0
            track_msg.bounding_area = 0.0

            track_msg.detected = has_detection
            if track_msg.detected:
                track_msg.center_x = self.center_x
                track_msg.center_y = self.center_y
                track_msg.bounding_area = self.bounding_area

            self.publisher.publish(track_msg)
                
            self.last_processed = now
            
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")

    def process_frame(self, frame):
        # Detection variable for publisher
        has_detection = False

        # Frame size for publisher
        frame_h, frame_w = frame.shape[:2]

        # 1. Preprocess with aspect ratio preservation
        img, (scale, pad_left, pad_top) = self._preprocess_frame(frame)
        img = img.transpose(2, 0, 1)[np.newaxis].astype(np.float32) / 255.0
        
        # 2. Run inference
        outputs = self.sess.run(None, {"images": img})[0]
        predictions = np.squeeze(outputs).T
        scores = np.max(predictions[:, 4:], axis=1)
        class_ids = np.argmax(predictions[:, 4:], axis=1)
        boxes = predictions[:, :4]

        # 3. Apply NMS
        valid_indices = [i for i in range(len(scores)) 
                        if class_ids[i] == 0 and scores[i] > CONFIDENCE_THRESHOLD]
        if valid_indices:
            has_detection = True
            boxes_filtered = boxes[valid_indices]
            scores_filtered = scores[valid_indices]
            keep_indices = self._apply_nms(boxes_filtered, scores_filtered, IOU_THRESHOLD)

            for i in keep_indices:
                cx, cy, w, h = boxes_filtered[i]
                # Convert coordinates back to original frame
                cx = (cx - pad_left) / scale
                cy = (cy - pad_top) / scale
                w /= scale
                h /= scale
                
                # Set class variables for publisher
                self.center_x = cx / frame_w
                self.center_y = cy / frame_h
                self.bounding_area = w * h / (frame_w * frame_h)

                # Extract corner coordinates
                x1 = int(cx - w/2)
                y1 = int(cy - h/2)
                x2 = int(cx + w/2)
                y2 = int(cy + h/2)
                
                # Clip to frame dimensions
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"Person: {scores_filtered[i]:.2f}", 
                            (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return frame, has_detection