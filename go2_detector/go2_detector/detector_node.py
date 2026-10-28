"""2D object detector for the Go2's camera.

Backends (run in the order given by the ``backends`` parameter, results are combined):
  * ``color``: HSV colour segmentation. No model needed; finds the coloured boxes and
    cylinders in the Gazebo world. Classes: red, yellow, green, cyan, blue.
  * ``yolo``: Ultralytics YOLO (COCO classes, e.g. person). Loaded lazily; if Ultralytics
    is missing, the node logs a warning and keeps running with the other backends.

Publishes vision_msgs/Detection2DArray on /perception/detections_2d, stamped with the
image's header so the fusion node can check that LiDAR and camera describe the same moment.
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# OpenCV hue runs 0-179. Red wraps around 0, so it needs two ranges.
# Saturation >= 120 keeps the grey floor, sky and the robot itself out.
COLOR_RANGES = {
    'red': [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (179, 255, 255))],
    'yellow': [((20, 120, 70), (35, 255, 255))],
    'green': [((40, 120, 70), (85, 255, 255))],
    'cyan': [((86, 120, 70), (99, 255, 255))],
    'blue': [((100, 120, 70), (130, 255, 255))],
}


def image_to_bgr(msg: Image) -> np.ndarray:
    """Convert a sensor_msgs/Image to a BGR array without cv_bridge.

    Skipping cv_bridge avoids NumPy ABI clashes between apt and pip packages.
    """
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'unsupported encoding {msg.encoding}')
    # Rows may be padded (step >= width * 3), so slice each row to the real width.
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = rows[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img.copy()


class ColorDetector:
    name = 'color'

    def __init__(self, min_area: int):
        self.min_area = min_area
        self.kernel = np.ones((5, 5), np.uint8)

    def detect(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        out = []
        for label, ranges in COLOR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)  # remove speckles
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < self.min_area:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                # How much of the box the blob fills: a crude but honest confidence.
                score = float(np.clip(area / float(w * h), 0.5, 1.0))
                out.append((x, y, x + w, y + h, label, score))
        return out


class YoloDetector:
    name = 'yolo'

    def __init__(self, model: str, conf: float, device: str, classes: list):
        from ultralytics import YOLO  # imported here so the colour backend works without it

        self.model = YOLO(model)
        self.conf = conf
        self.device = device
        # Keep only the classes of interest (by name); empty list = all COCO classes.
        name_to_id = {name: i for i, name in self.model.names.items()}
        unknown = [c for c in classes if c not in name_to_id]
        if unknown:
            raise ValueError(f'unknown YOLO classes {unknown}')
        self.class_ids = [name_to_id[c] for c in classes] or None

    def detect(self, bgr):
        result = self.model.predict(
            bgr, conf=self.conf, device=self.device, classes=self.class_ids, verbose=False)[0]
        out = []
        for box in result.boxes:
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
            label = result.names[int(box.cls[0])]
            out.append((x1, y1, x2, y2, label, float(box.conf[0])))
        return out


class DetectorNode(Node):

    def __init__(self):
        super().__init__('detector_node')
        image_topic = self.declare_parameter('image_topic', '/d455/image').value
        output_topic = self.declare_parameter('output_topic', '/perception/detections_2d').value
        backends = self.declare_parameter('backends', ['color', 'yolo']).value
        self.min_period = 1.0 / float(self.declare_parameter('max_rate_hz', 5.0).value)
        min_area = int(self.declare_parameter('min_area', 150).value)
        yolo_model = self.declare_parameter('yolo_model', 'yolo11n.pt').value
        yolo_conf = float(self.declare_parameter('yolo_conf', 0.4).value)
        yolo_device = self.declare_parameter('yolo_device', 'cpu').value
        yolo_classes = list(self.declare_parameter('yolo_classes', ['person']).value)

        self.detectors = []
        for backend in backends:
            if backend == 'color':
                self.detectors.append(ColorDetector(min_area))
            elif backend == 'yolo':
                try:
                    self.detectors.append(YoloDetector(yolo_model, yolo_conf, yolo_device, yolo_classes))
                except Exception as e:  # missing package, failed download, ...
                    self.get_logger().warn(f'YOLO disabled ({e}); continuing without it')
            else:
                self.get_logger().warn(f'Unknown backend "{backend}" ignored')

        self.pub = self.create_publisher(Detection2DArray, output_topic, 10)
        self.sub = self.create_subscription(
            Image, image_topic, self.on_image, qos_profile_sensor_data)

        self.last_run = 0.0
        self.frames = 0
        self.total_ms = 0.0
        self.last_log = time.monotonic()
        names = ', '.join(d.name for d in self.detectors) or 'none'
        self.get_logger().info(f'Detecting on {image_topic} with backends: {names}')

    def on_image(self, msg: Image):
        now = time.monotonic()
        if now - self.last_run < self.min_period:
            return  # rate limit: the LiDAR only runs at a few Hz anyway
        self.last_run = now

        try:
            bgr = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return

        t0 = time.perf_counter()
        boxes = []
        for detector in self.detectors:
            boxes.extend(detector.detect(bgr))
        ms = (time.perf_counter() - t0) * 1000.0

        out = Detection2DArray()
        out.header = msg.header
        for i, (x1, y1, x2, y2, label, score) in enumerate(boxes):
            det = Detection2D()
            det.header = msg.header
            det.id = str(i)
            det.bbox.center.position.x = 0.5 * (x1 + x2)
            det.bbox.center.position.y = 0.5 * (y1 + y2)
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = label
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)
            out.detections.append(det)
        self.pub.publish(out)

        self.frames += 1
        self.total_ms += ms
        if now - self.last_log > 5.0:
            self.last_log = now
            labels = ', '.join(sorted({b[4] for b in boxes})) or '-'
            self.get_logger().info(
                f'{len(boxes)} detections [{labels}] | {ms:.0f} ms '
                f'(avg {self.total_ms / self.frames:.0f} ms)')


def main():
    rclpy.init()
    node = DetectorNode()
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
