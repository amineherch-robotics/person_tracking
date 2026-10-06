"""detector_node : image -> /detections (vision_msgs/Detection2DArray).

Backends : 'color' (HSV, Phase 1) et 'yolo' (personnes, Phase 2).
Le header de l'image d'origine est recopie tel quel (ENF-01).
"""

import time

from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from tb_perception.color_detector import ColorDetector

# File d'attente de 1 image : si le detecteur est plus lent que la camera, il traite toujours
# l'image la plus recente au lieu d'accumuler du retard (meme probleme que le buffer de la webcam).
LATEST_IMAGE_QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)


class DetectorNode(Node):

    def __init__(self):
        super().__init__('detector_node')
        image_topic = self.declare_parameter('image_topic', '/camera/image_raw').value
        transport = self.declare_parameter('image_transport', 'raw').value
        backend = self.declare_parameter('backend', 'color').value
        self.class_id = self.declare_parameter('class_id', 'tennis_ball').value
        self.publish_debug = self.declare_parameter('publish_debug', True).value

        if backend == 'color':
            # Intervalles HSV a plat : [h_min, s_min, v_min, h_max, s_max, v_max] repete N fois
            flat = list(self.declare_parameter('hsv_ranges', [25, 80, 100, 45, 255, 255]).value)
            if not flat or len(flat) % 6:
                raise ValueError('hsv_ranges doit contenir 6 valeurs par intervalle HSV')
            ranges = [(flat[i:i + 3], flat[i + 3:i + 6]) for i in range(0, len(flat), 6)]
            self.detector = ColorDetector(
                ranges,
                min_area=self.declare_parameter('min_area', 40.0).value,
                max_detections=self.declare_parameter('max_detections', 1).value,
            )
        elif backend == 'yolo':
            from tb_perception.yolo_detector import YoloDetector
            self.detector = YoloDetector(
                model=self.declare_parameter('model', 'yolo26n.pt').value,
                weights_dir=self.declare_parameter('weights_dir', '~/.cache/person_tracking').value,
                imgsz=self.declare_parameter('imgsz', 640).value,
                conf_threshold=self.declare_parameter('conf_threshold', 0.4).value,
                class_ids=self.declare_parameter('class_ids', [0]).value,
                device=self.declare_parameter('device', 'cpu').value,
                max_detections=self.declare_parameter('max_detections', 10).value,
                num_threads=self.declare_parameter('num_threads', 3).value,
            )
        else:
            raise ValueError(f"backend inconnu : '{backend}' (backends disponibles : color, yolo)")
        self.inference_ms = []
        self.create_timer(5.0, self.report_speed)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Detection2DArray, 'detections', 10)
        self.debug_pub = self.create_publisher(Image, 'detector/debug_image', 1)

        if transport == 'compressed':
            self.create_subscription(
                CompressedImage, image_topic + '/compressed', self.on_compressed, LATEST_IMAGE_QOS)
            self.get_logger().info(f'Ecoute {image_topic}/compressed, backend {backend}')
        else:
            self.create_subscription(Image, image_topic, self.on_image, LATEST_IMAGE_QOS)
            self.get_logger().info(f'Ecoute {image_topic}, backend {backend}')

    def on_image(self, msg):
        self.process(self.bridge.imgmsg_to_cv2(msg, 'bgr8'), msg.header)

    def on_compressed(self, msg):
        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if image is not None:
            self.process(image, msg.header)

    def process(self, image, header):
        start = time.perf_counter()
        detections = self.detector.detect(image)
        self.inference_ms.append((time.perf_counter() - start) * 1000.0)

        out = Detection2DArray()
        out.header = header
        for det in detections:
            d = Detection2D()
            d.header = header
            d.bbox.center.position.x = det.cx
            d.bbox.center.position.y = det.cy
            d.bbox.size_x = det.w
            d.bbox.size_y = det.h
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.class_id
            hyp.hypothesis.score = det.score
            d.results.append(hyp)
            out.detections.append(d)
        self.pub.publish(out)

        if self.publish_debug and self.debug_pub.get_subscription_count() > 0:
            for det in detections:
                p1 = (int(det.cx - det.w / 2), int(det.cy - det.h / 2))
                p2 = (int(det.cx + det.w / 2), int(det.cy + det.h / 2))
                cv2.rectangle(image, p1, p2, (0, 255, 0), 2)
                cv2.putText(image, f'{self.class_id} {det.score:.2f}', (p1[0], max(p1[1] - 6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            debug = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            debug.header = header
            self.debug_pub.publish(debug)

    def report_speed(self):
        if not self.inference_ms:
            return
        values = sorted(self.inference_ms)
        self.inference_ms = []
        median = values[len(values) // 2]
        self.get_logger().info(f'Detection : mediane {median:.0f} ms/image, {len(values) / 5.0:.1f} images/s traitees')


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
