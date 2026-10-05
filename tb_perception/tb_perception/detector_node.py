"""detector_node : image -> /detections (vision_msgs/Detection2DArray).

Backends : 'color' (HSV, Phase 1). Le backend 'yolo' arrive en Phase 2.
Le header de l'image d'origine est recopie tel quel (ENF-01).
"""

from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from tb_perception.color_detector import ColorDetector


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
        else:
            raise ValueError(f"backend inconnu : '{backend}' (backends disponibles : color)")

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Detection2DArray, 'detections', 10)
        self.debug_pub = self.create_publisher(Image, 'detector/debug_image', 1)

        if transport == 'compressed':
            self.create_subscription(
                CompressedImage, image_topic + '/compressed', self.on_compressed, qos_profile_sensor_data)
            self.get_logger().info(f'Ecoute {image_topic}/compressed, backend {backend}')
        else:
            self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
            self.get_logger().info(f'Ecoute {image_topic}, backend {backend}')

    def on_image(self, msg):
        self.process(self.bridge.imgmsg_to_cv2(msg, 'bgr8'), msg.header)

    def on_compressed(self, msg):
        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if image is not None:
            self.process(image, msg.header)

    def process(self, image, header):
        detections = self.detector.detect(image)

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
