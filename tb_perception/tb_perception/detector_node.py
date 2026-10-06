"""detector_node : image -> /detections (vision_msgs/Detection2DArray), et /tracks en mode suivi.

Backends : 'color' (HSV, Phase 1) et 'yolo' (personnes, Phase 2).
Avec backend yolo et tracker: bytetrack.yaml, Ultralytics fait detection + suivi multi-objets en un appel :
chaque personne garde un identifiant stable, publie dans Detection2D.id et sur /tracks (TrackArray).
Le header de l'image d'origine est recopie tel quel (ENF-01).
"""

from collections import deque
import os
import time

from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from tb_interfaces.msg import Track, TrackArray
from tb_perception.clothing_color import clothing_color
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
                tracker=self.tracker_path(),
                track_conf=self.declare_parameter('track_conf', 0.1).value,
            )
        else:
            raise ValueError(f"backend inconnu : '{backend}' (backends disponibles : color, yolo)")
        self.inference_ms = []
        self.create_timer(5.0, self.report_speed)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Detection2DArray, 'detections', 10)
        self.debug_pub = self.create_publisher(Image, 'detector/debug_image', 1)
        self.tracking = backend == 'yolo' and bool(self.detector.tracker)
        if self.tracking:
            self.tracks_pub = self.create_publisher(TrackArray, 'tracks', 10)
            self.track_memory = {}  # id -> (cx, cy, temps, age) pour la vitesse et l'age des pistes
            self.track_colors = {}  # id -> dernieres couleurs de vetements estimees (vote majoritaire)
            self.get_logger().info(f'Suivi multi-objets : {self.detector.tracker}')

        if transport == 'compressed':
            self.create_subscription(
                CompressedImage, image_topic + '/compressed', self.on_compressed, LATEST_IMAGE_QOS)
            self.get_logger().info(f'Ecoute {image_topic}/compressed, backend {backend}')
        else:
            self.create_subscription(Image, image_topic, self.on_image, LATEST_IMAGE_QOS)
            self.get_logger().info(f'Ecoute {image_topic}, backend {backend}')

    def tracker_path(self):
        """Fichier de config du tracker : cherche d'abord dans tracker_dir (configs du projet, passe par le
        launch), sinon nom d'une config fournie par Ultralytics (bytetrack.yaml, botsort.yaml)."""
        tracker = self.declare_parameter('tracker', '').value
        tracker_dir = self.declare_parameter('tracker_dir', '').value
        if tracker and tracker_dir and os.path.isfile(os.path.join(tracker_dir, tracker)):
            return os.path.join(tracker_dir, tracker)
        return tracker

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
            if det.track_id >= 0:
                d.id = str(det.track_id)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.class_id
            hyp.hypothesis.score = det.score
            d.results.append(hyp)
            out.detections.append(d)
        self.pub.publish(out)
        if self.tracking:
            self.publish_tracks(detections, header, image)

        if self.publish_debug and self.debug_pub.get_subscription_count() > 0:
            for det in detections:
                p1 = (int(det.cx - det.w / 2), int(det.cy - det.h / 2))
                p2 = (int(det.cx + det.w / 2), int(det.cy + det.h / 2))
                cv2.rectangle(image, p1, p2, (0, 255, 0), 2)
                label = f'ID {det.track_id} ' if det.track_id >= 0 else ''
                cv2.putText(image, f'{label}{self.class_id} {det.score:.2f}', (p1[0], max(p1[1] - 6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            debug = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            debug.header = header
            self.debug_pub.publish(debug)

    def publish_tracks(self, detections, header, image):
        now = Time.from_msg(header.stamp).nanoseconds * 1e-9
        msg = TrackArray()
        msg.header = header
        seen = set()
        for det in detections:
            if det.track_id < 0:
                continue
            seen.add(det.track_id)
            track = Track()
            track.id = det.track_id
            track.bbox.center.position.x = det.cx
            track.bbox.center.position.y = det.cy
            track.bbox.size_x = det.w
            track.bbox.size_y = det.h
            track.score = det.score
            previous = self.track_memory.get(det.track_id)
            age = 1
            if previous is not None:
                px, py, pt, page = previous
                age = page + 1
                if now > pt:
                    track.vx = float((det.cx - px) / (now - pt))
                    track.vy = float((det.cy - py) / (now - pt))
            self.track_memory[det.track_id] = (det.cx, det.cy, now, age)
            track.age = age
            track.frames_since_update = 0
            track.confirmed = True  # Ultralytics ne publie que les pistes confirmees
            # Couleur des vetements, stabilisee par un vote sur les 15 dernieres images de la piste.
            # Pas de vote si la bbox touche (a 5 % pres) le haut de l'image : le haut du corps est coupe
            # (de pres, la camera basse ne voit que les jambes, on lirait la couleur du jean).
            # Repli tant qu'aucune vue complete n'existe : le haut de la partie visible (0-25 %), qui est la
            # poitrine quand la personne est coupee au cou.
            votes = self.track_colors.setdefault(det.track_id, (deque(maxlen=15), deque(maxlen=15)))
            if det.cy - det.h / 2.0 > 0.05 * image.shape[0]:
                vote_list, band = votes[0], (0.15, 0.45)
            else:
                vote_list, band = votes[1], (0.0, 0.25)
            color = clothing_color(image, det.cx, det.cy, det.w, det.h, band=band)
            if color:
                vote_list.append(color)
            best = votes[0] or votes[1]
            track.color = max(set(best), key=best.count) if best else ''
            msg.tracks.append(track)
        # Oublier les pistes absentes depuis plus de 10 s
        self.track_memory = {k: v for k, v in self.track_memory.items() if k in seen or now - v[2] < 10.0}
        self.track_colors = {k: v for k, v in self.track_colors.items() if k in self.track_memory}
        self.tracks_pub.publish(msg)

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
