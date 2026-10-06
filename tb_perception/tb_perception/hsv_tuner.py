"""hsv_tuner : regler les seuils HSV du detecteur couleur sur l'image en direct.

Fenetre OpenCV : image avec les detections (a gauche) et masque (a droite), curseurs H/S/V min-max.
  s : afficher la ligne hsv_ranges a copier dans config/sim.yaml ou config/real.yaml
  q ou Echap : quitter

Exemples :
  ros2 run tb_perception hsv_tuner                                   # simulation
  ros2 run tb_perception hsv_tuner --ros-args -p image_topic:=/image_raw -p image_transport:=compressed
"""

from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from tb_perception.color_detector import ColorDetector

WINDOW = 'Reglage HSV  (s = afficher les seuils, q = quitter)'
SLIDERS = [('H min', 180), ('H max', 180), ('S min', 255), ('S max', 255), ('V min', 255), ('V max', 255),
           ('Aire min', 2000)]


class HsvTuner(Node):

    def __init__(self):
        super().__init__('hsv_tuner')
        image_topic = self.declare_parameter('image_topic', '/camera/image_raw').value
        transport = self.declare_parameter('image_transport', 'raw').value
        start = list(self.declare_parameter('hsv_ranges', [25, 80, 100, 45, 255, 255]).value)[:6]
        start_area = int(self.declare_parameter('min_area', 40.0).value)
        self.scale = self.declare_parameter('display_scale', 0.75).value

        self.bridge = CvBridge()
        self.image = None
        if transport == 'compressed':
            self.create_subscription(CompressedImage, image_topic + '/compressed', self.on_compressed,
                                     qos_profile_sensor_data)
        else:
            self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)

        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        initial = [start[0], start[3], start[1], start[4], start[2], start[5], start_area]
        for (name, maximum), value in zip(SLIDERS, initial):
            cv2.createTrackbar(name, WINDOW, int(value), maximum, lambda _v: None)
        self.get_logger().info(f'Ecoute {image_topic} ({transport})')

    def on_image(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def on_compressed(self, msg):
        self.image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)

    def values(self):
        h0, h1, s0, s1, v0, v1, area = (cv2.getTrackbarPos(name, WINDOW) for name, _ in SLIDERS)
        return (h0, s0, v0), (h1, s1, v1), area

    def render(self):
        lower, upper, area = self.values()
        detector = ColorDetector([(lower, upper)], min_area=float(area), max_detections=5)
        view = self.image.copy()
        detections = detector.detect(view)
        for det in detections:
            p1 = (int(det.cx - det.w / 2), int(det.cy - det.h / 2))
            p2 = (int(det.cx + det.w / 2), int(det.cy + det.h / 2))
            cv2.rectangle(view, p1, p2, (0, 0, 255), 2)
            cv2.putText(view, f'{det.score:.2f} h={det.h:.0f}px', (p1[0], max(p1[1] - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(view, f'{len(detections)} detection(s)', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 255), 2)
        mask = cv2.cvtColor(detector.mask(self.image), cv2.COLOR_GRAY2BGR)
        both = np.hstack([view, mask])
        if self.scale != 1.0:
            both = cv2.resize(both, None, fx=self.scale, fy=self.scale)
        cv2.imshow(WINDOW, both)

    def print_ranges(self):
        lower, upper, area = self.values()
        print('\nA copier dans la section detector_node de config/sim.yaml ou config/real.yaml :')
        print(f'    hsv_ranges: [{lower[0]}, {lower[1]}, {lower[2]}, {upper[0]}, {upper[1]}, {upper[2]}]')
        print(f'    min_area: {float(area)}\n', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = HsvTuner()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if node.image is not None:
                node.render()
            key = cv2.waitKey(20) & 0xFF
            if key == ord('s'):
                node.print_ranges()
            elif key in (ord('q'), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.print_ranges()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()
