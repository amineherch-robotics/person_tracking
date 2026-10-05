"""target_selector_node : /detections -> /target (tb_interfaces/TargetState).

Phase 1 : pas encore de tracker, la cible est la detection de meilleur score dans chaque image.
Phase 3 : l'entree deviendra /tracks et la cible sera verrouillee sur un id (EF-05).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray

from tb_interfaces.msg import TargetState
from tb_tracking.target_geometry import bearing, distance_from_height, intrinsics_from_hfov


class TargetSelectorNode(Node):

    def __init__(self):
        super().__init__('target_selector_node')
        camera_info_topic = self.declare_parameter('camera_info_topic', '/camera/camera_info').value
        self.object_height = self.declare_parameter('object_height', 0.067).value
        self.lost_timeout = self.declare_parameter('lost_timeout', 2.0).value
        # Repli si camera_info absent ou non calibre (K nul, cas frequent avec usb_cam)
        self.intrinsics = intrinsics_from_hfov(
            self.declare_parameter('image_width', 640).value,
            self.declare_parameter('image_height', 480).value,
            self.declare_parameter('camera_hfov', 1.2217).value,
        )
        self.has_camera_info = False

        self.last_seen = None
        self.last_target = None

        self.pub = self.create_publisher(TargetState, 'target', 10)
        self.create_subscription(Detection2DArray, 'detections', self.on_detections, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self.on_camera_info, qos_profile_sensor_data)

    def on_camera_info(self, msg):
        if self.has_camera_info or msg.k[0] <= 0.0:
            return
        self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.has_camera_info = True
        self.get_logger().info(f'Intrinseques camera : fx={msg.k[0]:.1f} cx={msg.k[2]:.1f}')

    def on_detections(self, msg):
        fx, fy, cx, _ = self.intrinsics
        stamp = Time.from_msg(msg.header.stamp)
        target = TargetState()
        target.header = msg.header
        target.distance = float('nan')

        if msg.detections:
            best = max(msg.detections, key=lambda d: (
                d.results[0].hypothesis.score if d.results else 0.0, d.bbox.size_x * d.bbox.size_y))
            target.state = TargetState.STATE_LOCKED
            target.bbox = best.bbox
            target.bearing = bearing(best.bbox.center.position.x, cx, fx)
            target.distance = distance_from_height(best.bbox.size_y, fy, self.object_height)
            target.distance_source = TargetState.DISTANCE_BBOX
            target.time_since_seen = 0.0
            self.last_seen = stamp
            self.last_target = target
        elif self.last_seen is None:
            target.state = TargetState.STATE_IDLE
        else:
            elapsed = (stamp - self.last_seen).nanoseconds * 1e-9
            target.state = (TargetState.STATE_LOST if elapsed < self.lost_timeout
                            else TargetState.STATE_SEARCHING)
            target.bbox = self.last_target.bbox
            target.bearing = self.last_target.bearing
            target.distance_source = TargetState.DISTANCE_NONE
            target.time_since_seen = float(elapsed)

        if math.isnan(target.distance):
            target.distance_source = TargetState.DISTANCE_NONE
        self.pub.publish(target)


def main(args=None):
    rclpy.init(args=args)
    node = TargetSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
