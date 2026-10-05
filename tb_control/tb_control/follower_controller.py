"""follower_controller : /target -> /cmd_vel (TwistStamped), avec watchdog et limites de securite.

Securite : SEC-01 watchdog, SEC-02 saturation, SEC-03 limitation d'acceleration,
SEC-04 distance minimale, SEC-05 arret d'urgence via /follower/enable (voir estop_keyboard).
"""

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool

from tb_control.follower_law import FollowerParams, compute_command, rate_limit
from tb_interfaces.msg import TargetState


class FollowerController(Node):

    def __init__(self):
        super().__init__('follower_controller')
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        rate = self.declare_parameter('control_rate', 20.0).value
        self.watchdog_timeout = self.declare_parameter('watchdog_timeout', 0.3).value
        self.max_linear_accel = self.declare_parameter('max_linear_accel', 0.3).value
        self.max_angular_accel = self.declare_parameter('max_angular_accel', 2.0).value
        self.enabled = self.declare_parameter('start_enabled', False).value

        defaults = FollowerParams()
        self.params = FollowerParams(**{
            name: self.declare_parameter(name, getattr(defaults, name)).value
            for name in vars(defaults)
        })

        self.target = None
        self.target_rx_time = None
        self.v = 0.0
        self.w = 0.0
        self.dt = 1.0 / rate
        self.stopped_reason = None
        self.latencies = []

        self.pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self.create_subscription(TargetState, 'target', self.on_target, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'follower/enable', self.on_enable, latched)
        self.create_timer(self.dt, self.on_timer)
        self.create_timer(5.0, self.report_latency)

        self.get_logger().info(
            f"Suiveur {'ACTIF' if self.enabled else 'EN ATTENTE'} : "
            "lancer `ros2 run tb_control estop_keyboard` (g = go, espace = stop)")

    def on_enable(self, msg):
        if msg.data != self.enabled:
            self.get_logger().warn('Suiveur ACTIF' if msg.data else 'ARRET D\'URGENCE')
        self.enabled = msg.data

    def on_target(self, msg):
        now = self.get_clock().now()
        self.target = msg
        self.target_rx_time = now
        # Latence image -> controleur (ENF-01). En reel, suppose les horloges robot/laptop synchronisees.
        self.latencies.append((now - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9)

    def on_timer(self):
        now = self.get_clock().now()
        v_target, w_target = 0.0, 0.0

        if not self.enabled:
            reason = 'desactive'
        elif self.target_rx_time is None:
            reason = 'aucune cible recue'
        elif (now - self.target_rx_time).nanoseconds * 1e-9 > self.watchdog_timeout:
            reason = 'watchdog : plus de cible ni d\'image'
        elif self.target.state != TargetState.STATE_LOCKED:
            reason = 'cible perdue'
        else:
            reason = None
            v_target, w_target = compute_command(self.target.bearing, self.target.distance, self.params)

        if reason != self.stopped_reason:
            if reason is None:
                self.get_logger().info('Suivi de la cible')
            else:
                self.get_logger().warn(f'Arret : {reason}')
            self.stopped_reason = reason

        if reason is not None and reason.startswith('watchdog'):
            # SEC-01 : arret immediat, sans rampe de deceleration
            self.v, self.w = 0.0, 0.0
        else:
            self.v = rate_limit(self.v, v_target, self.max_linear_accel, self.dt)
            self.w = rate_limit(self.w, w_target, self.max_angular_accel, self.dt)

        cmd = TwistStamped()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = self.base_frame
        cmd.twist.linear.x = self.v
        cmd.twist.angular.z = self.w
        self.pub.publish(cmd)

    def report_latency(self):
        if not self.latencies:
            return
        values = sorted(self.latencies)
        self.latencies = []
        median = values[len(values) // 2] * 1000.0
        worst = values[-1] * 1000.0
        self.get_logger().info(f'Latence image -> controleur : mediane {median:.0f} ms, max {worst:.0f} ms')

    def stop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.base_frame
        self.pub.publish(cmd)


def main(args=None):
    # Pas de handler SIGINT de rclpy : le contexte reste valide apres Ctrl+C,
    # pour pouvoir publier la commande d'arret avant de quitter.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = FollowerController()
    try:
        # spin_once avec timeout (temps reel) : si /clock s'arrete (Gazebo ferme), les timers en temps
        # simule ne se declenchent plus et rclpy.spin() bloquerait sans jamais traiter Ctrl+C.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
