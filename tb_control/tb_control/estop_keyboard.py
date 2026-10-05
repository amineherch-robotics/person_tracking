"""estop_keyboard : arret d'urgence au clavier (SEC-05).

  g       : activer le suivi
  espace  : arret d'urgence (desactive le suivi et envoie une commande nulle)
  q       : quitter (arret d'urgence avant de sortir)
"""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool


class EstopKeyboard(Node):

    def __init__(self):
        super().__init__('estop_keyboard')
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enable_pub = self.create_publisher(Bool, 'follower/enable', latched)
        self.cmd_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)

    def set_enabled(self, enabled):
        self.enable_pub.publish(Bool(data=enabled))
        if not enabled:
            # Commande nulle directe, au cas ou le controleur ne repondrait plus
            for _ in range(3):
                cmd = TwistStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()
                self.cmd_pub.publish(cmd)


def main(args=None):
    # Pas de handler SIGINT de rclpy : le contexte reste valide apres Ctrl+C,
    # pour pouvoir publier la commande d'arret avant de quitter.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = EstopKeyboard()
    settings = termios.tcgetattr(sys.stdin)
    print(__doc__)
    node.set_enabled(False)
    print('Etat : ARRET')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue
            key = sys.stdin.read(1)
            if key == 'g':
                node.set_enabled(True)
                print('Etat : SUIVI ACTIF')
            elif key == ' ':
                node.set_enabled(False)
                print('Etat : ARRET')
            elif key == 'q':
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.set_enabled(False)
        print('Etat : ARRET')
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.try_shutdown()
