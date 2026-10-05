"""ball_joystick : joystick a l'ecran pour deplacer la balle de tennis dans Gazebo.

Glisser le bouton a la souris, ou utiliser les fleches du clavier (fenetre active).
Publie geometry_msgs/Twist sur /ball/cmd_vel (20 Hz), traduit vers Gazebo par ros_gz_bridge.
Axes du monde : haut = +x (devant le robot au depart), gauche = +y.
"""

import math
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

PAD_RADIUS = 110     # px, zone du joystick
KNOB_RADIUS = 24     # px, bouton
SIZE = 2 * PAD_RADIUS + 2 * KNOB_RADIUS + 20
KEYS = {'Up': (1, 0), 'Down': (-1, 0), 'Left': (0, 1), 'Right': (0, -1)}


class BallJoystick(Node):

    def __init__(self):
        super().__init__('ball_joystick')
        topic = self.declare_parameter('cmd_vel_topic', 'ball/cmd_vel').value
        self.max_speed = self.declare_parameter('max_speed', 0.2).value
        rate = self.declare_parameter('publish_rate', 20.0).value
        self.pub = self.create_publisher(Twist, topic, 10)
        self.period_ms = int(1000 / rate)

        # Consigne normalisee : (avant, gauche) dans [-1, 1]
        self.mouse = (0.0, 0.0)
        self.keys = set()

        self.root = tk.Tk()
        self.root.title('Joystick balle de tennis')
        self.root.resizable(False, False)
        tk.Label(self.root, text='Glisser le bouton ou fleches du clavier\nhaut = +x (devant le robot au depart), '
                 'gauche = +y').pack(padx=10, pady=(10, 4))
        self.canvas = tk.Canvas(self.root, width=SIZE, height=SIZE, bg='#f4f4f4', highlightthickness=0)
        self.canvas.pack(padx=10)
        c = SIZE / 2
        self.center = c
        self.canvas.create_oval(c - PAD_RADIUS, c - PAD_RADIUS, c + PAD_RADIUS, c + PAD_RADIUS,
                                outline='#888', width=2, fill='#e6e6e6')
        self.canvas.create_line(c, c - PAD_RADIUS, c, c + PAD_RADIUS, fill='#bbb', dash=(3, 3))
        self.canvas.create_line(c - PAD_RADIUS, c, c + PAD_RADIUS, c, fill='#bbb', dash=(3, 3))
        self.canvas.create_text(c, c - PAD_RADIUS - 10, text='+x', fill='#555')
        self.canvas.create_text(c - PAD_RADIUS - 12, c, text='+y', fill='#555')
        self.knob = self.canvas.create_oval(c - KNOB_RADIUS, c - KNOB_RADIUS, c + KNOB_RADIUS, c + KNOB_RADIUS,
                                            fill='#c6e000', outline='#6b7a00', width=2)

        speed_frame = tk.Frame(self.root)
        speed_frame.pack(fill='x', padx=10, pady=4)
        tk.Label(speed_frame, text='Vitesse max (m/s)').pack(side='left')
        self.speed = tk.DoubleVar(value=self.max_speed)
        tk.Scale(speed_frame, variable=self.speed, from_=0.05, to=0.5, resolution=0.05,
                 orient='horizontal', length=170).pack(side='right')
        self.status = tk.Label(self.root, text='', font=('TkFixedFont', 10))
        self.status.pack(pady=(0, 6))
        tk.Button(self.root, text='STOP balle', command=self.stop_all).pack(pady=(0, 10))

        self.canvas.bind('<B1-Motion>', self.on_drag)
        self.canvas.bind('<Button-1>', self.on_drag)
        self.canvas.bind('<ButtonRelease-1>', self.on_release)
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.get_logger().info(f'Joystick pret : publie sur {topic}')
        self.root.after(self.period_ms, self.tick)

    def on_drag(self, event):
        dx, dy = event.x - self.center, event.y - self.center
        norm = math.hypot(dx, dy)
        if norm > PAD_RADIUS:
            dx, dy = dx * PAD_RADIUS / norm, dy * PAD_RADIUS / norm
        # Ecran : y vers le bas. Monde : haut de l'ecran = +x, gauche = +y
        self.mouse = (-dy / PAD_RADIUS, -dx / PAD_RADIUS)

    def on_release(self, _event):
        self.mouse = (0.0, 0.0)

    def on_key_press(self, event):
        if event.keysym in KEYS:
            self.keys.add(event.keysym)

    def on_key_release(self, event):
        self.keys.discard(event.keysym)

    def stop_all(self):
        self.mouse = (0.0, 0.0)
        self.keys.clear()

    def command(self):
        if self.keys:
            fwd = sum(KEYS[k][0] for k in self.keys)
            left = sum(KEYS[k][1] for k in self.keys)
            norm = math.hypot(fwd, left)
            return (fwd / norm, left / norm) if norm else (0.0, 0.0)
        return self.mouse

    def tick(self):
        if not rclpy.ok():  # Ctrl+C dans le terminal : rclpy est deja arrete
            self.root.destroy()
            return
        fwd, left = self.command()
        speed = self.speed.get()
        msg = Twist()
        msg.linear.x = fwd * speed
        msg.linear.y = left * speed
        self.pub.publish(msg)

        c = self.center
        kx, ky = c - left * PAD_RADIUS, c - fwd * PAD_RADIUS
        self.canvas.coords(self.knob, kx - KNOB_RADIUS, ky - KNOB_RADIUS, kx + KNOB_RADIUS, ky + KNOB_RADIUS)
        self.status.config(text=f'vx = {msg.linear.x:+.2f} m/s   vy = {msg.linear.y:+.2f} m/s')
        self.root.after(self.period_ms, self.tick)

    def close(self):
        if rclpy.ok():
            for _ in range(3):
                self.pub.publish(Twist())
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = BallJoystick()
    try:
        node.root.mainloop()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
