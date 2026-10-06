"""target_chooser : fenetre pour choisir la personne a suivre.

Affiche l'image de la camera avec chaque personne detectee (ID et couleur des vetements).
Cliquer sur une personne, ou sur son bouton, publie son ID sur /target/select ; « Arreter » publie -1.
Le robot ne suit personne tant qu'aucun ID n'est choisi (target_selector_node en selection: manual).
"""

import tkinter as tk

from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import Image as PilImage, ImageTk
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32

from tb_interfaces.msg import TargetState, TrackArray

LATEST = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
# Couleur d'affichage (RGB) selon la couleur des vetements estimee par le detecteur
DISPLAY = {'rouge': '#e53935', 'orange': '#fb8c00', 'jaune': '#fdd835', 'vert': '#43a047', 'bleu': '#1e88e5',
           'violet': '#8e24aa', 'blanc': '#eeeeee', 'gris': '#9e9e9e', 'noir': '#212121', '': '#00e5ff'}
STATES = {TargetState.STATE_IDLE: 'aucune cible choisie', TargetState.STATE_LOCKED: 'suivie',
          TargetState.STATE_LOST: 'perdue', TargetState.STATE_SEARCHING: 'introuvable (recherche)'}


class TargetChooser(Node):

    def __init__(self):
        super().__init__('target_chooser')
        image_topic = self.declare_parameter('image_topic', '/camera/image_raw').value
        transport = self.declare_parameter('image_transport', 'raw').value
        self.scale = self.declare_parameter('display_scale', 1.0).value
        self.select_pub = self.create_publisher(Int32, 'target/select', 10)
        self.bridge = CvBridge()
        self.image = None
        self.tracks = []
        self.target = None
        if transport == 'compressed':
            self.create_subscription(CompressedImage, image_topic + '/compressed', self.on_compressed, LATEST)
        else:
            self.create_subscription(Image, image_topic, self.on_image, LATEST)
        self.create_subscription(TrackArray, 'tracks', lambda m: setattr(self, 'tracks', list(m.tracks)), 10)
        self.create_subscription(TargetState, 'target', lambda m: setattr(self, 'target', m), 10)

        self.root = tk.Tk()
        self.root.title('Choisir la personne a suivre')
        left = tk.Frame(self.root)
        left.pack(side='left', padx=8, pady=8)
        self.canvas = tk.Canvas(left, width=int(640 * self.scale), height=int(480 * self.scale), bg='black',
                                highlightthickness=0, cursor='hand2')
        self.canvas.pack()
        self.canvas.bind('<Button-1>', self.on_click)
        tk.Label(left, text='Cliquer sur une personne pour la suivre').pack(pady=(4, 0))

        right = tk.Frame(self.root)
        right.pack(side='left', fill='y', padx=8, pady=8)
        tk.Label(right, text='Personnes detectees', font=('TkDefaultFont', 11, 'bold')).pack(anchor='w')
        self.buttons = tk.Frame(right)
        self.buttons.pack(fill='x', pady=6)
        self.status = tk.Label(right, text='', justify='left', wraplength=220)
        self.status.pack(anchor='w', pady=8)
        tk.Button(right, text='Arreter le suivi', command=lambda: self.select(-1), bg='#ffcdd2').pack(fill='x')
        self.photo = None
        self.button_ids = []
        self.root.protocol('WM_DELETE_WINDOW', self.root.destroy)
        self.root.after(50, self.tick)

    def on_image(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def on_compressed(self, msg):
        bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is not None:
            self.image = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def select(self, track_id):
        self.select_pub.publish(Int32(data=int(track_id)))

    def on_click(self, event):
        x, y = event.x / self.scale, event.y / self.scale
        for tr in self.tracks:
            b = tr.bbox
            if abs(x - b.center.position.x) <= b.size_x / 2 and abs(y - b.center.position.y) <= b.size_y / 2:
                self.select(tr.id)
                return

    def target_id(self):
        if self.target is None or self.target.state == TargetState.STATE_IDLE:
            return None
        return self.target.target_id

    def refresh_buttons(self):
        ids = [(tr.id, tr.color) for tr in sorted(self.tracks, key=lambda t: t.id)]
        if ids == self.button_ids:
            return
        self.button_ids = ids
        for child in self.buttons.winfo_children():
            child.destroy()
        if not ids:
            tk.Label(self.buttons, text='(personne)').pack(anchor='w')
        for track_id, color in ids:
            text_color = 'white' if color in ('rouge', 'violet', 'bleu', 'noir', 'vert') else 'black'
            tk.Button(self.buttons, text=f'ID {track_id}  -  {color or "?"}', anchor='w',
                      bg=DISPLAY.get(color, '#00e5ff'), fg=text_color,
                      command=lambda i=track_id: self.select(i)).pack(fill='x', pady=2)

    def draw(self):
        if self.image is None:
            return
        rgb = self.image
        if self.scale != 1.0:
            rgb = cv2.resize(rgb, None, fx=self.scale, fy=self.scale)
        self.photo = ImageTk.PhotoImage(PilImage.fromarray(rgb))
        self.canvas.delete('all')
        self.canvas.create_image(0, 0, anchor='nw', image=self.photo)
        chosen = self.target_id()
        for tr in self.tracks:
            b, s = tr.bbox, self.scale
            x1, y1 = (b.center.position.x - b.size_x / 2) * s, (b.center.position.y - b.size_y / 2) * s
            x2, y2 = (b.center.position.x + b.size_x / 2) * s, (b.center.position.y + b.size_y / 2) * s
            is_target = tr.id == chosen
            color = DISPLAY.get(tr.color, '#00e5ff')
            self.canvas.create_rectangle(x1, y1, x2, y2, outline=color, width=5 if is_target else 2)
            label = f'ID {tr.id} {tr.color}' + ('  << CIBLE' if is_target else '')
            self.canvas.create_rectangle(x1, max(y1, 0), x1 + 7 * len(label) + 8, max(y1, 0) + 18, fill=color,
                                         outline='')
            self.canvas.create_text(x1 + 4, max(y1, 0) + 9, anchor='w', text=label, fill='black')

    def update_status(self):
        if self.target is None:
            self.status.config(text='En attente de target_selector_node...')
            return
        t = self.target
        if t.state == TargetState.STATE_IDLE:
            text = 'Aucune cible : choisir une personne'
        else:
            text = f'Cible : ID {t.target_id} - {STATES.get(t.state, "?")}'
            if t.state in (TargetState.STATE_LOST, TargetState.STATE_SEARCHING):
                text += f' depuis {t.time_since_seen:.1f} s'
            elif t.distance == t.distance:  # pas NaN
                text += f'\ndistance {t.distance:.2f} m'
        self.status.config(text=text)

    def tick(self):
        if not rclpy.ok():
            self.root.destroy()
            return
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.0)
        self.refresh_buttons()
        self.draw()
        self.update_status()
        self.root.after(100, self.tick)


def main(args=None):
    rclpy.init(args=args)
    node = TargetChooser()
    try:
        node.root.mainloop()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
