"""Couleur dominante des vetements d'une personne detectee (haut du corps). Aucune dependance ROS.

Sert a aider l'utilisateur a reconnaitre les personnes quand il choisit l'ID a suivre.
"""

import cv2
import numpy as np

# Plages de teinte OpenCV (H de 0 a 179) -> nom de couleur
HUE_NAMES = [(8, 'rouge'), (20, 'orange'), (35, 'jaune'), (85, 'vert'), (128, 'bleu'), (165, 'violet'), (180, 'rouge')]


def hue_name(h):
    for upper, name in HUE_NAMES:
        if h < upper:
            return name
    return 'rouge'


def clothing_color(bgr, cx, cy, w, h, min_saturation=70, min_value=50, band=(0.15, 0.45)):
    """Nom de la couleur dominante sur le haut du corps de la bbox (centre, largeur, hauteur en pixels).

    Zone analysee : la moitie centrale en largeur, et la bande verticale `band` (fractions de la hauteur depuis
    le haut de la bbox) : par defaut 15 % a 45 %, le tronc d'une personne entiere, en evitant la tete, les bras
    et les jambes. Peu de pixels colores : blanc, gris ou noir.
    """
    height, width = bgr.shape[:2]
    top = cy - h / 2.0
    x1, x2 = int(max(0, cx - 0.25 * w)), int(min(width, cx + 0.25 * w))
    y1, y2 = int(max(0, top + band[0] * h)), int(min(height, top + band[1] * h))
    if x2 - x1 < 2 or y2 - y1 < 2:
        return ''
    hsv = cv2.cvtColor(bgr[y1:y2, x1:x2], cv2.COLOR_BGR2HSV).reshape(-1, 3)
    colored = hsv[(hsv[:, 1] >= min_saturation) & (hsv[:, 2] >= min_value)]
    if len(colored) < 0.15 * len(hsv):
        value = float(np.median(hsv[:, 2]))
        return 'blanc' if value > 170 else ('noir' if value < 60 else 'gris')
    names = [hue_name(int(h_)) for h_ in colored[:, 0]]
    return max(set(names), key=names.count)
