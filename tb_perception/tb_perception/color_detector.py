"""Detection d'objets par seuillage de couleur HSV (EF-12). Aucune dependance ROS."""

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass
class Detection:
    cx: float      # centre de la bbox, pixels
    cy: float
    w: float       # taille de la bbox, pixels
    h: float
    score: float   # 0..1
    track_id: int = -1  # identifiant de suivi (mode tracking YOLO), -1 sinon


class ColorDetector:
    """Seuillage HSV puis contours. Un objet = une composante connexe assez grande.

    hsv_ranges : liste de (lower, upper) en HSV OpenCV (H 0-180, S et V 0-255).
    Le rouge est a cheval sur H=0 : il faut deux intervalles.
    """

    def __init__(self, hsv_ranges, min_area=150.0, max_detections=1, open_kernel=5):
        self.ranges = [(np.array(lo, np.uint8), np.array(hi, np.uint8)) for lo, hi in hsv_ranges]
        self.min_area = min_area
        self.max_detections = max_detections
        self.kernel = np.ones((open_kernel, open_kernel), np.uint8) if open_kernel > 1 else None

    def mask(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in self.ranges:
            mask |= cv2.inRange(hsv, lo, hi)
        if self.kernel is not None:
            # Ouverture : supprime les pixels isoles dus au bruit
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        return mask

    def detect(self, bgr):
        contours, _ = cv2.findContours(self.mask(bgr), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            # Score : remplissage de la bbox, normalise pour qu'un disque plein vaille 1
            fill = area / float(w * h)
            score = min(1.0, fill / (math.pi / 4.0))
            detections.append((area, Detection(x + w / 2.0, y + h / 2.0, float(w), float(h), score)))
        detections.sort(key=lambda item: item[0], reverse=True)
        return [det for _, det in detections[:self.max_detections]]
