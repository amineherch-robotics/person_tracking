"""Detection de personnes avec YOLO (Ultralytics). Aucune dependance ROS.

Meme interface que ColorDetector : detect(image_bgr) -> liste de Detection.
"""

import os

from tb_perception.color_detector import Detection


class YoloDetector:
    """Detecteur YOLO pre-entraine (COCO). class_ids=[0] : uniquement les personnes.

    model : nom (ex. 'yolo26n.pt', telecharge au premier usage dans weights_dir) ou chemin d'un fichier .pt.
    """

    def __init__(self, model='yolo26n.pt', weights_dir='~/.cache/person_tracking', imgsz=640,
                 conf_threshold=0.4, class_ids=(0,), device='cpu', max_detections=10, num_threads=0):
        import torch
        from ultralytics import YOLO  # import tardif : le backend couleur n'a pas besoin de torch

        if num_threads > 0:
            # Limiter les threads : avec Gazebo sur la meme machine, trop de threads ralentit tout (mesure :
            # 4 threads -> jusqu'a 300 ms/image, 3 threads -> 95 ms stable)
            torch.set_num_threads(num_threads)

        path = os.path.expanduser(model)
        if not os.path.isfile(path):
            # Ultralytics telecharge les poids dans le dossier courant : on s'y place le temps du chargement
            weights_dir = os.path.expanduser(weights_dir)
            os.makedirs(weights_dir, exist_ok=True)
            path = os.path.join(weights_dir, os.path.basename(model))
            cwd = os.getcwd()
            os.chdir(weights_dir)
            try:
                self.model = YOLO(os.path.basename(model))
            finally:
                os.chdir(cwd)
        else:
            self.model = YOLO(path)
        self.imgsz = imgsz
        self.conf_threshold = conf_threshold
        self.class_ids = list(class_ids)
        self.device = device
        self.max_detections = max_detections
        self.names = self.model.names

    def detect(self, bgr):
        result = self.model.predict(bgr, imgsz=self.imgsz, conf=self.conf_threshold, classes=self.class_ids,
                                    device=self.device, max_det=self.max_detections, verbose=False)[0]
        detections = []
        for (x1, y1, x2, y2), score in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist()):
            detections.append(Detection((x1 + x2) / 2.0, (y1 + y2) / 2.0, x2 - x1, y2 - y1, float(score)))
        detections.sort(key=lambda d: d.score, reverse=True)
        return detections
