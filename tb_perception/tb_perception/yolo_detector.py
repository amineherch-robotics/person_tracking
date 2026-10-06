"""Detection et suivi de personnes avec YOLO (Ultralytics). Aucune dependance ROS.

Meme interface que ColorDetector : detect(image_bgr) -> liste de Detection.
Avec tracker='bytetrack.yaml' (ou 'botsort.yaml'), model.track() associe les detections d'une image a l'autre :
chaque Detection recoit un track_id stable (suivi multi-objets).
"""

import os

from tb_perception.color_detector import Detection


class YoloDetector:
    """Detecteur YOLO pre-entraine (COCO). class_ids=[0] : uniquement les personnes.

    model : nom (ex. 'yolo26n.pt', telecharge au premier usage dans weights_dir) ou chemin d'un fichier .pt.
    """

    def __init__(self, model='yolo26n.pt', weights_dir='~/.cache/person_tracking', imgsz=640,
                 conf_threshold=0.4, class_ids=(0,), device='cpu', max_detections=10, num_threads=0,
                 tracker='', track_conf=0.1):
        import torch
        from ultralytics import YOLO  # import tardif : le backend couleur n'a pas besoin de torch

        if num_threads > 0:
            # Limiter les threads : avec Gazebo sur la meme machine, trop de threads ralentit tout (mesure :
            # 4 threads -> jusqu'a 300 ms/image, 3 threads -> 95 ms stable). PyTorch n'est pas seul :
            # OpenCV et OpenBLAS (NumPy, utilise par le tracker) lancent aussi un thread par coeur.
            # Les variables OMP/OPENBLAS_NUM_THREADS sont fixees au lancement (bringup.launch.py).
            import cv2
            torch.set_num_interop_threads(1)
            cv2.setNumThreads(1)
        self.torch = torch
        self.num_threads = num_threads

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
        # Suivi : le tracker a besoin aussi des detections peu sures (ByteTrack les utilise en 2e passe),
        # on abaisse donc le seuil de detection ; c'est le tracker qui decide quelles pistes publier.
        self.tracker = tracker
        self.track_conf = track_conf

    def detect(self, bgr):
        # Ultralytics remet torch a min(8, coeurs - 1) threads lors de sa premiere prediction
        # (select_device) : on reimpose notre limite a chaque appel (verification tres peu couteuse).
        if self.num_threads > 0 and self.torch.get_num_threads() != self.num_threads:
            self.torch.set_num_threads(self.num_threads)
        options = dict(imgsz=self.imgsz, classes=self.class_ids, device=self.device, max_det=self.max_detections,
                       verbose=False)
        if self.tracker:
            # persist=True : le tracker garde ses pistes d'un appel a l'autre
            result = self.model.track(bgr, persist=True, tracker=self.tracker, conf=self.track_conf, **options)[0]
            ids = result.boxes.id.int().tolist() if result.boxes.id is not None else []
        else:
            result = self.model.predict(bgr, conf=self.conf_threshold, **options)[0]
            ids = [-1] * len(result.boxes)
        detections = []
        for (x1, y1, x2, y2), score, track_id in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist(), ids):
            detections.append(Detection((x1 + x2) / 2.0, (y1 + y2) / 2.0, x2 - x1, y2 - y1, float(score),
                                        int(track_id)))
        detections.sort(key=lambda d: d.score, reverse=True)
        return detections
