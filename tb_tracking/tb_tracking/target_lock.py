"""Verrouillage d'une cible parmi plusieurs pistes (EF-05) et re-acquisition. Aucune dependance ROS.

Regles :
- mode automatique : sans cible, on verrouille la piste la plus proche du centre de l'image ;
- mode manuel (auto_select=False) : on attend que l'utilisateur choisisse un ID avec select() ;
- ensuite on suit CE numero de piste, meme si une autre personne passe plus pres du centre ;
- piste absente : etat LOST tant que l'absence dure moins de lost_timeout, puis SEARCHING ;
  en mode automatique le verrou est alors relache et la prochaine piste vue est verrouillee
  (re-acquisition) ; en mode manuel on garde l'ID choisi jusqu'a un nouveau choix ;
- re-identification par la couleur des vetements : quand une autre personne passe devant la cible,
  le tracker perd souvent la piste et la cible reapparait avec un NOUVEL ID, ou l'autre personne
  reprend l'ID de la cible. On memorise donc la couleur des vetements de la cible :
  * l'ID suivi change de couleur -> ce n'est plus la cible (ID repris), on la considere perdue ;
  * la cible est perdue et une piste jamais vue en meme temps qu'elle a la meme couleur -> c'est elle,
    on se verrouille sur ce nouvel ID. Exception : une boite qui chevauche fortement celle de la cible
    peut etre un doublon de la meme personne (tres proche de la camera, YOLO detecte parfois le corps
    et les jambes a part) ; elle n'est donc pas classee comme « autre personne ».
"""

from dataclasses import dataclass

IDLE, LOCKED, LOST, SEARCHING = 0, 1, 2, 3  # memes valeurs que tb_interfaces/TargetState


@dataclass
class TrackObs:
    id: int
    cx: float  # centre de la bbox, pixels
    cy: float
    w: float
    h: float
    color: str = ''  # couleur des vetements estimee par le detecteur ('' = inconnue)


def overlap(a, b):
    """Part de la plus petite des deux boites couverte par l'autre (0 = disjointes, 1 = l'une dans l'autre)."""
    dx = min(a.cx + a.w / 2, b.cx + b.w / 2) - max(a.cx - a.w / 2, b.cx - b.w / 2)
    dy = min(a.cy + a.h / 2, b.cy + b.h / 2) - max(a.cy - a.h / 2, b.cy - b.h / 2)
    if dx <= 0 or dy <= 0:
        return 0.0
    return dx * dy / max(1e-9, min(a.w * a.h, b.w * b.h))


class TargetLock:

    def __init__(self, lost_timeout=2.0, auto_select=True, duplicate_overlap=0.6):
        self.lost_timeout = lost_timeout
        self.auto_select = auto_select
        self.locked_id = None
        self.last_seen = None    # temps (s) de la derniere observation de la cible
        self.last_track = None
        self.lock_count = 0      # nombre de verrouillages (1 = jamais perdu)
        self.target_color = ''   # couleur des vetements de la cible (sa signature d'apparence)
        self.others = set()      # IDs vus en meme temps que la cible : d'autres personnes, jamais elle
        self.reid_count = 0      # nombre de re-identifications par la couleur
        self.duplicate_overlap = duplicate_overlap  # au-dela : boite peut-etre doublon de la cible

    def remember_others(self, tracks, target):
        """Note les IDs vus en meme temps que la cible, sauf les boites qui sont peut-etre elle-meme."""
        self.others |= {tr.id for tr in tracks
                        if tr.id != target.id and overlap(tr, target) < self.duplicate_overlap}

    def select(self, track_id, t):
        """Choix de l'utilisateur : suivre track_id (ou plus rien si track_id < 0)."""
        self.target_color, self.others = '', set()
        if track_id < 0:
            self.locked_id, self.last_seen, self.last_track = None, None, None
            return
        self.locked_id, self.last_seen, self.last_track = track_id, t, None
        self.lock_count += 1

    def reidentify(self, tracks):
        """Piste qui peut etre la cible revenue sous un nouvel ID : meme couleur, jamais vue avec elle."""
        if not self.target_color:
            return None
        candidates = [tr for tr in tracks if tr.id != self.locked_id and tr.id not in self.others
                      and tr.color == self.target_color]
        if not candidates:
            return None
        last_cx = self.last_track.cx if self.last_track is not None else 0.0
        return min(candidates, key=lambda tr: abs(tr.cx - last_cx))  # la plus proche de sa derniere position

    def update(self, tracks, t, image_cx):
        """Une image de pistes au temps t (s). Retourne (etat, piste suivie ou derniere connue, temps sans la voir)."""
        if self.locked_id is not None:
            match = next((tr for tr in tracks if tr.id == self.locked_id), None)
            if match is not None and self.target_color and match.color and match.color != self.target_color:
                match = None  # meme ID, autres vetements : une autre personne a repris l'ID de la cible
            if match is None:
                match = self.reidentify(tracks)
                if match is not None:
                    self.locked_id = match.id
                    self.lock_count += 1
                    self.reid_count += 1
            if match is not None:
                self.last_seen, self.last_track = t, match
                if not self.target_color:
                    self.target_color = match.color
                self.remember_others(tracks, match)
                return LOCKED, match, 0.0
            elapsed = t - self.last_seen
            if elapsed < self.lost_timeout:
                return LOST, self.last_track, elapsed
            if not self.auto_select:
                return SEARCHING, self.last_track, elapsed  # mode manuel : on garde l'ID choisi
            self.locked_id = None  # perdue trop longtemps : on relache le verrou

        if tracks and self.auto_select:
            best = min(tracks, key=lambda tr: abs(tr.cx - image_cx))
            self.locked_id, self.last_seen, self.last_track = best.id, t, best
            self.target_color, self.others = best.color, set()
            self.remember_others(tracks, best)
            self.lock_count += 1
            return LOCKED, best, 0.0
        if self.last_seen is None:
            return IDLE, None, 0.0
        return SEARCHING, self.last_track, t - self.last_seen
