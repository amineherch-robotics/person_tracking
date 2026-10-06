"""Verrouillage d'une cible parmi plusieurs pistes (EF-05) et re-acquisition. Aucune dependance ROS.

Regles :
- sans cible, on verrouille la piste la plus proche du centre de l'image ;
- ensuite on suit CE numero de piste, meme si une autre personne passe plus pres du centre ;
- piste absente : etat LOST tant que l'absence dure moins de lost_timeout, puis SEARCHING ;
  le verrou est alors relache et la prochaine piste vue est verrouillee (re-acquisition).
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


class TargetLock:

    def __init__(self, lost_timeout=2.0):
        self.lost_timeout = lost_timeout
        self.locked_id = None
        self.last_seen = None    # temps (s) de la derniere observation de la cible
        self.last_track = None
        self.lock_count = 0      # nombre de verrouillages (1 = jamais perdu)

    def update(self, tracks, t, image_cx):
        """Une image de pistes au temps t (s). Retourne (etat, piste suivie ou derniere connue, temps sans la voir)."""
        if self.locked_id is not None:
            match = next((tr for tr in tracks if tr.id == self.locked_id), None)
            if match is not None:
                self.last_seen, self.last_track = t, match
                return LOCKED, match, 0.0
            elapsed = t - self.last_seen
            if elapsed < self.lost_timeout:
                return LOST, self.last_track, elapsed
            self.locked_id = None  # perdue trop longtemps : on relache le verrou

        if tracks:
            best = min(tracks, key=lambda tr: abs(tr.cx - image_cx))
            self.locked_id, self.last_seen, self.last_track = best.id, t, best
            self.lock_count += 1
            return LOCKED, best, 0.0
        if self.last_seen is None:
            return IDLE, None, 0.0
        return SEARCHING, self.last_track, t - self.last_seen
