"""Identite stable des personnes, par-dessus les pistes du tracker. Aucune dependance ROS.

ByteTrack donne un numero de piste a chaque personne, mais il ne regarde que les positions : apres une
occlusion, la personne revient souvent avec un NOUVEAU numero de piste. IdentityKeeper garde un numero de
PERSONNE qui, lui, ne change pas :

- piste deja connue -> meme personne ;
- nouvelle piste (moins de `provisional_time` s, ou dont le pull n'a encore jamais ete bien vu) de la meme
  couleur de vetements qu'une personne perdue depuis moins de `memory` s -> c'est elle qui revient : on lui
  rend son ancien numero ;
- couleur « fiable » = lue sur le tronc d'une personne vue en entier ; une personne coupee (de pres, seules
  les jambes) n'a qu'une couleur de repli, qui ne sert jamais a reconnaitre quelqu'un ;
- une personne n'est jamais confondue avec une personne vue EN MEME TEMPS qu'elle (sauf boite qui la
  chevauche fortement : c'est un doublon de la meme personne, ex. corps + jambes tout pres de la camera) ;
- doublon : une piste nee a l'interieur de la boite d'une personne reprend son numero si cette personne
  disparait ET si sa couleur est la meme (sinon ce peut etre quelqu'un qui etait cache derriere elle).
Sinon -> nouvelle personne, nouveau numero.
"""

from collections import Counter
from dataclasses import dataclass, field


@dataclass
class TrackBox:
    track_id: int   # numero de piste du tracker (ByteTrack)
    cx: float       # centre et taille de la bbox, pixels
    cy: float
    w: float
    h: float
    color: str = ''  # couleur des vetements ('' = inconnue)
    reliable: bool = True  # couleur lue sur le tronc (False : repli, personne coupee par le haut de l'image)


@dataclass
class Person:
    pid: int
    color: str
    box: TrackBox
    first_seen: float
    last_seen: float
    seen_with: set = field(default_factory=set)  # numeros des personnes vues en meme temps qu'elle
    colors: Counter = field(default_factory=Counter)    # couleurs fiables lues (tronc visible)
    fallback: Counter = field(default_factory=Counter)  # couleurs de repli (personne coupee)

    @property
    def reliable(self):
        return bool(self.colors)

    def add_color(self, color, reliable):
        """Couleur de la personne = la plus lue sur toute sa vie, en lectures fiables s'il y en a : une
        lecture fausse de pres (le jean lu « bleu ») ne la change pas."""
        if not color:
            return
        (self.colors if reliable else self.fallback)[color] += 1
        self.color = (self.colors or self.fallback).most_common(1)[0][0]


def overlap(a, b):
    """Part de la plus petite des deux boites couverte par l'autre (0 = disjointes, 1 = l'une dans l'autre)."""
    dx = min(a.cx + a.w / 2, b.cx + b.w / 2) - max(a.cx - a.w / 2, b.cx - b.w / 2)
    dy = min(a.cy + a.h / 2, b.cy + b.h / 2) - max(a.cy - a.h / 2, b.cy - b.h / 2)
    if dx <= 0 or dy <= 0:
        return 0.0
    return dx * dy / max(1e-9, min(a.w * a.h, b.w * b.h))


class IdentityKeeper:

    def __init__(self, memory=60.0, provisional_time=1.0, duplicate_overlap=0.6):
        self.memory = memory                      # s : duree pendant laquelle on se souvient d'une personne
        self.provisional_time = provisional_time  # s : une piste plus jeune peut encore etre reconnue
        self.duplicate_overlap = duplicate_overlap
        self.persons = {}       # numero de personne -> Person
        self.track_to_person = {}
        self.duplicate_of = {}  # piste nee dans la boite d'une personne -> numero de cette personne
        self.next_pid = 1
        self.events = []        # (numero de piste, numero de personne retrouvee) : pour les logs

    def update(self, boxes, t):
        """Une image : liste de TrackBox au temps t (s). Retourne {numero de piste: numero de personne}."""
        self.events = []
        self.forget(t)
        ids = {b.track_id: self.track_to_person.get(b.track_id) for b in boxes}

        # 1. Doublon dont la personne a disparu : il prend son numero, s'il a la meme couleur qu'elle.
        visible = {pid for pid in ids.values() if pid is not None}
        for b in boxes:
            owner = self.duplicate_of.get(b.track_id)
            if (owner is not None and owner in self.persons and owner not in visible
                    and self.persons[owner].reliable and self.track_color(b, ids) == self.persons[owner].color):
                self.reassign(b, owner, ids)
                visible = {pid for pid in ids.values() if pid is not None}

        # 2. Pistes nouvelles, jeunes ou jamais bien vues : une personne perdue qui revient ?
        for b in boxes:
            pid = ids[b.track_id]
            open_to_merge = (pid is None or t - self.persons[pid].first_seen < self.provisional_time
                             or not self.persons[pid].reliable)
            if not open_to_merge or not b.color or not b.reliable:
                continue
            match = self.lost_person_like(b, pid, visible)
            if match is not None:
                self.reassign(b, match, ids)
                visible = {p for p in ids.values() if p is not None}

        # 3. Toujours inconnues : nouvelles personnes (en notant si elles naissent dans une autre boite).
        for b in boxes:
            if ids[b.track_id] is None:
                pid = self.next_pid
                self.next_pid += 1
                self.persons[pid] = Person(pid, b.color, b, t, t)
                self.track_to_person[b.track_id] = pid
                ids[b.track_id] = pid
                host = next((ids[o.track_id] for o in boxes if o is not b and ids[o.track_id] is not None
                             and overlap(b, o) >= self.duplicate_overlap), None)
                if host is not None:
                    self.duplicate_of[b.track_id] = host

        # 4. Mise a jour des personnes visibles.
        for b in boxes:
            person = self.persons[ids[b.track_id]]
            person.box, person.last_seen = b, t
            person.add_color(b.color, b.reliable)
            person.seen_with |= {ids[o.track_id] for o in boxes
                                 if o is not b and overlap(b, o) < self.duplicate_overlap}
        return ids

    def track_color(self, box, ids):
        """Couleur la plus lue de la piste (celle de sa personne provisoire), sinon celle de l'image."""
        pid = ids.get(box.track_id)
        if pid is not None and pid in self.persons and (self.persons[pid].colors or self.persons[pid].fallback):
            return self.persons[pid].color
        return box.color

    def lost_person_like(self, box, own_pid, visible):
        """Personne absente de l'image, de la meme couleur, jamais vue avec cette piste ; la plus proche."""
        candidates = [p for p in self.persons.values()
                      if p.pid not in visible and p.pid != own_pid and p.reliable and p.color == box.color
                      and (own_pid is None or own_pid not in p.seen_with)]
        if not candidates:
            return None
        return min(candidates, key=lambda p: abs(p.box.cx - box.cx)).pid

    def reassign(self, box, pid, ids):
        """La piste `box` est la personne `pid` : son numero provisoire eventuel est abandonne."""
        old = ids[box.track_id]
        if old is not None and old != pid:
            person = self.persons.pop(old, None)
            if person is not None:
                self.persons[pid].seen_with |= person.seen_with - {pid}
                self.persons[pid].colors.update(person.colors)
                self.persons[pid].fallback.update(person.fallback)
        self.track_to_person[box.track_id] = pid
        self.duplicate_of.pop(box.track_id, None)
        ids[box.track_id] = pid
        self.events.append((box.track_id, pid))

    def forget(self, t):
        old = {pid for pid, p in self.persons.items() if t - p.last_seen > self.memory}
        for pid in old:
            del self.persons[pid]
        self.track_to_person = {k: v for k, v in self.track_to_person.items() if v in self.persons}
        self.duplicate_of = {k: v for k, v in self.duplicate_of.items() if v in self.persons}
