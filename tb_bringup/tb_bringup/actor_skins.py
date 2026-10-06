"""Copies colorees du modele de personne de Gazebo Fuel : un pull de couleur differente par personne.

Le modele 3D (Mingfei/actor, licence CC BY 4.0) definit la couleur du pull dans un materiau
'sweater-green' de son fichier walk.dae. On en ecrit une copie par couleur, a l'execution, dans
<cache>/models/person_actors/meshes/walk_<couleur>.dae (reference dans le monde par model://person_actors/...).
"""

import glob
import os
import re
import urllib.request

SOURCE_URL = 'https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae'
FUEL_CACHE_GLOB = '~/.gz/fuel/fuel.gazebosim.org/mingfei/models/actor/*/meshes/walk.dae'

# Couleurs RGB (0-1) des pulls : teintes bien separees, aucune proche du jaune-vert de la balle de tennis
COLORS = {
    'rouge': (0.75, 0.05, 0.05),
    'vert': (0.10, 0.60, 0.10),
    'violet': (0.50, 0.10, 0.65),
}


def recolor_sweater(dae_text, rgb):
    """Remplace les couleurs ambient et diffuse de l'effet 'sweater-green-effect' du fichier COLLADA."""
    color = f'{rgb[0]} {rgb[1]} {rgb[2]} 1'

    def recolor(match):
        block = match.group(0)
        for sid in ('ambient', 'diffuse'):
            block = re.sub(rf'(<color sid="{sid}">)[^<]*(</color>)', rf'\g<1>{color}\g<2>', block)
        return block

    result, count = re.subn(r'<effect id="sweater-green-effect">.*?</effect>', recolor, dae_text, flags=re.S)
    if count != 1:
        raise ValueError("materiau 'sweater-green-effect' introuvable dans le modele de personne")
    return result


def load_source():
    cached = sorted(glob.glob(os.path.expanduser(FUEL_CACHE_GLOB)))
    if cached:
        with open(cached[-1], encoding='utf8') as f:
            return f.read()
    with urllib.request.urlopen(SOURCE_URL, timeout=60) as response:
        return response.read().decode('utf8')


def ensure_skins(cache_dir='~/.cache/person_tracking'):
    """Cree les copies colorees si elles manquent. Retourne le dossier a ajouter a GZ_SIM_RESOURCE_PATH."""
    models_dir = os.path.join(os.path.expanduser(cache_dir), 'models')
    meshes = os.path.join(models_dir, 'person_actors', 'meshes')
    missing = [name for name in COLORS if not os.path.isfile(os.path.join(meshes, f'walk_{name}.dae'))]
    if missing:
        os.makedirs(meshes, exist_ok=True)
        source = load_source()
        for name in missing:
            with open(os.path.join(meshes, f'walk_{name}.dae'), 'w', encoding='utf8') as f:
                f.write(recolor_sweater(source, COLORS[name]))
    return models_dir
