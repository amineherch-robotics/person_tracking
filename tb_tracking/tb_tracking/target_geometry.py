"""Geometrie camera pinhole : bbox en pixels -> angle et distance. Aucune dependance ROS."""

import math


def intrinsics_from_hfov(width, height, hfov):
    """fx, fy, cx, cy pour une camera sans calibration (pixels carres)."""
    fx = (width / 2.0) / math.tan(hfov / 2.0)
    return fx, fx, width / 2.0, height / 2.0


def bearing(u, cx, fx):
    """Angle horizontal vers le pixel u, en rad, positif a gauche (convention ROS, z vers le haut)."""
    return math.atan2(cx - u, fx)


def distance_from_height(h_px, fy, object_height):
    """Distance a un objet de hauteur connue a partir de la hauteur de sa bbox. NaN si bbox vide."""
    if h_px <= 0.0:
        return float('nan')
    return fy * object_height / h_px


def distance_from_scan(ranges, angle_min, angle_increment, bearing, half_width, range_min, range_max,
                       percentile=20.0):
    """Distance a la cible mesuree par le LiDAR dans la direction de la camera (fusion camera-LiDAR, EF-09).

    On garde les rayons du secteur [bearing - half_width, bearing + half_width] (rad, meme convention que
    bearing : 0 devant, positif a gauche) et on prend un percentile bas des distances valides : les jambes
    sont le plus proche dans le secteur, le mur derriere ne compte pas. NaN si aucun rayon valide.
    """
    n = len(ranges)
    if n == 0 or angle_increment <= 0.0:
        return float('nan')
    values = []
    steps = int(math.ceil(half_width / angle_increment))
    center = (bearing - angle_min) / angle_increment
    for k in range(int(round(center)) - steps, int(round(center)) + steps + 1):
        r = ranges[k % n]  # balayage sur 360 deg : on fait le tour
        if math.isfinite(r) and range_min <= r <= range_max:
            values.append(r)
    if not values:
        return float('nan')
    values.sort()
    return values[min(len(values) - 1, int(len(values) * percentile / 100.0))]
