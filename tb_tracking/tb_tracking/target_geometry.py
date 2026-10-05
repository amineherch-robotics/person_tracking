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
