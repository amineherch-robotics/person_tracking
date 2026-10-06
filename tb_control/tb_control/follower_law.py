"""Loi de commande du suiveur (asservissement visuel P) et limites de securite. Aucune dependance ROS."""

from dataclasses import dataclass
import math


@dataclass
class FollowerParams:
    k_angular: float = 1.2           # rad/s par rad d'erreur d'angle
    k_linear: float = 0.5            # m/s par m d'erreur de distance
    distance_setpoint: float = 1.0   # m (EF-06)
    angular_deadband: float = 0.03   # rad, zone morte
    distance_deadband: float = 0.05  # m, zone morte
    max_linear: float = 0.15         # m/s (SEC-02)
    max_angular: float = 1.0         # rad/s (SEC-02)
    min_distance: float = 0.5        # m, arret en dessous (SEC-04)
    max_reverse: float = 0.0         # m/s, recul autorise si la cible est trop proche (0 = jamais)


def clamp(value, low, high):
    return max(low, min(high, value))


def compute_command(bearing, distance, p):
    """Vitesses (v, w) visees pour une cible a l'angle `bearing` (rad) et a `distance` (m, NaN si inconnue).

    Le robot s'arrete si la distance est inconnue ou sous min_distance (SEC-04). Il ne recule que si
    max_reverse > 0, et au plus a cette vitesse : la camera ne voit pas derriere lui.
    """
    w = 0.0 if abs(bearing) < p.angular_deadband else p.k_angular * bearing
    w = clamp(w, -p.max_angular, p.max_angular)

    if math.isnan(distance) or distance < p.min_distance:
        v = 0.0
    else:
        error = distance - p.distance_setpoint
        v = 0.0 if abs(error) < p.distance_deadband else p.k_linear * error
        v = clamp(v, -p.max_reverse, p.max_linear)
    return v, w


def rate_limit(previous, target, max_rate, dt):
    """Limite la variation de la commande a max_rate par seconde (SEC-03)."""
    step = max_rate * dt
    return previous + clamp(target - previous, -step, step)
