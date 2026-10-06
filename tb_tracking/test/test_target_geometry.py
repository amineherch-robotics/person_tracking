import math

from tb_tracking.target_geometry import bearing, distance_from_height, distance_from_scan, intrinsics_from_hfov


def test_intrinsics_match_gazebo_camera():
    fx, fy, cx, cy = intrinsics_from_hfov(640, 480, 1.2217)
    assert abs(fx - 457.0) < 0.5 and fx == fy
    assert (cx, cy) == (320.0, 240.0)


def test_bearing_sign_convention():
    assert bearing(320, 320, 457) == 0.0
    assert bearing(100, 320, 457) > 0  # objet a gauche de l'image -> angle positif
    assert bearing(600, 320, 457) < 0


def test_distance_from_height():
    assert abs(distance_from_height(91.4, 457.0, 0.2) - 1.0) < 1e-3
    assert math.isnan(distance_from_height(0.0, 457.0, 0.2))


def scan_with(objects, background=3.0):
    """Balayage 360 rayons (1 par degre, angle_min = 0) avec un mur a `background` et des objets {deg: dist}."""
    ranges = [background] * 360
    for deg, dist in objects.items():
        ranges[deg % 360] = dist
    return ranges


def test_scan_distance_finds_legs_in_front_of_wall():
    ranges = scan_with({-2: 1.02, -1: 1.0, 1: 1.05, 2: 1.01})  # deux jambes devant, mur a 3 m
    d = distance_from_scan(ranges, 0.0, math.radians(1), 0.0, math.radians(5), 0.12, 3.5)
    assert 0.99 <= d <= 1.06


def test_scan_distance_wraps_around_zero_and_uses_bearing():
    ranges = scan_with({30: 1.5, 31: 1.5, 0: 0.4})  # objet a 30 deg a gauche ; un obstacle proche droit devant
    d = distance_from_scan(ranges, 0.0, math.radians(1), math.radians(30), math.radians(3), 0.12, 3.5)
    assert abs(d - 1.5) < 1e-6


def test_scan_distance_ignores_invalid_rays():
    ranges = scan_with({0: float('inf'), 1: 0.05, -1: float('nan')}, background=float('inf'))
    assert math.isnan(distance_from_scan(ranges, 0.0, math.radians(1), 0.0, math.radians(2), 0.12, 3.5))
