import math

from tb_tracking.target_geometry import bearing, distance_from_height, intrinsics_from_hfov


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
