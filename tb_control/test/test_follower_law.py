import math

from tb_control.follower_law import FollowerParams, compute_command, rate_limit

P = FollowerParams()


def test_target_centered_at_setpoint_gives_zero():
    assert compute_command(0.0, P.distance_setpoint, P) == (0.0, 0.0)


def test_turns_toward_target():
    _, w = compute_command(0.3, 1.0, P)
    assert w > 0
    _, w = compute_command(-0.3, 1.0, P)
    assert w < 0


def test_speed_saturation_sec02():
    v, w = compute_command(3.0, 10.0, P)
    assert v == P.max_linear and w == P.max_angular


def test_never_moves_backward():
    v, _ = compute_command(0.0, 0.7, P)
    assert v == 0.0


def test_stops_below_min_distance_sec04():
    v, _ = compute_command(0.0, P.min_distance - 0.01, P)
    assert v == 0.0


def test_unknown_distance_does_not_advance():
    v, _ = compute_command(0.0, math.nan, P)
    assert v == 0.0


def test_rate_limit_sec03():
    assert rate_limit(0.0, 1.0, max_rate=0.3, dt=0.05) == 0.3 * 0.05
    assert rate_limit(0.1, 0.1, max_rate=0.3, dt=0.05) == 0.1
