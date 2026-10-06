from tb_tracking.target_lock import IDLE, LOCKED, LOST, SEARCHING, TargetLock, TrackObs

CX = 320.0


def obs(track_id, cx):
    return TrackObs(track_id, cx, 240.0, 80.0, 200.0)


def test_idle_until_someone_appears():
    lock = TargetLock()
    assert lock.update([], 0.0, CX)[0] == IDLE


def test_locks_track_closest_to_center_ef05():
    lock = TargetLock()
    state, track, _ = lock.update([obs(4, 100.0), obs(7, 330.0), obs(9, 600.0)], 0.0, CX)
    assert state == LOCKED and track.id == 7


def test_keeps_locked_id_when_another_person_is_more_central():
    lock = TargetLock()
    lock.update([obs(7, 330.0)], 0.0, CX)
    state, track, _ = lock.update([obs(7, 500.0), obs(8, 321.0)], 0.1, CX)
    assert state == LOCKED and track.id == 7


def test_short_occlusion_is_lost_not_switched_ef04():
    lock = TargetLock(lost_timeout=2.0)
    lock.update([obs(7, 330.0)], 0.0, CX)
    state, track, elapsed = lock.update([obs(8, 320.0)], 1.5, CX)  # la cible est cachee, quelqu'un d'autre au centre
    assert state == LOST and track.id == 7 and abs(elapsed - 1.5) < 1e-9
    state, track, _ = lock.update([obs(7, 340.0), obs(8, 320.0)], 1.8, CX)  # elle reapparait avec le meme ID
    assert state == LOCKED and track.id == 7 and lock.lock_count == 1


def test_reacquires_after_timeout():
    lock = TargetLock(lost_timeout=2.0)
    lock.update([obs(7, 330.0)], 0.0, CX)
    state, track, _ = lock.update([obs(8, 200.0)], 2.5, CX)
    assert state == LOCKED and track.id == 8 and lock.lock_count == 2


def test_searching_when_nobody_after_timeout():
    lock = TargetLock(lost_timeout=2.0)
    lock.update([obs(7, 330.0)], 0.0, CX)
    state, track, elapsed = lock.update([], 3.0, CX)
    assert state == SEARCHING and track.id == 7 and elapsed == 3.0
