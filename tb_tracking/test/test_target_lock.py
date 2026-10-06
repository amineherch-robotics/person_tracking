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


def test_manual_mode_waits_for_user_choice():
    lock = TargetLock(auto_select=False)
    state, _, _ = lock.update([obs(4, 320.0), obs(7, 100.0)], 0.0, CX)
    assert state == IDLE
    lock.select(7, 0.5)
    state, track, _ = lock.update([obs(4, 320.0), obs(7, 100.0)], 0.6, CX)
    assert state == LOCKED and track.id == 7


def test_manual_mode_keeps_chosen_id_after_timeout():
    lock = TargetLock(lost_timeout=2.0, auto_select=False)
    lock.select(7, 0.0)
    lock.update([obs(7, 300.0)], 0.1, CX)
    state, track, _ = lock.update([obs(8, 320.0)], 5.0, CX)   # 7 a disparu, 8 au centre : on ne change pas
    assert state == SEARCHING and track.id == 7
    state, track, _ = lock.update([obs(7, 200.0), obs(8, 320.0)], 6.0, CX)
    assert state == LOCKED and track.id == 7


def test_user_can_cancel_and_change_target():
    lock = TargetLock(auto_select=False)
    lock.select(7, 0.0)
    lock.select(-1, 1.0)
    assert lock.update([obs(7, 300.0)], 1.1, CX)[0] == IDLE
    lock.select(4, 2.0)
    state, track, _ = lock.update([obs(4, 500.0), obs(7, 300.0)], 2.1, CX)
    assert state == LOCKED and track.id == 4


def cobs(track_id, cx, color):
    return TrackObs(track_id, cx, 240.0, 80.0, 200.0, color)


def test_reidentifies_target_by_clothing_color_after_occlusion():
    lock = TargetLock(lost_timeout=2.0, auto_select=False)
    lock.select(7, 0.0)
    lock.update([cobs(7, 300.0, 'rouge'), cobs(8, 500.0, 'vert')], 0.1, CX)
    state, track, _ = lock.update([cobs(8, 310.0, 'vert')], 0.5, CX)       # 8 passe devant 7
    assert state == LOST and track.id == 7
    state, track, _ = lock.update([cobs(8, 200.0, 'vert'), cobs(12, 320.0, 'rouge')], 1.0, CX)  # 7 revient en 12
    assert state == LOCKED and track.id == 12 and lock.reid_count == 1


def test_reidentifies_even_after_timeout_in_manual_mode():
    lock = TargetLock(lost_timeout=2.0, auto_select=False)
    lock.select(7, 0.0)
    lock.update([cobs(7, 300.0, 'rouge')], 0.1, CX)
    assert lock.update([], 4.0, CX)[0] == SEARCHING
    state, track, _ = lock.update([cobs(15, 100.0, 'rouge')], 5.0, CX)
    assert state == LOCKED and track.id == 15


def test_does_not_reidentify_a_person_seen_with_the_target():
    lock = TargetLock(lost_timeout=2.0, auto_select=False)
    lock.select(7, 0.0)
    lock.update([cobs(7, 300.0, 'rouge'), cobs(9, 600.0, 'rouge')], 0.1, CX)  # 9 : meme couleur, autre personne
    state, track, _ = lock.update([cobs(9, 600.0, 'rouge')], 0.5, CX)
    assert state == LOST and track.id == 7


def test_id_taken_by_another_person_is_not_followed():
    lock = TargetLock(lost_timeout=2.0, auto_select=False)
    lock.select(7, 0.0)
    lock.update([cobs(7, 300.0, 'rouge'), cobs(8, 500.0, 'vert')], 0.1, CX)
    state, track, _ = lock.update([cobs(7, 310.0, 'vert')], 0.5, CX)   # 8 a pris l'ID 7 en passant devant
    assert state == LOST and track.cx == 300.0
    state, track, _ = lock.update([cobs(7, 200.0, 'vert'), cobs(13, 330.0, 'rouge')], 1.0, CX)
    assert state == LOCKED and track.id == 13


def test_unknown_color_keeps_following_the_id():
    lock = TargetLock(auto_select=False)
    lock.select(7, 0.0)
    lock.update([cobs(7, 300.0, 'rouge')], 0.1, CX)
    state, track, _ = lock.update([cobs(7, 310.0, '')], 0.2, CX)  # couleur pas encore estimee : on garde l'ID
    assert state == LOCKED and track.id == 7


def test_duplicate_box_of_the_target_can_take_over():
    # Video 1 (06/10) : la personne rouge, tres pres, est detectee deux fois (corps ID 4 + jambes ID 22) ;
    # ID 4 disparait : ID 22, qui chevauchait la cible, doit pouvoir la re-identifier.
    lock = TargetLock(lost_timeout=2.0, auto_select=False)
    lock.select(4, 0.0)
    lock.update([TrackObs(4, 500.0, 200.0, 200.0, 400.0, 'rouge'),
                 TrackObs(22, 500.0, 300.0, 180.0, 200.0, 'bleu')], 0.1, CX)
    state, track, _ = lock.update([TrackObs(22, 520.0, 240.0, 200.0, 380.0, 'rouge')], 1.0, CX)
    assert state == LOCKED and track.id == 22
