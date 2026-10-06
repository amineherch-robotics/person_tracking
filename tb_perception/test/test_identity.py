from tb_perception.identity import IdentityKeeper, TrackBox


def box(track_id, cx, color, cy=240.0, w=80.0, h=200.0, reliable=True):
    return TrackBox(track_id, cx, cy, w, h, color, reliable)


def test_each_new_person_gets_its_own_number():
    keeper = IdentityKeeper()
    ids = keeper.update([box(11, 100.0, 'rouge'), box(12, 400.0, 'vert')], 0.0)
    assert ids == {11: 1, 12: 2}
    assert keeper.update([box(11, 110.0, 'rouge'), box(12, 390.0, 'vert')], 0.1) == {11: 1, 12: 2}


def test_person_back_under_new_tracker_id_keeps_her_number():
    # La rouge (piste 11) est cachee par la verte (piste 12), puis ByteTrack la rend sous la piste 27.
    keeper = IdentityKeeper()
    keeper.update([box(11, 300.0, 'rouge'), box(12, 500.0, 'vert')], 0.0)
    keeper.update([box(12, 310.0, 'vert')], 1.0)
    ids = keeper.update([box(12, 200.0, 'vert'), box(27, 330.0, 'rouge')], 3.0)
    assert ids == {12: 2, 27: 1}
    assert keeper.events == [(27, 1)]


def test_back_after_a_long_time_within_memory():
    keeper = IdentityKeeper(memory=60.0)
    keeper.update([box(11, 300.0, 'rouge')], 0.0)
    assert keeper.update([box(40, 100.0, 'rouge')], 45.0) == {40: 1}


def test_forgotten_after_memory():
    keeper = IdentityKeeper(memory=60.0)
    keeper.update([box(11, 300.0, 'rouge')], 0.0)
    assert keeper.update([box(40, 100.0, 'rouge')], 70.0) == {40: 2}


def test_colour_read_one_image_late_still_recognised():
    keeper = IdentityKeeper(provisional_time=1.0)
    keeper.update([box(11, 300.0, 'rouge')], 0.0)
    assert keeper.update([box(27, 320.0, '')], 2.0) == {27: 2}       # couleur pas encore lue : provisoire
    assert keeper.update([box(27, 320.0, 'rouge')], 2.1) == {27: 1}  # puis reconnue


def test_old_track_is_not_renamed():
    # Une piste suivie depuis longtemps ne change pas de numero, meme si sa couleur change.
    keeper = IdentityKeeper(provisional_time=1.0)
    keeper.update([box(11, 300.0, 'rouge')], 0.0)
    keeper.update([box(12, 500.0, 'vert')], 5.0)
    assert keeper.update([box(12, 500.0, 'rouge')], 10.0) == {12: 2}


def test_same_colour_person_seen_together_is_not_merged():
    keeper = IdentityKeeper()
    keeper.update([box(11, 100.0, 'rouge'), box(12, 500.0, 'rouge')], 0.0)
    assert keeper.update([box(12, 500.0, 'rouge')], 0.5) == {12: 2}


def test_duplicate_box_takes_over_when_the_person_disappears():
    # Video 1 (06/10) : tout pres de la camera, corps (piste 4) + jambes (piste 22, lues « bleu »).
    keeper = IdentityKeeper()
    keeper.update([box(4, 500.0, 'rouge', cy=200.0, w=200.0, h=400.0)], 0.0)
    ids = keeper.update([box(4, 500.0, 'rouge', cy=200.0, w=200.0, h=400.0),
                         box(22, 500.0, 'bleu', cy=300.0, w=180.0, h=200.0)], 5.0)
    assert ids == {4: 1, 22: 2}
    # ID 4 disparait ; le doublon est encore lu « bleu » : on attend (ce pourrait etre quelqu'un d'autre)
    assert keeper.update([box(22, 520.0, 'bleu', cy=240.0, w=200.0, h=380.0)], 5.5) == {22: 2}
    for i in range(4):   # puis la personne recule : son pull est lu « rouge » (majoritaire a la 3e lecture)
        ids = keeper.update([box(22, 520.0, 'rouge', cy=240.0, w=200.0, h=380.0)], 6.0 + i)
    assert ids == {22: 1}


def test_person_hidden_behind_cannot_steal_the_front_person_number():
    # La verte (piste 12) etait cachee derriere la rouge (piste 11) ; elle reapparait dans sa boite
    # sous la piste 30, puis la rouge sort de l'image : la verte ne doit pas devenir la rouge.
    keeper = IdentityKeeper()
    keeper.update([box(11, 300.0, 'rouge', w=120.0, h=300.0), box(12, 600.0, 'vert')], 0.0)
    for i in range(20):   # 12 disparait derriere 11 pendant 2 s (au-dela de la periode provisoire)
        keeper.update([box(11, 300.0, 'rouge', w=120.0, h=300.0)], 0.1 + 0.1 * i)
    ids = keeper.update([box(11, 300.0, 'rouge', w=120.0, h=300.0), box(30, 310.0, 'vert')], 5.0)
    assert ids[30] == 2                                  # reconnue : la verte
    assert keeper.update([box(30, 310.0, 'vert')], 5.5) == {30: 2}


def test_closest_lost_person_is_chosen_when_two_match():
    keeper = IdentityKeeper()
    keeper.update([box(11, 100.0, 'rouge'), box(12, 600.0, 'rouge')], 0.0)  # deux personnes en rouge
    keeper.update([], 1.0)                                                # toutes les deux cachees
    assert keeper.update([box(30, 580.0, 'rouge')], 2.0) == {30: 2}       # revient a droite : la 2


def test_wrong_colour_read_up_close_does_not_change_the_person():
    keeper = IdentityKeeper()
    for i in range(10):
        keeper.update([box(11, 300.0, 'rouge')], 0.1 * i)
    keeper.update([box(11, 300.0, 'bleu')], 1.0)          # de pres, on lit le jean
    keeper.update([], 2.0)
    assert keeper.update([box(30, 320.0, 'rouge')], 3.0) == {30: 1}


def test_person_seen_up_close_is_recognised_once_her_sweater_is_visible():
    # La rouge revient tout pres de la camera : seules ses jambes sont visibles (repli « bleu », pas fiable).
    keeper = IdentityKeeper(provisional_time=1.0)
    keeper.update([box(11, 300.0, 'rouge')], 0.0)
    keeper.update([], 1.0)
    for i in range(30):   # 3 s de jambes seules : numero provisoire, jamais reconnue comme quelqu'un d'autre
        ids = keeper.update([box(40, 320.0, 'bleu', reliable=False)], 2.0 + 0.1 * i)
    assert ids == {40: 2}
    assert keeper.update([box(40, 320.0, 'rouge')], 6.0) == {40: 1}   # elle recule : pull visible -> ID 1


def test_fallback_colour_never_matches_a_lost_person():
    keeper = IdentityKeeper()
    keeper.update([box(11, 300.0, 'bleu')], 0.0)          # une personne en pull bleu, bien vue
    keeper.update([], 1.0)
    assert keeper.update([box(40, 320.0, 'bleu', reliable=False)], 2.0) == {40: 2}  # jean d'une autre
