import pytest

from tb_bringup.actor_skins import recolor_sweater

DAE = """<effect id="skin-mat-effect">
<color sid="ambient">0.8 0.5 0.4 1</color><color sid="diffuse">0.8 0.5 0.4 1</color></effect>
<effect id="sweater-green-effect"><profile_COMMON><technique sid="common"><phong>
<color sid="emission">0 0 0 1</color><ambient><color sid="ambient">0.1 0.25 0.07 1</color></ambient>
<diffuse><color sid="diffuse">0.1 0.25 0.07 1</color></diffuse></phong></technique></profile_COMMON></effect>"""


def test_recolors_only_the_sweater():
    out = recolor_sweater(DAE, (0.75, 0.05, 0.05))
    assert out.count('0.75 0.05 0.05 1') == 2               # ambient + diffuse du pull
    assert '<color sid="emission">0 0 0 1</color>' in out
    assert out.count('0.8 0.5 0.4 1') == 2                  # la peau ne change pas


def test_fails_clearly_if_material_missing():
    with pytest.raises(ValueError):
        recolor_sweater('<effect id="other"></effect>', (1, 0, 0))
