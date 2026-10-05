import cv2
import numpy as np

from tb_perception.color_detector import ColorDetector

RED = [((0, 120, 70), (8, 255, 255)), ((172, 120, 70), (180, 255, 255))]


def blank():
    return np.full((480, 640, 3), 180, np.uint8)


def test_detects_red_disc_at_its_position():
    image = blank()
    cv2.circle(image, (400, 300), 40, (20, 20, 220), -1)  # BGR rouge
    dets = ColorDetector(RED).detect(image)
    assert len(dets) == 1
    assert abs(dets[0].cx - 400) <= 2 and abs(dets[0].cy - 300) <= 2
    assert abs(dets[0].h - 81) <= 3
    assert dets[0].score > 0.9


def test_ignores_brown_box_and_gray_scene():
    image = blank()
    cv2.rectangle(image, (50, 150), (150, 250), (51, 89, 140), -1)  # boite marron du monde Gazebo
    assert ColorDetector(RED).detect(image) == []


def test_ignores_small_blobs():
    image = blank()
    cv2.circle(image, (100, 100), 4, (20, 20, 220), -1)
    assert ColorDetector(RED, min_area=150).detect(image) == []


def test_keeps_largest_detections_first():
    image = blank()
    cv2.circle(image, (100, 100), 20, (20, 20, 220), -1)
    cv2.circle(image, (400, 300), 50, (20, 20, 220), -1)
    dets = ColorDetector(RED, max_detections=2).detect(image)
    assert [round(d.cx) for d in dets] == [400, 100]


TENNIS = [((25, 80, 100), (45, 255, 255))]  # config/sim.yaml


def test_detects_small_tennis_ball():
    image = blank()
    cv2.circle(image, (180, 266), 8, (0, 230, 190), -1)  # BGR jaune-vert fluo, ~16 px comme a 2 m
    dets = ColorDetector(TENNIS, min_area=40).detect(image)
    assert len(dets) == 1
    assert abs(dets[0].cx - 180) <= 1 and abs(dets[0].h - 17) <= 2


def test_tennis_ranges_ignore_green_tshirt():
    image = blank()
    cv2.rectangle(image, (250, 0), (350, 120), (45, 95, 55), -1)  # t-shirt de l'actor : H 54, V 95 (mesure en sim)
    assert ColorDetector(TENNIS, min_area=40).detect(image) == []
