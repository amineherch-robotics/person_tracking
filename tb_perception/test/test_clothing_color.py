import numpy as np

from tb_perception.clothing_color import clothing_color

# Personne simulee : pull (haut) et jean (bas) dans une bbox centree en (320, 240), 100 x 300 px


def person(sweater_bgr, jeans_bgr=(90, 23, 15)):
    img = np.full((480, 640, 3), 150, np.uint8)
    img[90:240, 270:370] = sweater_bgr
    img[240:390, 270:370] = jeans_bgr
    return img


def test_colors_of_the_three_simulated_people():
    # Couleurs des pulls du monde Gazebo (RGB 0-1 converti en BGR 0-255)
    assert clothing_color(person((13, 13, 191)), 320, 240, 100, 300) == 'rouge'
    assert clothing_color(person((25, 153, 25)), 320, 240, 100, 300) == 'vert'
    assert clothing_color(person((166, 25, 128)), 320, 240, 100, 300) == 'violet'


def test_grey_sweater_is_not_a_color():
    assert clothing_color(person((128, 128, 128)), 320, 240, 100, 300) == 'gris'


def test_box_outside_image_returns_empty():
    assert clothing_color(person((13, 13, 191)), 700, 240, 10, 300) == ''


def test_band_reads_chest_of_person_cut_at_the_neck():
    img = np.full((480, 640, 3), 150, np.uint8)
    img[0:100, 270:370] = (13, 13, 191)    # pull rouge en haut de l'image
    img[100:400, 270:370] = (90, 23, 15)   # jean (bande par defaut 60-180 : 2/3 de jean)
    assert clothing_color(img, 320, 200, 100, 400) == 'bleu'                    # bande par defaut : le jean
    assert clothing_color(img, 320, 200, 100, 400, band=(0.0, 0.25)) == 'rouge'  # repli : le haut visible
