# Etat de la cible verrouillee, publie par target_selector_node sur /target.

uint8 STATE_IDLE = 0               # aucune cible choisie
uint8 STATE_LOCKED = 1             # cible verrouillee et visible
uint8 STATE_LOST = 2               # cible perdue depuis moins de lost_timeout (prediction Kalman)
uint8 STATE_SEARCHING = 3          # cible perdue depuis plus de lost_timeout : mode recherche (EF-07)

uint8 DISTANCE_NONE = 0
uint8 DISTANCE_BBOX = 1            # distance estimee par la hauteur de la bbox
uint8 DISTANCE_LIDAR = 2           # distance par fusion camera-LiDAR (EF-09)

std_msgs/Header header             # stamp de l'image d'origine (ENF-01)
uint8 state
uint32 target_id                   # id de la piste suivie (Track.id)
vision_msgs/BoundingBox2D bbox
float32 bearing                    # angle horizontal vers la cible, rad, positif a gauche
float32 distance                   # m, NaN si inconnue
uint8 distance_source
float32 time_since_seen            # s depuis la derniere detection associee a la cible
