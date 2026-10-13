# Architecture

Le Raspberry Pi du robot gère uniquement capteurs et moteurs. Toute la chaîne de perception et de commande tourne sur le laptop, relié en WiFi (même `ROS_DOMAIN_ID`).

```
 Robot (Raspberry Pi)                     Laptop (ROS2 Jazzy)
 ┌──────────────────┐  image compressed  ┌────────────────────────────┐
 │ usb_cam          │ ─────────────────▶ │ detector_node              │  tb_perception
 └──────────────────┘                    │ YOLO26n + ByteTrack        │
                                         │ couleur des vêtements      │
                                         │ ID de personne stable      │
                                         └──────────┬─────────────────┘
                                                    │ /tracks (personnes) ou /detections (balle)
                                                    │            │
                                                    │            ▼
                                                    │   target_chooser (fenêtre) ──▶ /target/select
                                                    │                                     │
 ┌──────────────────┐       /scan        ┌──────────▼─────────────────┐                   │
 │ turtlebot3_      │ ─────────────────▶ │ target_selector_node       │ ◀─────────────────┘
 │ bringup          │                    │ verrouillage de l'ID,      │  tb_tracking
 │ moteurs, /odom,  │                    │ angle + distance (LiDAR)   │
 │ /scan            │                    └──────────┬─────────────────┘
 │                  │                               │ /target
 │                  │       /cmd_vel     ┌──────────▼─────────────────┐
 │                  │ ◀───────────────── │ follower_controller        │  tb_control
 └──────────────────┘                    │ contrôleur P, recherche,   │
                                         │ watchdog, limites          │
                                         └────────────────────────────┘
```

En simulation, Gazebo remplace le robot : mêmes topics, sauf l'image publiée en `raw` sur `/camera/image_raw`.

## Deux cibles possibles (`target:=`)

| | `target:=person` (par défaut) | `target:=ball` |
|---|---|---|
| Détection | YOLO26n (classe personne) | Seuillage de couleur HSV |
| Suivi | ByteTrack (Ultralytics) + `IdentityKeeper` : un ID par personne, gardé quand elle revient après une perte (reconnue par la couleur de ses vêtements) | Aucun : meilleure détection de chaque image |
| Choix de la cible | L'utilisateur clique sur une personne (`target_chooser`) | La balle |
| Distance | LiDAR dans la direction de la personne ; au-delà de 3,5 m, hauteur de la silhouette | Hauteur de la balle dans l'image |
| Config | `config/target_person.yaml` | `config/target_ball.yaml` |

## Topics de la chaîne

| Topic | Type | Publié par | Consommé par |
|---|---|---|---|
| image (`image_topic`) | `sensor_msgs/Image` ou `CompressedImage` | usb_cam / Gazebo | detector_node, target_chooser |
| `/tracks` | `tb_interfaces/TrackArray` | detector_node (personnes) | target_selector_node, target_chooser |
| `/detections` | `vision_msgs/Detection2DArray` | detector_node | target_selector_node (balle) |
| `/target/select` | `std_msgs/Int32` (ID choisi, −1 = arrêter) | target_chooser | target_selector_node |
| `/target` | `tb_interfaces/TargetState` | target_selector_node | follower_controller, target_chooser |
| `/scan` | `sensor_msgs/LaserScan` | robot / Gazebo | target_selector_node (EF-09) |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | follower_controller | robot / Gazebo |
| `/detector/debug_image` | `sensor_msgs/Image` | detector_node | rqt_image_view |
| `/follower/enable` | `std_msgs/Bool` (transient local) | estop_keyboard | follower_controller |

Tous les messages de la chaîne recopient le `header.stamp` de l'image d'origine (ENF-01), ce qui permet de mesurer la latence image → `/cmd_vel` dans le contrôleur.

## États de la cible et réaction du robot

| État (`TargetState.state`) | Signification | Robot |
|---|---|---|
| IDLE | Aucune cible choisie | Arrêté |
| LOCKED | Cible vue | La suit (contrôleur P, consigne 1 m) |
| LOST | Cachée depuis moins de `lost_timeout` (2 s) | Arrêté, attend |
| SEARCHING | Perdue depuis plus de 2 s | Tourne sur place du côté où elle a disparu (`search_angular`, désactivé en réel) |

## Messages (`tb_interfaces`)

- **`Track`** : `id` (ID de personne), `bbox` (`vision_msgs/BoundingBox2D`), `score`, vitesse `vx`/`vy` en px/s, `age`, `frames_since_update`, `confirmed`, `color` (couleur des vêtements).
- **`TrackArray`** : `header` + `Track[]`.
- **`TargetState`** : `state` (IDLE, LOCKED, LOST, SEARCHING), `target_id`, `bbox`, `bearing` (rad), `distance` (m, NaN si inconnue), `distance_source` (NONE, BBOX, LIDAR), `time_since_seen`.

## Paramètres

`tb_bringup/config/sim.yaml` et `real.yaml` regroupent les paramètres communs (topics, `use_sim_time`, transport d'image, gains et limites du contrôleur). `target_person.yaml` et `target_ball.yaml` contiennent ceux de la détection et du choix de la cible. Aucun topic, seuil ou gain codé en dur dans les nodes (ENF-02).
