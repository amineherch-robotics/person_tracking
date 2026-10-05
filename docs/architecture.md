# Architecture

Le Raspberry Pi du robot gère uniquement capteurs et moteurs. Toute la chaîne de perception et de commande tourne sur le laptop, relié en WiFi (même `ROS_DOMAIN_ID`).

```
 Robot (Raspberry Pi)                     Laptop (ROS2 Jazzy)
 ┌──────────────────┐  image compressed  ┌──────────────────────┐
 │ usb_cam          │ ─────────────────▶ │ detector_node        │  tb_perception
 └──────────────────┘                    └──────────┬───────────┘
                                                    │ /detections
                                         ┌──────────▼───────────┐
                                         │ tracker_node (SORT)  │  tb_tracking
                                         └──────────┬───────────┘
                                                    │ /tracks
 ┌──────────────────┐       /scan        ┌──────────▼───────────┐
 │ turtlebot3_      │ ─────────────────▶ │ target_selector_node │  tb_tracking
 │ bringup          │                    └──────────┬───────────┘
 │ moteurs, /odom,  │                               │ /target
 │ /scan            │       /cmd_vel     ┌──────────▼───────────┐
 │                  │ ◀───────────────── │ follower_controller  │  tb_control
 └──────────────────┘                    └──────────────────────┘
                                         debug_viz_node : /tracks → /debug_image (EF-10)
```

En simulation, Gazebo remplace le robot : mêmes topics, sauf l'image publiée en `raw` sur `/camera/image_raw`.

## Topics de la chaîne

| Topic | Type | Publié par | Consommé par |
|---|---|---|---|
| image (`image_topic`) | `sensor_msgs/Image` ou `CompressedImage` | usb_cam / Gazebo | detector_node |
| `/detections` | `vision_msgs/Detection2DArray` | detector_node | tracker_node |
| `/tracks` | `tb_interfaces/TrackArray` | tracker_node | target_selector_node, debug_viz_node |
| `/target` | `tb_interfaces/TargetState` | target_selector_node | follower_controller |
| `/scan` | `sensor_msgs/LaserScan` | robot / Gazebo | target_selector_node (EF-09) |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | follower_controller | robot / Gazebo |
| `/debug_image` | `sensor_msgs/Image` | debug_viz_node | rqt_image_view |
| `/detector/debug_image` | `sensor_msgs/Image` | detector_node (Phase 1) | rqt_image_view |
| `/follower/enable` | `std_msgs/Bool` (transient local) | estop_keyboard | follower_controller |

**Phase 1 :** il n'y a pas encore de tracker. `target_selector_node` lit directement `/detections` et prend la détection de meilleur score. En Phase 3, son entrée deviendra `/tracks`, sans changer `/target` ni le contrôleur.

Tous les messages de la chaîne recopient le `header.stamp` de l'image d'origine (ENF-01), ce qui permet de mesurer la latence image → `/cmd_vel` dans le contrôleur.

## Messages (`tb_interfaces`)

- **`Track`** : `id`, `bbox` (`vision_msgs/BoundingBox2D`), `score`, vitesse `vx`/`vy` en px/s, `age`, `frames_since_update`, `confirmed`.
- **`TrackArray`** : `header` + `Track[]`.
- **`TargetState`** : `state` (IDLE, LOCKED, LOST, SEARCHING), `target_id`, `bbox`, `bearing` (rad), `distance` (m, NaN si inconnue), `distance_source` (NONE, BBOX, LIDAR), `time_since_seen`.

## Paramètres

`tb_bringup/config/sim.yaml` et `real.yaml` regroupent les paramètres communs (topics, `use_sim_time`, transport d'image). Aucun topic, seuil ou gain codé en dur dans les nodes (ENF-02).
