# Une piste du tracker SORT (une personne suivie au fil des images).

uint32 id                          # identifiant stable de la personne (EF-03)
vision_msgs/BoundingBox2D bbox     # bbox estimee par le filtre de Kalman, en pixels
float32 score                      # confiance de la derniere detection associee
float32 vx                         # vitesse du centre de la bbox, pixels/s
float32 vy
uint32 age                         # nombre d'images depuis la creation de la piste
uint32 frames_since_update         # images sans detection associee (> 0 : prediction seule, EF-04)
bool confirmed                     # piste confirmee (assez de detections successives)
string color                       # couleur dominante des vetements (haut du corps), ex. "rouge" ; vide si inconnue
