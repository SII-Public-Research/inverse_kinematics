# Organisation du répertoire

Le répertoire est composé de 2 dossiers principaux. 

Chaque système sera représenté et étudié dans un sous-dossier du dossier ''exemples''.

## **livre** 
*Créé à partir du template [md-book](https://rust-lang.github.io/mdBook/index.html) fourni par la crate Rust du même nom.*

Vous pouvez lancer la lecture du livre via la commande suivante :
```bash
mdbook build --open
```
## **exemples** 

*Comprend l’environnement prêt à être déployé sur une raspberry pi. Nous vous invitons à vous renseigner sur l'utilisation de [ROS2](https://docs.ros.org/en/humble/index.html) avant d'utiliser cette partie.*

Le programme est divisé en deux packages :
* **command** : c'est la partie algorithmie, qui prendra en entrée un message de type *geometry_msgs/msg/Point* pour le transformer en message de type *messages/msg/MotorsAngles*.

* **motors_ctr** : c'est la partie contrôle des moteurs qui prendra en entré un message de type *messages/msg/MotorsAngles* afin de le transformer en commande moteurs.


Vous devrez dans un premier temps compiler le programme :
```bash
cd exemples/$exemple_name
colcon build
```
Une fois réalisé, il est important de sourcer les exécutables générés afin que ROS puisse les retrouver.
```bash
. install/setup.bash
```
Vous pouvez utiliser le ficher .launch qui permet de lancer les deux nodes utilisées dans le processus de contrôle du bras. 
```bash
ros2 launch $exemple_name.py
```
Enfin, vous devrez utiliser un autre terminal afin d'envoyer une commande de type *geometry_msgs/msg/Point* et ainsi contrôler le bras en position. 
```bash
ros2 topic pub --once /cmd_arm geometry_msgs/msg/Point "{x = 0.0, y = 0.0, z = 200.0}"
```

