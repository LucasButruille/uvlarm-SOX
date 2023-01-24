# grp-SOX repository for the UV-LARM (IMT Nord Europe)

# Description
Ce répertoire contient :

4 packages :

1. challenge1 	(les fichiers launch du challenge1)
2. challenge2   (les fichiers launch du challenge2)
2. tuto_move 	(les fichiers python concernant la navigation)
3. tuto_sim		(un fichier launch pour utiliser gazebo et rviz)
4. tuto_vision	(nos premiers tests de vision)

# Installation

On utilise ubuntu 20.04 et on suppose ros2-foxy déjà installé (https://docs.ros.org/en/foxy/Installation.html) avec le workspace `~/ros2_ws/` crée.

**Etape 1 :** 

Clonez les packages suivants :

```
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws/

git clone https://bitbucket.org/imt-mobisyst/mb6-tbot.git

git clone https://github.com/LucasButruille/uvlarm-SOX.git
```

**Etape 2 :** 

Renommez `mb6-tbot` en `pkg-tbot`.

**Etape 3 :** 

Build et Sourcer le workspace :
```
cd ~/ros2_ws/
colcon build
. install/setup.sh
```


# Execution 

## Challenge 1 :

Pour lancer le robot avec le laser, exécutez la commande ci-dessous : 
```
ros2 launch challenge1 tbot.launch.py
```

Pour lancer la simulation :
```
ros2 launch challenge1 simulation.launch.py
```

Pour visualiser les données du laser sur rviz2 :
```
ros2 launch challenge1 visualize.launch.py
```

## Challenge 2 :

Exécuter le fichier launch suivant :
```
ros2 launch challenge2 challenge2.launch.py
```
Cette commande permet de lancer rviz2, slam_toolbox, nav2 et le robot avec le laser et la caméra.

### Robot :
Un appui sur B0 permet de lancer le robot en complète autonomie et de détecter les bouteilles.

Un appui sur B1 permet d'atteindre un goal pose envoyé à partir de rviz2.

Un appui sur B0 permet de mettre le robot en pause.

# Author

Mérand Julien

Butruille Lucas

Groupe ID : 4

