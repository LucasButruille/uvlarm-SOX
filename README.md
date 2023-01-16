# grp-SOX repository for the UV-LARM (IMT Nord Europe)

## Description

4 packages :
	challenge1 	(contient les fichiers launch du challenge1)
	tuto_move 	(contient les fichiers python concernant la navigation)
	tuto_sim	(contient un fichier launch pour utiliser gazebo et rviz)
	tuto_vision	(contient nos premiers tests de vision)

3 fichiers launch dans le pkg challenge :
	tbot.launch.py
	simulation.launch.py
	visualize.launch.py

## Installation

On utilise ubuntu 20.04 et on suppose ros2-foxy déjà installé (`https://docs.ros.org/en/foxy/Installation.html`) avec le workspace `~/ros2_ws/` crée.

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


## Execution 

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

## Author

Mérand Julien

Butruille Lucas

Groupe ID : 4

