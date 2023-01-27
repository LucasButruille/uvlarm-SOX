# grp-SOX repository for the UV-LARM (IMT Nord Europe)

## Description

Grâce au code et manipulation ci-dessous, il est possible de faire déplacer le robot ```Kobuki turtleBot3``` de façon autonome, en évitant les obstacles. De plus, il est capable de cartographier l'espace qui l'entoure et de repérer les bouteilles oranges de NukaCola. Le robot place ces dernières sur une carte créée par ```rviz2``` à l'aide d'un petit marqueur cylindrique orange.

Ce répertoire contient :

6 packages :

1. challenge1 	(les fichiers launch du challenge 1)
2. challenge2   (les fichiers launch du challenge 2)
3. challenge3   (les fichiers launch du challenge 3)
4. tuto_move 	(les noeuds python concernant la navigation)
5. tuto_sim		(un fichier launch pour utiliser gazebo et rviz)
6. tuto_vision	(nos premiers tests de vision)
7. tuto_goalpose (les fichiers pythons permettant la localisation des bouteilles)

Les packages ```tuto``` contiennent les différents noeuds utilisés dans les challenges.

## Installation

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

**Etape 4 :**

Connectez vous en ssh sur le pc connecté au robot :
```
ssh name@xx.xx.xx.xx
```

## Exécution 

**Robot :**

Un appui sur B0 permet de lancer le robot en complète autonomie et de détecter les bouteilles.

Un appui sur B1 permet d'atteindre un goal pose envoyé à partir de rviz2.

Un appui sur B0 permet de mettre le robot en pause.

Si le robot est soulevé ou que le bumper est activé, le robot se met en pause.

**Vision :**

Le traitement pour détecter les bouteilles se fait via un filtre de couleur en HSV et calcul le rapport hauteur/largeur pour s'assurer que l'objet est bien une bouteille.

Pour visualiser la caméra seulement, exécutez le noeud suivant :
```
ros2 run tuto_vision vision
```

**Challenge 1 :**

Dans ce challenge le robot se déplace dans un environnement fermé en évitant tous les obstacles.

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

**Challenge 2 :**

Dans ce challenge, le robot est capable de se déplacer de manière autonome dans une zone fermée en évitant les obstacles. De plus, il est capable de construire une carte et de détecter des bouteilles dans son environnement.

Il est possible de mettre des ```goal_pose``` sur rviz2 pour que le robot s'y dirige.

Pour lancer ce challenge, exécutez le fichier launch suivant sur le pc connecté au robot:
```
ros2 launch challenge2 challenge2.launch.py
```
Cette commande permet de lancer nav2 et le robot avec le laser et la caméra.

Pour visualiser la carte sur rviz2 :
```
ros2 launch challenge2 visualize.launch.py
```

Pour la simulation :
```
ros2 launch challenge2 simulation.launch.py
```

**Challenge 3 :**

Ce challenge reprend les éléments du challenge 2 et ajoute les bouteilles sur la carte sous forme de marqueurs. Aussi, il est capable d'identifier les bouteilles pour ne pas les marquer plusieurs fois.

Exécutez le fichier launch suivant sur le pc connecté au robot :
```
ros2 launch challenge3 challenge3.launch.py
```

Pour visualiser la carte :
```
ros2 launch challenge3 visualize.launch.py
```

Pour la simulation :
```
ros2 launch challenge3 simulation.launch.py
```


## Développeurs

Mérand Julien / Butruille Lucas (CI2/M1 - IMT Nord Europe)

Groupe SOX

