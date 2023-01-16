# grp-SOX repository for the UV-LARM

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

## Execution 

Pour lancer le robot avec le laser, exécutez la commande ci-dessous :
	ros2 launch challenge1 tbot.launch.py

Pour lancer la simulation : 
	ros2 launch challenge1 simulation.launch.py

Pour visualiser les données du laser sur rviz2 :
	ros2 launch challenge1 visualize.launch.py

## Author

Mérand Julien
Butruille Lucas
Groupe ID : 4

