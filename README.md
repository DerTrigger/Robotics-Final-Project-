# Robotics-Final-Project-

Instrucciones para correr el programa

    Crear un workspace mkdir finalproject

    Crear un paquete catkin_create_pkg followrosbag 

    Dentro del paquete se tiene que crear el folder "src", dicho folder ya se encuentra en el GitHub, una vez descargado, se tiene que copiar

    Dentro del directorio del paquete ya con la carpeta "src" correr: catkin_make

    Ejecutar el setup.bash source devel/setup.bash

    Una vez compilado el proyecto, se notara que tenemos dos ejecutables: "estimated_pose.cpp" y "control_kalman.cpp"

    La parte 1 del proyecto que consiste en la estimación del estado por medio de un filtro de histogramas se corre con el primer ejecutable anteriormente mencionado.
    Para ejecutarlo:
    1) En la carpeta de su workspace correr el comando y ya con el nodo maestro corriendo (roscore) ejecute: rosrun followrosbag estimated_pose
    2) En un nueva terminal hacer echo del nodo "/states_node" con el comando: rostopic echo /states_node el cual envía las probabilidades del siguiente estado del autonomous

    La parte 2 y 3 del proyecto que implementa el filtro de kalman y el control del robot mini_1 corresponde al ejecutable "control_kalman.cpp"
	Para ejecutarlo: 
	1) En la carpeta de su workspace correr el comando y ya con el nodo maestro corriendo (roscore) ejecute: rosrun followrosbag control_kalman
	Al correrlo se observa la pose estimada y la pose medida por el sensor LiDAR.
	2) Se le puede dar una velocidad linear y angular al mini_2 al ejecutar: 
	rostopic pub /AutoNOMOS_mini_2/manual_control/velocity std_msgs/Float32 '{data: 2}'
	rostopic pub /AutoNOMOS_mini_2/manual_control/steering std_msgs/Float32 '{data: 10}'

	3) Posteriormente se puede hacer un echo de dichos topicos pero del mini_1 para observar como se modifica su velocidad de acuerdo al control implementado "move to point"
