@echo off
rem Abre Windows Terminal con una pestana por proceso de la demo ROS (agente, comprobacion, nodo, angulo y RViz2).
rem Requisitos: placa programada y en marcha (Vitis o xsct), WSL2 con la distro de ROS 2 Humble (aqui Ubuntu-22.04)
rem y el agente micro-ROS compilado en ~/uros_ws. Ajusta DISTRO, REPO y la ruta del agente si las tuyas son distintas.
rem Pon los dos cuadernos (bisagra) en la posicion que quieras como 0 grados ANTES de lanzar: la pestana 3 toma ese cero.
set DISTRO=Ubuntu-22.04
set REPO=/mnt/c/workspace/Madgwick_Filter_HLS/github_ws/ros
set ROS=source /opt/ros/humble/setup.bash

wt -w 0 new-tab --title "1 Agente" wsl -d %DISTRO% -- bash -lc "%ROS% && source ~/uros_ws/install/local_setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888" ; ^
 new-tab --title "2 Topics" wsl -d %DISTRO% -- bash -lc "%ROS% && sleep 15 && ros2 topic list --no-daemon && ros2 topic hz /madgwick/q" ; ^
 new-tab --title "3 Nodo TF" wsl -d %DISTRO% -- bash -lc "%ROS% && python3 %REPO%/imu_tf.py --ros-args -p imu_a:=imu0 -p imu_b:=imu1 -p mode:=total -p tare:=true" ; ^
 new-tab --title "4 Angulo" wsl -d %DISTRO% -- bash -lc "%ROS% && sleep 20 && ros2 topic echo /hinge/angle_deg --no-daemon" ; ^
 new-tab --title "5 RViz2" wsl -d %DISTRO% -- bash -lc "%ROS% && sleep 20 && rviz2 -d %REPO%/imu_tf.rviz"
