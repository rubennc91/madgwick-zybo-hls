@echo off
rem Demo en RViz2 con la Zybo + PYNQ (ROS 2 nativo en la placa, sin agente micro-ROS).
rem Abre Windows Terminal con una pestana por proceso: nodo de la placa (por SSH), comprobacion, nodo TF, angulo y RViz2.
rem Requisitos: ver pynq/README.md (IP 192.168.2.1 en Windows y 192.168.2.99 en la placa, cortafuegos, Cyclone DDS en el PC
rem y start_node.sh + pl_ros_node.py + cyclone_board.xml + zybo_multi.bit/.hwh ya copiados a la placa).
rem Pon la bisagra en 0 grados y las IMU QUIETAS antes de lanzar. La pestana 1 pide la contrasena de SSH (xilinx) y la de sudo.
set DISTRO=Ubuntu-22.04
set REPO=/mnt/c/workspace/Madgwick_Filter_HLS/github_ws/ros
set BOARD=xilinx@192.168.2.99
set ROS=source /opt/ros/humble/setup.bash ^&^& export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ^&^& export CYCLONEDDS_URI=file://%REPO%/cyclone_pc.xml

wt -w 0 new-tab --title "1 Placa" ssh -t %BOARD% "sudo /home/xilinx/start_node.sh" ; ^
 new-tab --title "2 Topics" wsl -d %DISTRO% -- bash -lc "%ROS% && sleep 25 && ros2 topic list --no-daemon && ros2 topic hz /madgwick/q" ; ^
 new-tab --title "3 Nodo TF" wsl -d %DISTRO% -- bash -lc "%ROS% && sleep 40 && python3 %REPO%/imu_tf.py --ros-args -p imu_a:=imu0 -p imu_b:=imu1 -p mode:=total -p tare:=true" ; ^
 new-tab --title "4 Angulo" wsl -d %DISTRO% -- bash -lc "%ROS% && sleep 45 && ros2 topic echo /hinge/angle_deg --no-daemon" ; ^
 new-tab --title "5 RViz2" wsl -d %DISTRO% -- bash -lc "%ROS% && sleep 45 && rviz2 -d %REPO%/imu_tf.rviz"
