#!/bin/bash
# Lanza el nodo ROS 2 de la placa (PYNQ). Ejecutar como root, con las IMU QUIETAS los primeros ~8 s:
#   sudo -i /home/xilinx/start_node.sh [n_imu] [odr_hz] [pub_hz]   (por defecto: 2 119 10.0)
# IMPORTANTE: con "sudo -i" (sesion de login de root); con "sudo" a secas PYNQ no encuentra la PL (No Devices Found).
# BIT = ruta del .bit (el .hwh debe estar al lado, con el mismo nombre base).
BIT="${BIT:-/home/xilinx/jupyter_notebooks/zybo/zybo_multi.bit}"
N="${1:-2}"; ODR="${2:-119}"; HZ="${3:-10.0}"
source /etc/profile.d/pynq_venv.sh        # Python de PYNQ (entorno virtual con el modulo pynq)
source /opt/ros2_min/setup.bash           # ROS 2 compilado para la placa
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/xilinx/cyclone_board.xml
exec python3 /home/xilinx/pl_ros_node.py --ros-args -p bit:="$BIT" -p n_imu:="$N" -p odr_hz:="$ODR" -p pub_hz:="$HZ"
