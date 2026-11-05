# Entorno comun de los scripts de medida (el mismo que start_node.sh). Se usa con "source".
source /etc/profile.d/pynq_venv.sh        # Python de PYNQ
source /opt/ros2_min/setup.bash           # ROS 2 compilado para la placa
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/xilinx/cyclone_board.xml
