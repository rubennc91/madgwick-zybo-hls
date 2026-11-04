#!/usr/bin/env bash
# Compila un ROS 2 Humble minimo (rclcpp, rclpy, std_msgs, geometry_msgs, Cyclone DDS) dentro del contenedor armhf.
# Variables: JOBS (trabajos en paralelo, defecto 4), SKIP_FASTDDS (1 = no compilar Fast DDS, defecto 1),
#            EXTRA_PKGS (paquetes extra, p. ej. "ros2topic" para tener `ros2 topic` en la placa).
set -euo pipefail
JOBS="${JOBS:-4}"
SKIP_FASTDDS="${SKIP_FASTDDS:-1}"
EXTRA_PKGS="${EXTRA_PKGS:-}"
INSTALL=/opt/ros2_min
cd /ws

if [ ! -d src ]; then
  echo "== Descargando la lista de repositorios de ROS 2 Humble"
  git clone --depth 1 -b humble https://github.com/ros2/ros2 ros2_repo
  mkdir -p src
  vcs import --shallow src < ros2_repo/ros2.repos
fi

# rmw_implementation declara a Fast DDS como dependencia (build_depend y group_depend); si lo omitimos, colcon falla al
# buscar sus package.sh. Quitamos esas lineas de su package.xml (idempotente).
RI=/ws/src/ros2/rmw_implementation/rmw_implementation/package.xml
if [ "$SKIP_FASTDDS" = "1" ] && [ -f "$RI" ]; then
  sed -i -e '/rmw_fastrtps_cpp<\/build_depend>/d' -e '/rmw_fastrtps_dynamic_cpp<\/build_depend>/d' \
         -e '/<group_depend>rmw_implementation_packages<\/group_depend>/d' "$RI"
fi

SKIP=""
if [ "$SKIP_FASTDDS" = "1" ]; then
  # Fast DDS pesa mucho y no esta probado en 32 bits; usamos Cyclone DDS. Si la configuracion se queja de que
  # falta alguno de estos paquetes, ejecuta de nuevo con SKIP_FASTDDS=0.
  SKIP="--packages-skip rmw_fastrtps_cpp rmw_fastrtps_dynamic_cpp rmw_fastrtps_shared_cpp fastrtps foonathan_memory_vendor"
fi

# Paquetes que colcon arrastra solo por ser dependencias de PRUEBAS (con BUILD_TESTING=OFF no se usan) y que bajo QEMU
# son muy lentos (test_msgs genera muchisimos mensajes). Quitalos con SKIP_TESTDEPS=0 si algo falla por su ausencia.
SKIP_TESTDEPS="${SKIP_TESTDEPS:-1}"
if [ "$SKIP_TESTDEPS" = "1" ]; then
  if [ -z "$SKIP" ]; then SKIP="--packages-skip"; fi
  SKIP="$SKIP test_msgs mimick_vendor osrf_testing_tools_cpp performance_test_fixture"
fi

export MAKEFLAGS="-j${JOBS}"
echo "== colcon build (JOBS=$JOBS, SKIP_FASTDDS=$SKIP_FASTDDS, SKIP_TESTDEPS=$SKIP_TESTDEPS)"
colcon build --merge-install --install-base "$INSTALL" \
  --parallel-workers "$JOBS" \
  --packages-skip-build-finished \
  --packages-up-to rclcpp rclpy std_msgs geometry_msgs rmw_cyclonedds_cpp $EXTRA_PKGS \
  $SKIP \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
               -DCMAKE_EXE_LINKER_FLAGS=-latomic -DCMAKE_SHARED_LINKER_FLAGS=-latomic \
  --event-handlers console_cohesion+ 2>&1 | tee -a /ws/build.log

echo "== Comprobacion rapida"
set +u   # el setup.bash de ROS 2 lee variables sin definir
source "$INSTALL/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 -c "import rclpy, geometry_msgs.msg, std_msgs.msg; print('rclpy, geometry_msgs y std_msgs importan bien')"
set -u
echo "== Hecho."
