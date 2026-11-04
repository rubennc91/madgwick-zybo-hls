# ROS 2 Humble minimo para ARM de 32 bits (armhf / ARMv7), el de la Zybo con PYNQ (Ubuntu 22.04).
# Se compila en el PC con emulacion QEMU y el resultado (/opt/ros2_min) se copia a la placa.
FROM --platform=linux/arm/v7 ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git curl ca-certificates \
      python3 python3-dev python3-pip python3-setuptools python3-wheel \
      python3-numpy python3-yaml python3-lark python3-empy python3-packaging python3-pytest \
      libssl-dev libtinyxml2-dev libyaml-dev libspdlog-dev libacl1-dev pybind11-dev \
    && rm -rf /var/lib/apt/lists/*
# colcon y vcstool son Python puro: se instalan con pip
RUN python3 -m pip install --no-cache-dir -U colcon-common-extensions vcstool
COPY build_ros2_min.sh /build/build_ros2_min.sh
RUN chmod +x /build/build_ros2_min.sh
WORKDIR /ws
