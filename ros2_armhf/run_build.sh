#!/usr/bin/env bash
# Ejecutar en Ubuntu (WSL2) con Docker Desktop. Trabaja en volumenes de Docker (no en /mnt/c) por velocidad.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p out
echo "== 1) Emulacion ARM (QEMU/binfmt)"
docker run --privileged --rm tonistiigi/binfmt --install arm
echo "== 2) Imagen armhf con las dependencias"
docker build --platform linux/arm/v7 -t ros2-armhf-min .
echo "== 3) Compilacion (lenta; si se corta, vuelve a ejecutar este script: continua donde se quedo)"
docker run --platform linux/arm/v7 --rm -it \
  -v ros2ws:/ws -v ros2inst:/opt/ros2_min \
  -e JOBS="${JOBS:-4}" -e SKIP_FASTDDS="${SKIP_FASTDDS:-1}" -e EXTRA_PKGS="${EXTRA_PKGS:-}" \
  ros2-armhf-min /build/build_ros2_min.sh
echo "== 4) Empaquetar"
docker run --platform linux/arm/v7 --rm \
  -v ros2inst:/opt/ros2_min -v "$PWD/out":/out \
  ros2-armhf-min tar czf /out/ros2_min_armhf.tgz -C /opt ros2_min
( cd out && sha256sum ros2_min_armhf.tgz > SHA256SUMS.txt && cat SHA256SUMS.txt )
ls -lh out/ros2_min_armhf.tgz
echo "== 5) Paquetes .deb que la placa necesita y PYNQ no trae (la placa no tiene Internet)"
mkdir -p out/debs
docker run --platform linux/arm/v7 --rm -v "$PWD/out/debs":/debs ros2-armhf-min \
  bash -c "apt-get update -qq && cd /debs && apt-get download libspdlog1 libfmt8"
ls out/debs
echo "Siguiente: scp out/ros2_min_armhf.tgz out/debs/*.deb xilinx@192.168.2.99:~/   (ver README.md)"
