# ROS 2 Humble mínimo para ARM de 32 bits (Zybo con PYNQ)

Compila en el PC, con emulación QEMU dentro de Docker, un ROS 2 Humble reducido (`rclcpp`, `rclpy`, `std_msgs`, `geometry_msgs` y Cyclone DDS como RMW) para el Cortex-A9 (armhf, Ubuntu 22.04, Python 3.10). Los binarios ya compilados están en los activos de la release (`ros2_min_armhf.tgz` y su `SHA256SUMS.txt`); esta receta sirve para repetirlos, cambiar paquetes o actualizar. Cómo usarlos en la placa: `../pynq/README.md`.

ROS 2 Humble no soporta oficialmente armhf; esta receta se ejecutó una vez y dio un árbol con el que `import rclpy`, `geometry_msgs` y `std_msgs` funcionan en la placa. No se ha probado con otros paquetes (sin `ros2cli`, por ejemplo: en la placa no hay `ros2 topic`).

| Fichero | Para qué |
|---|---|
| `Dockerfile` | Imagen `linux/arm/v7` (Ubuntu 22.04) con las dependencias de compilación; `colcon` y `vcstool` por `pip` |
| `build_ros2_min.sh` | Descarga `ros2.repos` (rama `humble`), importa las fuentes y ejecuta `colcon build --packages-up-to` con los ajustes de abajo |
| `run_build.sh` | Orquesta todo en el PC: emulación, imagen, compilación, empaquetado, `SHA256SUMS.txt` y descarga de los `.deb` que la placa necesita |

## Requisitos
Ubuntu 22.04 en WSL2 con Docker Desktop (integración con WSL activada), unos 20 GB libres y el máximo de núcleos que puedas dar a WSL2. Trabaja en el disco de Linux (`~`), no en `/mnt/c`: copia la carpeta con `cp -r` y, si los scripts traen saltos de línea de Windows, `sed -i 's/\r$//' *.sh Dockerfile`.

## Uso
```bash
cd ~/ros2_armhf
JOBS=8 ./run_build.sh
```
Resultado en `out/`: `ros2_min_armhf.tgz`, `SHA256SUMS.txt` y `debs/` (`libspdlog1` y `libfmt8`). Si se corta, vuelve a lanzarlo: `colcon` salta lo ya compilado (`--packages-skip-build-finished`) porque el árbol vive en los volúmenes de Docker `ros2ws` e `ros2inst`. No borres esos volúmenes (`docker volume rm`, `docker system prune --volumes`) hasta tener el `.tgz`.

Variables: `JOBS` (trabajos en paralelo, 4 por defecto; si la emulación falla por memoria, bájalo), `SKIP_FASTDDS` (1 por defecto), `SKIP_TESTDEPS` (1), `EXTRA_PKGS` (p. ej. `ros2topic`).

Tiempo en el PC del autor: unas 2 h 15 min en total según los resúmenes de `colcon` (114 paquetes en 1 h 44 min, más 8 paquetes en 29 min tras corregir un fallo).

## Ajustes que hicieron falta (y por qué)
* `-DBUILD_TESTING=OFF` no evita que `colcon --packages-up-to` compile las dependencias de **pruebas**. `test_msgs` tardó 39 min bajo QEMU; se omiten con `--packages-skip test_msgs mimick_vendor osrf_testing_tools_cpp performance_test_fixture` (`SKIP_TESTDEPS=1`). Si algún paquete falla por su ausencia, `SKIP_TESTDEPS=0`.
* Fast DDS (`fastrtps`, `foonathan_memory_vendor`, `rmw_fastrtps_*`) se omite (`SKIP_FASTDDS=1`): pesa mucho y no se probó en 32 bits; el RMW es Cyclone DDS. Pero `rmw_implementation` los declara como dependencia (dos `build_depend` y un `group_depend`) y `colcon` abortaba al no encontrar sus `package.sh`. `build_ros2_min.sh` borra esas tres líneas de su `package.xml` antes de compilar. Las bibliotecas de tipos de Fast DDS para los mensajes (`rosidl_typesupport_fastrtps_*`, `fastcdr`) sí se compilan.
* `-latomic` en los enlazadores, por si algún paquete usa operaciones atómicas de 64 bits en 32 bits (se añadió como precaución; no se comprobó si hacía falta).
* Prefijo de instalación `/opt/ros2_min`: los `setup.bash` de `colcon` llevan la ruta de compilación, así que en la placa el árbol debe quedar en `/opt/ros2_min`.
* La comprobación final desactiva `set -u`: el `setup.bash` de ROS 2 lee `COLCON_TRACE` sin definir.

## En la placa
Con `libspdlog1` y `libfmt8` (de `out/debs/`) el resto de bibliotecas de `rclcpp`, `rcl` y `rclpy` se resuelve con PYNQ 3.0.1; la comprobación con `ldd` está en `../pynq/README.md`.

## Licencias
El `.tgz` incluye ROS 2 Humble y Cyclone DDS compilados sin modificar su código (salvo la línea de `package.xml` de arriba); cada paquete indica su licencia en `share/<paquete>/package.xml`. Revisa esas licencias antes de redistribuirlo fuera de este repositorio.
