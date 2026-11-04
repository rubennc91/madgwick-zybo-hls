# Guía para replicar la v0.8.0 (ROS 2 nativo en la Zybo con PYNQ)

Esta guía reúne en orden todo lo necesario para repetir la demo en otro PC y con otra placa: qué comprar o tener, qué instalar, qué descargar y qué ejecutar. Los detalles y el razonamiento de cada paso están en `README.md` (esta misma carpeta).

**Qué se consigue:** la cadena Madgwick de la PL (de 1 a 4 Pmod NAV) publica sus cuaterniones como ROS 2 desde la propia placa, sin agente micro-ROS, y el PC los muestra en RViz2.

**Qué se ha probado:** 4 IMU a 952 Hz con 100 Hz por IMU (400 mensajes/s) sin lecturas incoherentes, y la demo completa de RViz2. **No se ha comprobado** el ángulo de la bisagra con esta variante ni la latencia.

## 0. Lista de la compra y de descargas

**Hardware**
* Zybo Z7-10 con su fuente de alimentación o USB, y una microSD de 8 GB o más.
* De 1 a 4 Pmod NAV (LSM9DS1) en JE, JD, JC y JA (en ese orden son `imu0`, `imu1`, `imu2`, `imu3`).
* Un cable Ethernet entre la placa y el PC (conexión directa, sin router, es lo que se probó).
* Un PC con Windows 11 (el modo de red `mirrored` de WSL2 exige Windows 11 22H2 o posterior).

**Software en el PC** (apartado 1)
* WSL2 con Ubuntu 22.04, ROS 2 Humble (`ros-humble-desktop`, que incluye RViz2 y `tf2_ros`) y `ros-humble-rmw-cyclonedds-cpp`.
* Windows Terminal (`wt`) y el cliente OpenSSH de Windows (vienen con Windows 11), `git`.
* Docker Desktop **solo** si quieres compilar tú el ROS 2 de la placa (no hace falta si usas el activo de la release).

**Descargas**
* El repositorio: `git clone https://github.com/rubennc91/madgwick-zybo-hls`.
* Los activos de la release [v0.8.0](https://github.com/rubennc91/madgwick-zybo-hls/releases/tag/v0.8.0): `ros2_min_armhf.tgz`, `SHA256SUMS.txt`, `zybo_multi.bit`, `zybo_multi.hwh`, `libfmt8_*.deb` y `libspdlog1_*.deb`.
* La imagen PYNQ 3.0.1 de 32 bits para la Zybo Z7-10: [nick-petrovsky/PYNQ-ZYBO](https://github.com/nick-petrovsky/PYNQ-ZYBO) (hay imágenes ya compiladas; para la Z7-10 no hace falta el paso del MAC que menciona).

Comprueba las descargas (en WSL2, en la carpeta con los activos): `sha256sum -c SHA256SUMS.txt` (cubre los ficheros que lista).

## 1. Preparar el PC

**WSL2 con red `mirrored`.** En PowerShell: `wsl --install -d Ubuntu-22.04`. Crea o edita `%UserProfile%\.wslconfig` con:
```
[wsl2]
networkingMode=mirrored
```
y ejecuta `wsl --shutdown` para que se aplique.

**ROS 2 Humble en Ubuntu 22.04** (dentro de WSL2). Añade el repositorio de ROS 2 siguiendo [la guía oficial de instalación](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) y luego:
```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-rmw-cyclonedds-cpp
```

**Cortafuegos (una vez).** PowerShell como administrador; hacen falta **las dos** reglas, la de Hyper-V y la de Windows, para que los paquetes de la placa lleguen a WSL2:
```
New-NetFirewallHyperVRule -Name "ROS-DDS-UDP" -DisplayName "ROS 2 DDS UDP 7400-7600" -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol UDP -LocalPorts 7400-7600 -Action Allow
New-NetFirewallRule -DisplayName "ROS 2 DDS UDP 7400-7600" -Direction Inbound -Protocol UDP -LocalPort 7400-7600 -Action Allow -Profile Any
```

**Red del PC.** Adaptador Ethernet con IP fija `192.168.2.1`, máscara `255.255.255.0`, sin puerta de enlace.

## 2. Preparar la placa

1. Graba la imagen PYNQ en la microSD (con balenaEtcher o similar) siguiendo las instrucciones del repositorio de la imagen.
2. Pon el puenteo de arranque (**JP5**) en **SD**, inserta la tarjeta y conecta el Ethernet y la alimentación.
3. Conecta los Pmod NAV a JE (`imu0`), JD, JC y JA.
4. La imagen usa la IP `192.168.2.99` pero lanza un cliente DHCP que la **borra** al no hallar servidor. Hay que fijarla; hazlo una vez por la UART (micro-USB, 115200 baudios), usuario y contraseña `xilinx`, o por SSH mientras responde al `ping`:
```bash
sudo tee /etc/network/interfaces.d/eth0 > /dev/null <<'EOT'
auto eth0
iface eth0 inet static
address 192.168.2.99
netmask 255.255.255.0
EOT
sudo reboot
```
Comprueba con `ping 192.168.2.99` desde Windows.

## 3. Copiar los ficheros a la placa
Desde PowerShell, en la carpeta donde tienes los activos de la release y los ficheros de `pynq/` del repositorio (contraseña `xilinx`):
```
ssh xilinx@192.168.2.99 "mkdir -p ~/jupyter_notebooks/zybo"
scp zybo_multi.bit zybo_multi.hwh xilinx@192.168.2.99:~/jupyter_notebooks/zybo/
scp pl_ros_node.py start_node.sh cyclone_board.xml nav_diag.py pl_read_q.ipynb xilinx@192.168.2.99:~/
scp ros2_min_armhf.tgz libfmt8*.deb libspdlog1*.deb xilinx@192.168.2.99:~/
```
Ojo con los finales de línea: `start_node.sh` debe ser texto Unix. En la placa, `file start_node.sh` no debe decir `with CRLF line terminators`; si lo dice, `sed -i 's/\r$//' start_node.sh`. Y haz el script ejecutable: `chmod +x start_node.sh`.

## 4. Instalar ROS 2 en la placa
Por SSH (`ssh xilinx@192.168.2.99`):
```bash
sudo tar xzf ~/ros2_min_armhf.tgz -C /opt        # debe quedar en /opt/ros2_min (ruta fija)
sudo dpkg -i ~/libfmt8*.deb ~/libspdlog1*.deb    # la placa no tiene Internet; por eso van los .deb
source /opt/ros2_min/setup.bash
python3 -c "import rclpy, geometry_msgs.msg; print('ok')"
```
Si falla con `libspdlog.so.1`, falta el `.deb` anterior.

## 5. Probar la PL sin ROS (recomendado)
Con las IMU quietas, abre `http://192.168.2.99:9090` (contraseña `xilinx`), abre `pl_read_q.ipynb` y ejecútalo (ajusta `BIT`, `N_IMU` y `ODR_HZ` en la primera celda). Debe mostrar los cuaterniones de cada IMU. Si una IMU no responde, en la placa:
```bash
sudo -i python3 /home/xilinx/nav_diag.py
```
Cada IMU debe dar `WHO A/G=0x68` y `Mag=0x3D`; si no, reconecta ese Pmod.

## 6. Cyclone DDS en el PC y en la placa
La placa usa Cyclone DDS (`cyclone_board.xml`, ya apunta al PC en `192.168.2.1`) y el PC también, con descubrimiento por **unicast** (el multicast no funcionó entre WSL2 y la placa). En cada terminal de WSL2:
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://<ruta absoluta>/ros/cyclone_pc.xml
```
`ParticipantIndex` en `auto` es imprescindible. No uses `ros2 daemon` en WSL2 `mirrored` (se cuelga): añade `--no-daemon` en `ros2 topic list`.

## 7. Lanzar y comprobar
En la placa (por SSH), con las IMU **quietas** los primeros ~8 s:
```bash
sudo -i /home/xilinx/start_node.sh 4 952 10.0     # n_imu, odr_hz (119 o 952), pub_hz por IMU
```
Usa `sudo -i`, no `sudo` a secas (si no, PYNQ da `No Devices Found`). Verás el periodo y el sesgo del giroscopio de cada IMU y, cada 5 s, `CPU libre … | publicados …, incoherentes 0`.

En el PC (WSL2):
```bash
ros2 topic list --no-daemon
ros2 topic hz /madgwick/q          # 4 × pub_hz (40 Hz con 4 IMU y 10)
```
Espera hasta un minuto la primera vez. El filtro tarda ~20 s en asentarse; pon el cero de la bisagra después. Límite medido: hasta 100 Hz por IMU (400 mensajes/s) sin pérdidas; por encima el nodo de Python satura en ≈540 mensajes/s.

## 8. Demo en RViz2
Con el nodo de la placa en marcha, `ros/start_demo_pynq.bat` abre Windows Terminal con las cinco pestañas (placa, topics, nodo TF, ángulo y RViz2). Antes, edita las tres variables del principio del `.bat`:
```
set DISTRO=Ubuntu-22.04
set REPO=/mnt/c/<ruta del repositorio en Windows>/ros
set BOARD=xilinx@192.168.2.99
```
Pon la bisagra en 0° y las IMU quietas antes de lanzarlo. Si prefieres hacerlo a mano en WSL2: `bash ros/demo_pc.sh`. El ángulo de la bisagra no se ha comprobado con esta variante.

## 9. Sin contraseñas (opcional)
La pestaña 1 pide la contraseña de SSH y la de `sudo`. Para quitarlas, ver el apartado 8 de `README.md` (clave SSH sin frase y `sudo` sin contraseña para `xilinx`).

## 10. Si algo falla
| Síntoma | Qué mirar |
|---|---|
| La placa responde al `ping` al arrancar y luego no | IP estática (apartado 2) |
| El PC no ve los topics | Las dos reglas de cortafuegos (apartado 1); `CYCLONEDDS_URI` exportada en esa terminal; `ParticipantIndex auto` |
| `RuntimeError: No Devices Found` | Lanzaste con `sudo` y no con `sudo -i` |
| `RuntimeError: IMUn: error de configuracion` | Esa IMU no responde; reconecta el Pmod y ejecuta `nav_diag.py` (apartado 5) |
| `ImportError: libspdlog.so.1` | Instala los `.deb` (apartado 4) |
| `bad interpreter` / `^M` al lanzar `start_node.sh` | Finales de línea CRLF (apartado 3) |
| Faltan mensajes con `pub_hz` alto | Satura el nodo de Python: usa ≤ 100 por IMU |

Más casos en la tabla «Problemas típicos» de `README.md`.

## 11. Compilar tú el ROS 2 de la placa (opcional)
Solo si no quieres usar `ros2_min_armhf.tgz`. Necesita WSL2 con Docker Desktop, unos 20 GB libres y unas 2 h 15 min en el equipo del autor. Instrucciones en `../ros2_armhf/README.md` (`JOBS=8 ./run_build.sh`).

## Licencias
El `.tgz` contiene ROS 2 Humble y Cyclone DDS compilados sin modificar su código (salvo un `package.xml`); cada paquete indica su licencia en `share/<paquete>/package.xml`.
