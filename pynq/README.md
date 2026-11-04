# ROS 2 nativo en la Zybo Z7-10 con PYNQ (sin micro-ROS)

El Cortex-A9 ejecuta Linux (imagen PYNQ 3.0.1 de 32 bits, Ubuntu 22.04) y un nodo `rclpy` lee por AXI-Lite los cuaterniones de la cadena en la PL (el mismo bitstream `zybo_multi` de `Integration_Zybo_multi`, sin cambios) y los publica como ROS 2 por Ethernet: `/madgwick/q` (`geometry_msgs/QuaternionStamped`, `frame_id` `imu0..`) y `/madgwick/cpu_free` (`std_msgs/Float32`). No hay agente: el PC ve los topics por DDS (Cyclone DDS).

| Fichero | Para qué |
|---|---|
| `pl_read_q.ipynb` | Cuaderno de Jupyter: carga el overlay, configura la cadena (equivale a `main_pl.c`) y muestra los cuaterniones; sirve para comprobar el acceso a la PL antes de usar ROS |
| `pl_ros_node.py` | Nodo ROS 2: arranca la cadena y publica. Parámetros: `bit`, `n_imu` (2), `odr_hz` (119 o 952), `pub_hz` (10.0, por IMU), `stat_s` (5.0) |
| `start_node.sh` | Lanza el nodo con todo el entorno (venv de PYNQ, ROS 2, Cyclone DDS). Uso: `start_node.sh [n_imu] [odr_hz] [pub_hz]` (por defecto `2 119 10.0`) |
| `nav_diag.py` | Diagnóstico de las 4 IMU: arranca cada `nav_spi_ctrl` por separado y muestra `STATUS` y `WHO_AM_I` (solo lee) |
| `cyclone_board.xml` | Configuración de Cyclone DDS de la placa (descubrimiento por unicast hacia el PC, 192.168.2.1) |

**¿Quieres repetirlo en otro PC y otra placa?** Sigue `GUIA_REPLICAR.md`, que ordena todos los pasos (qué instalar y qué descargar). Este fichero es la referencia detallada.

La receta para compilar ROS 2 para la placa está en `../ros2_armhf/`; los binarios ya compilados están en los activos de la release.

## Estado de lo probado
* **2 IMU, 119 Hz, `pub_hz` 10**: el nodo arranca en el Python de PYNQ junto a `rclpy`; la placa publica 100 mensajes cada 5 s sin lecturas incoherentes y con ≈94,8 % de CPU libre (medida con `/proc/stat` de la placa; no es la misma medida que el bucle de reposo de FreeRTOS de v0.7.0, así que no es comparable con esa tabla). El PC recibe 20 Hz.
* **4 IMU, 952 Hz** (`start_node.sh 4 952 <pub_hz>`; periodos medidos 943,8 / 974,2 / 963,0 / 992,9 Hz), 0 lecturas incoherentes en todas las pruebas y el PC recibe lo que publica la placa:

| `pub_hz` por IMU | msg/s pedidos | publicados | recibidos en el PC | CPU libre |
|---|---|---|---|---|
| 10 | 40 | 40 | 40,0 | ≈92 % |
| 40 | 160 | 160 | 160,0 | ≈75 % |
| 100 | 400 | 400 | 399,5-399,8 | ≈55 % |
| 150 | 600 | ≈540-565 | ≈532-554 | ≈45 % |
| 200 | 800 | ≈540-570 | ≈545-571 | ≈46 % |

  Hasta 100 Hz por IMU el nodo publica lo pedido. Por encima satura en ≈540 mensajes/s: `top` muestra `python3` al 100 % de un núcleo, así que el límite es el hilo de Python (no la PL ni la red). `ros2 topic hz` ve las cuatro IMU de golpe en cada periodo de publicación (ráfagas de 4).
* **Demo completa** con `../ros/start_demo_pynq.bat` (nodo de la placa por SSH, topics, nodo TF, ángulo y RViz2): ejecutada y funciona. El **ángulo de la bisagra no se ha comprobado** con esta variante (no había montaje).
* **Sin medir**: la latencia de la muestra al mensaje y la tabla comparativa con micro-ROS.

## Requisitos
* Zybo Z7-10 con una imagen PYNQ 3.0.1 de 32 bits: [nick-petrovsky/PYNQ-ZYBO](https://github.com/nick-petrovsky/PYNQ-ZYBO) (hay imágenes ya compiladas para la Zybo, la Z7-10 y la Z7-20; para la Z7-10 no hace falta el paso del MAC). La placa no tiene Internet en esta configuración (cable directo al PC).
* El bitstream y su `.hwh` de `Integration_Zybo_multi` (`zybo_multi.bit` y `zybo_multi.hwh`, mismo nombre base; se adjuntan a la release y salen de `build_project.tcl` en `zybo_multi/zybo_multi.runs/impl_1/` y `.../hw_handoff/`).
* PC con Windows 11, WSL2 (red `mirrored`) con Ubuntu 22.04 y ROS 2 Humble, y Docker Desktop si vas a compilar tú el ROS 2 de la placa.

## 1. Red entre el PC y la placa
* Windows: adaptador Ethernet con IP fija `192.168.2.1`, máscara `255.255.255.0`, sin puerta de enlace.
* La imagen PYNQ usa la IP `192.168.2.99`, pero por defecto lanza un cliente DHCP que, al no encontrar servidor, **borra la dirección** y la placa deja de responder tras arrancar. Fija la IP en la placa (por la UART, usuario y contraseña `xilinx`):

```bash
sudo cp /etc/network/interfaces.d/eth0 ~/eth0.bak
sudo tee /etc/network/interfaces.d/eth0 > /dev/null <<'EOT'
auto eth0
iface eth0 inet static
address 192.168.2.99
netmask 255.255.255.0
EOT
sudo reboot
```
Para volver al DHCP: `sudo cp ~/eth0.bak /etc/network/interfaces.d/eth0`. Comprueba con `ping 192.168.2.99` desde Windows.

## 2. Copiar los ficheros a la placa
Desde PowerShell, en la carpeta con los ficheros (contraseña `xilinx`):

```
ssh xilinx@192.168.2.99 "mkdir -p ~/jupyter_notebooks/zybo"
scp zybo_multi.bit zybo_multi.hwh xilinx@192.168.2.99:~/jupyter_notebooks/zybo/
scp pl_ros_node.py start_node.sh cyclone_board.xml pl_read_q.ipynb xilinx@192.168.2.99:~/
scp ros2_min_armhf.tgz libfmt8*.deb libspdlog1*.deb xilinx@192.168.2.99:~/
```
`ros2_min_armhf.tgz` es el activo de la release (o `out/` de `ros2_armhf/run_build.sh`). Los `.deb` también salen de `run_build.sh` (`out/debs/`).

## 3. Probar el acceso a la PL (opcional, sin ROS)
Abre `http://192.168.2.99:9090` (contraseña `xilinx`), abre `pl_read_q.ipynb` y ejecútalo con las IMU quietas. Ajusta `BIT`, `N_IMU` y `ODR_HZ` en la primera celda. En una prueba con 2 IMU a 952 Hz midió ≈944 y ≈974 Hz, 0 tramas perdidas en 60 s y `|q|` = 1,0000.

## 4. Instalar ROS 2 en la placa
```bash
sudo tar xzf ~/ros2_min_armhf.tgz -C /opt          # debe quedar en /opt/ros2_min (ruta fija de la compilación)
sudo dpkg -i ~/libfmt8*.deb ~/libspdlog1*.deb      # librería que PYNQ no trae
source /opt/ros2_min/setup.bash
for f in /opt/ros2_min/lib/*.so /opt/ros2_min/local/lib/python3.10/dist-packages/*/*.so; do ldd "$f" 2>/dev/null | grep "not found"; done | sort -u
python3 -c "import rclpy, geometry_msgs.msg; print('ok')"
```
La comprobación de `ldd` no debe imprimir nada. Si `apt` no encuentra paquetes es porque la placa no tiene Internet; por eso se instalan los `.deb`.

## 5. Cortafuegos del PC (una vez)
Para que los paquetes de la placa lleguen a WSL2 (red `mirrored`) hacen falta **las dos** reglas de entrada, la de Hyper-V y la de Windows (PowerShell como administrador). Sin la de Hyper-V no llega nada: en las pruebas, `tcpdump` en WSL2 no veía ningún paquete de la placa hasta crearlas.

```
New-NetFirewallHyperVRule -Name "ROS-DDS-UDP" -DisplayName "ROS 2 DDS UDP 7400-7600" -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol UDP -LocalPorts 7400-7600 -Action Allow
New-NetFirewallRule -DisplayName "ROS 2 DDS UDP 7400-7600" -Direction Inbound -Protocol UDP -LocalPort 7400-7600 -Action Allow -Profile Any
```

## 6. Cyclone DDS en el PC
Humble usa Fast DDS por defecto; la placa usa Cyclone, así que el PC también (`sudo apt install ros-humble-rmw-cyclonedds-cpp`). El descubrimiento por multicast no funcionó entre WSL2 y la placa; con unicast sí. `../ros/cyclone_pc.xml` es el equivalente para el PC. En cada terminal de WSL2 que uses con ROS:

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://<ruta>/cyclone_pc.xml      # ruta absoluta de WSL
```
`ParticipantIndex` en `auto` es imprescindible: sin él Cyclone no abre los puertos de descubrimiento por unicast (7410 y siguientes) y el PC y la placa no se ven. No uses `ros2 daemon` con WSL2 en modo `mirrored` (se cuelga): `--no-daemon`.

## 7. Lanzar
En la placa (por SSH), con las IMU quietas los primeros ~8 s:

```bash
sudo -i /home/xilinx/start_node.sh                 # n_imu=2, odr_hz=119, pub_hz=10
sudo -i /home/xilinx/start_node.sh 4 952 10.0      # 4 IMU a 952 Hz, 10 Hz por IMU (el ODR solo admite 119 o 952)
```
Usa siempre `sudo -i`, no `sudo` a secas. Con 4 IMU, `ros2 topic hz /madgwick/q` da 4 × `pub_hz` (40 Hz con 10).
Verás el periodo y el sesgo del giroscopio de cada IMU y, cada 5 s, `CPU libre … | publicados …, incoherentes …`. En el PC:

```bash
ros2 topic list --no-daemon
ros2 topic hz /madgwick/q                  # ≈ 20 Hz con 2 IMU y pub_hz 10; 40 Hz con 4 IMU
```
Espera hasta un minuto la primera vez: el descubrimiento no es instantáneo. El filtro tarda unos 20 s en asentarse tras el arranque (en las pruebas, una IMU giró unos 3° en ese tiempo); pon el cero de la bisagra después.

## 8. Lanzar sin contraseñas (opcional)
Sin esto, la pestaña 1 de `start_demo_pynq.bat` pide la contraseña de SSH y la de `sudo`. Se quitan por separado, una sola vez.

**Clave SSH** (PowerShell en Windows):
```
ssh-keygen -t ed25519
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh xilinx@192.168.2.99 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"
```
Pulsa Intro en todas las preguntas de `ssh-keygen` para dejar la clave **sin frase**: con frase, SSH la pide cada vez (o hay que cargarla con `ssh-add`).

**`sudo` sin contraseña** (en la placa):
```bash
echo 'xilinx ALL=(ALL) NOPASSWD: ALL' | sudo tee /etc/sudoers.d/start_node
sudo chmod 440 /etc/sudoers.d/start_node
sudo visudo -c                       # debe decir "parsed OK"
sudo -k                              # borra la caché de sudo antes de probar
```
Prueba desde Windows (con `-n` falla en vez de preguntar):
```
ssh -t xilinx@192.168.2.99 "sudo -n -i /home/xilinx/start_node.sh 4 952 10.0"
```
Hay que dar permiso a **todo** y no solo al script: con `sudo -i` sudo comprueba la shell que lanza el script, no el script, y una regla limitada a `start_node.sh` no encaja (se probó). Eso deja a `xilinx` con `root` sin contraseña. Es razonable en una placa con cable directo al PC y sin Internet; no lo dejes así en una red compartida (`sudo rm /etc/sudoers.d/start_node` lo quita). `visudo -c` antes de salir evita un `sudoers` roto.

## Problemas típicos
| Síntoma | Causa y arreglo |
|---|---|
| La placa responde al `ping` mientras arranca y luego no | Cliente DHCP que borra la IP; fija la IP estática (apartado 1) |
| `apt` no encuentra paquetes | Sin Internet: usa los `.deb` descargados con `ros2_armhf/run_build.sh` |
| `ImportError: libspdlog.so.1` | Falta `libspdlog1` (apartado 4) |
| El PC no ve los topics, `tcpdump` no ve paquetes de la placa | Reglas de cortafuegos (apartado 5) |
| Idem, pero `tcpdump` sí ve tráfico | Falta `ParticipantIndex auto` o `CYCLONEDDS_URI` en esa terminal (apartado 6); la variable hay que exportarla en cada terminal |
| `RuntimeError: No Devices Found` (`is the XRT environment sourced?`) al lanzar `start_node.sh` | Se lanzó con `sudo` a secas, que no carga el entorno de `root`: usa `sudo -i /home/xilinx/start_node.sh` |
| `RuntimeError: IMUn: error de configuracion (STATUS=0x…)` al arrancar | Esa IMU no responde: `STATUS=0x3a` es `WHO_AM_I` del acelerómetro/giroscopio sin respuesta con el magnetómetro detectado. Reconecta el Pmod y ejecuta `sudo -i python3 /home/xilinx/nav_diag.py`; debe dar `WHO A/G=0x68` y `Mag=0x3D` en cada IMU. Una vez desapareció tras reiniciar la placa |
| La pestaña 1 pide la contraseña de SSH o de `sudo` | Apartado 8: clave SSH sin frase y `sudo` sin contraseña. Si pide la frase de la clave, quítala con `ssh-keygen -p -f $env:USERPROFILE\.ssh\id_ed25519` |
| `sudo -n … a password is required` aunque creaste la regla | La regla limitada al script no vale con `sudo -i`; usa `NOPASSWD: ALL` (apartado 8). Comprueba con `sudo -k` antes: la caché de sudo da falsos positivos |
| Con `pub_hz` alto llegan menos mensajes de los pedidos | El nodo de Python satura en ≈540 mensajes/s en total (≈135 Hz por IMU con 4); usa `pub_hz` ≤ 100 |
| `Permission denied` al escribir `cyclone_board.xml` | Lo creó `root`; escribe como `root` o con `sudo tee` |
| `ros2 daemon stop` se queda colgado | Con WSL2 `mirrored` el daemon no responde; no lo uses |
| `ros2 topic hz … --no-daemon`: argumento no reconocido | Ese flag solo existe en `ros2 topic list` |
