# micro-ROS sobre FreeRTOS en el Cortex-A9 (Zybo Z7-10)

Guía para repetir las medidas de la v0.7.0: el Cortex-A9 ejecuta FreeRTOS y un cliente micro-ROS que publica los cuaterniones de la cadena en la PL (bitstream de `Integration_Zybo_multi`, sin cambios) por UDP sobre el Ethernet del PS. El agente corre en un PC con ROS 2 Humble (aquí, WSL2 en Windows 11). No es Linux ni ROS 2 completo en la placa.

Probado con: Vitis 2023.2 (modo clásico), Windows 11 Pro, WSL2 con Ubuntu 22.04 y red `mirrored`, Docker Desktop. Los comandos de Linux van en la terminal de Ubuntu; los de `usbipd`, cortafuegos y `.wslconfig`, en Windows.

## 1. Biblioteca estática de micro-ROS para el Cortex-A9

```bash
mkdir -p ~/uros_zynq && cd ~/uros_zynq
git clone -b humble https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git
cd micro_ros_stm32cubemx_utils/microros_static_library/library_generation
cp /ruta/al/repo/microros/zynq_toolchain.cmake toolchain.cmake       # sustituye al toolchain de STM32
# colcon.meta: se dejó el original del repositorio (transporte personalizado, framing y MAX_* por defecto), sin cambios
cd ~/uros_zynq
docker run -it --rm -v "$(pwd)":/project \
  --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library \
  microros/micro_ros_static_library_builder:humble
```

* Tarda 10-20 min. Cuando pregunta por los `CFLAGS` (no hay Makefile de STM32) se contesta `y`.
* Resultado: `libmicroros/libmicroros.a` (≈ 10 MB) y `libmicroros/microros_include/`. Los ficheros quedan con propietario root (Docker).
* Los avisos de `stderr` (`-frtti` en C, `setenv` implícito) no son errores.
* Si falla `tracetools` por `typeid`/`-fno-rtti`, el toolchain lleva `-fno-rtti`: hay que quitarlo.

## 2. Plataforma y aplicación en Vitis (modo clásico: `vitis -classic`)

El Vitis 2023.2 nuevo falla al generar el BSP de FreeRTOS para `ps7_cortexa9` (comprobación de TTC).

1. Plataforma desde el `.xsa` de `Integration_Zybo_multi`, dominio `freertos10_xilinx` en `ps7_cortexa9_0`. En el BSP: `use_idle_hook = true` y **`total_heap_size = 1048576`** (el montón de FreeRTOS está en el BSP, no en `lscript.ld`; con el valor por defecto sale `vApplicationMallocFailedHook`). Para `main_microros_udp.c` añade la biblioteca **lwip213** (`api_mode = SOCKET_API`, como la plantilla *lwIP Echo Server*).
2. Aplicación vacía con `Integration_Zybo_multi/vitis_freertos/main_microros_udp.c` (o `main_freertos_pl.c` para el paso sin red).
3. Propiedades → C/C++ Build → Settings: símbolos `ODR_HZ=119` (o `952`) y `N_IMU=4`, opcionalmente `PUB_MS=20`; directorio de includes `.../libmicroros/microros_include`; en el enlazador `-lmicroros` y `-L.../libmicroros`. **`-lmicroros` va antes que `lwip`, `xil`, `freertos` y `c`.** Usa barras normales en las rutas de Windows.
4. Compila solo la aplicación (el proyecto `_system` puede fallar al generar `BOOT.BIN` por un `qemu_args.txt` ausente en la plataforma; no hace falta).
5. Comprueba que la aplicación y la biblioteca usan el mismo `-mfloat-abi` (aquí `hard`).

## 3. Red y agente

Cable directo entre el RJ45 del portátil y el de la Zybo:

* Windows: adaptador Ethernet con IP fija `192.168.50.1/24`, sin puerta de enlace.
* Placa: `192.168.50.2/24` (macros `BOARD_IP`, `BOARD_NM`, `BOARD_GW`); agente en `192.168.50.1:8888` (`AGENT_IP`, `AGENT_PORT`).
* WSL2 con red `mirrored`: en `%UserProfile%\.wslconfig` pon `[wsl2]` y `networkingMode=mirrored`, y ejecuta `wsl --shutdown`.
* Cortafuegos (PowerShell como administrador): hacen falta **las dos** reglas de entrada, la de Hyper-V y la de Windows. Sin la de Hyper-V los paquetes no llegan al agente en WSL.

```
New-NetFirewallHyperVRule -Name "MicroROS-UDP" -DisplayName "micro-ROS UDP 8888" -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol UDP -LocalPorts 8888 -Action Allow
New-NetFirewallRule -DisplayName "micro-ROS UDP 8888" -Direction Inbound -Protocol UDP -LocalPort 8888 -Action Allow -Profile Any
```

* Agente (se construyó con `micro_ros_setup`; el paquete `ros-humble-micro-ros-agent` no está en apt para Humble):

```bash
source /opt/ros/humble/setup.bash
source ~/uros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

* Se ejecuta la aplicación desde Vitis con las placas quietas. La consola muestra `Esperando al agente`, `Agente encontrado` y cada 2 s `CPU libre … | publicados …, errores …`.
* Comprobaciones (en otra terminal con ROS cargado). **`ros2 daemon` no responde con WSL en modo mirrored: usar `--no-daemon`.**

```bash
ros2 topic list --no-daemon
ros2 topic hz /madgwick/q                                  # 40 Hz (PUB_MS 100) o 200 Hz (PUB_MS 20)
ros2 topic echo /madgwick/q --once --no-daemon
ros2 topic echo /madgwick/cpu_free --once --no-daemon
```

## 4. Problemas conocidos

* **USB del FTDI con `usbipd`**: la Zybo expone JTAG y UART por un solo FTDI (0403:6010). Con `usbipd bind --force` + `attach` salió `Device in error state` y dejó el dispositivo inutilizable en Vitis hasta `usbipd detach` + `unbind` y reconectar el cable. Por eso el transporte es UDP; la versión por UART no se probó.
* La sesión con el agente no se recupera sola: tras reiniciar el agente hay que reiniciar la aplicación.
* El orden de los componentes del cuaternión es `(w,x,y,z)` por defecto (`Q_W_FIRST 1`); si al girar la placa sobre un eje horizontal varían `z` y `w` en vez de `x` o `y`, compila con `-DQ_W_FIRST=0`. No se ha comprobado todavía.
* Las pruebas a 119 Hz y la de 952 Hz con 40 mensajes/s se midieron sin repetir la lectura incoherente; la de 952 Hz con 200 mensajes/s, con la repetición (0 de 9 600 descartadas; sin ella, 8 de 6 000).
* La referencia de CPU libre (iteraciones por segundo del gancho de inactividad) cambió entre ejecuciones idénticas (4,762e7 frente a 2,016e7); el porcentaje se normaliza en cada ejecución. Conviene reiniciar el sistema antes de cada ejecución.

## 5. Probar sin crear el proyecto (ELF precompilados)

En la release se publican `system_multi.bit`, `ps7_init.tcl`, los `.elf` ya compilados (`microros_udp_<ODR>_<PUB_MS>.elf`, Release) y `libmicroros.a` con `microros_include.zip`. Los ELF llevan fijados `ODR_HZ`, `PUB_MS`, la IP de la placa (192.168.50.2) y el agente (192.168.50.1:8888). Con el agente en marcha (apartado 3):

```
xsct microros/program.tcl system_multi.bit ps7_init.tcl microros_udp_119_100ms.elf
```

La secuencia es la habitual de `xsct` para el Zynq-7000 (FPGA, `ps7_init`, carga del ELF); no se ha probado en la placa. Si se programa desde Vitis después, hay que cerrar antes `hw_server`.

## 6. Visualización en RViz2

`ros/imu_tf.py` (en la raíz de la release) publica una transformación `world -> imuN` por cada mensaje de `/madgwick/q` y el ángulo relativo entre dos IMU en `/hinge/angle_deg`.

```bash
sudo apt install -y ros-humble-rviz2        # una vez; las ventanas salen por WSLg
source /opt/ros/humble/setup.bash
python3 ros/imu_tf.py --ros-args -p imu_a:=imu0 -p imu_b:=imu1 -p hinge_axis:=y
# en otra terminal (con ROS cargado): rviz2   -> Fixed Frame = world, Add -> TF
```

Las IMU se colocan separadas `spacing` metros a lo largo de X solo para verlas; el giro de cada una es el cuaternión recibido.
