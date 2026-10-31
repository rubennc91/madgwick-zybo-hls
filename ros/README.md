# Demo ROS 2: las 4 IMU en RViz2 con el ángulo de la bisagra

Flujo completo (FPGA -> PS -> UDP -> agente micro-ROS -> ROS 2 -> RViz2) con un solo `start_demo.bat`.

| Fichero | Para qué |
|---|---|
| `imu_tf.py` | Nodo de Python: una transformación `world -> imuN` por mensaje de `/madgwick/q`, una caja con una cara de color por eje en cada IMU (`/imu/box`), el ángulo relativo entre dos IMU (`/hinge/angle_deg`, en grados) y ese ángulo como texto para RViz2 (`/hinge/marker`) |
| `imu_tf.rviz` | Configuración de RViz2 (Fixed Frame `world`, TF, texto del ángulo y cajas) |
| `start_demo.bat` | Abre Windows Terminal con 5 pestañas: agente, topics y `hz`, nodo, eco del ángulo y RViz2 |

Probado con 4 Pmod NAV, FreeRTOS + micro-ROS sobre UDP, el agente en WSL2 (red `mirrored`) y ROS 2 Humble en Ubuntu 22.04. Puertos: imu0 = JE, imu1 = JD, imu2 = JC, imu3 = JA.

## Requisitos (una vez)
* Adaptador Ethernet de Windows con IP fija 192.168.50.1/24 (sin puerta de enlace) y cable directo a la placa.
* WSL2 en modo `mirrored`, reglas de cortafuegos de Hyper-V y de Windows para UDP 8888 y agente compilado (ver `microros/README.md`).
* `wsl -l -v` debe mostrar la distro de ROS 2; pon su nombre en `DISTRO` (con otra distro por defecto salen errores de `rclpy`, p. ej. Python 3.8).
* `imu_tf.py`, `imu_tf.rviz` y `start_demo.bat` en la misma carpeta y `REPO` apuntando a ella (ruta de WSL, `/mnt/c/...`).

## Cada vez
1. Alimentación y cable Ethernet. Cuadernos (o la bisagra) en la posición de 0° y las IMU quietas.
2. En Vitis: programar la FPGA y ejecutar la aplicación. Esperar `Esperando al agente en 192.168.50.1:8888`.
3. Doble clic en `start_demo.bat`.
4. Pestaña 1: sesión establecida y publishers creados. Pestaña 2: `/madgwick/q` y `/madgwick/cpu_free` y la tasa (40 o 200 Hz). Pestaña 3: una línea «relativo b respecto a a» cada 2 s. Pestaña 4: el ángulo (sale a la tasa de los mensajes). Pestaña 5: RViz2.
5. Para repetir el cero: Ctrl+C en la pestaña 3 y volver a lanzar su comando con la bisagra en 0°.

## Parámetros de `imu_tf.py` (`--ros-args -p nombre:=valor`)
| Parámetro | Por defecto | Efecto |
|---|---|---|
| `imu_a`, `imu_b` | `imu0`, `imu1` | IMU de las dos hojas; el ángulo es la rotación de `imu_b` respecto a `imu_a` (intercambiarlas invierte el signo) |
| `mode` | `axis` | `axis`: ángulo proyectado sobre `hinge_axis`. `total`: ángulo total de la rotación relativa (con el eje de giro real distinto del elegido, `axis` da un valor menor) |
| `tare` | `false` | `true`: el ángulo es 0 en la primera muestra de las dos IMU (cancela el desfase de montaje y de yaw entre ellas) |
| `hinge_axis` | `y` | `x`, `y` o `z`: eje que fija el signo en `mode:=total` y el eje de proyección en `mode:=axis` |
| `yaw_tare` | `false` | `true`: iguala el rumbo (yaw) de todas las IMU en su primera muestra; supone IMU planas al arrancar y no sirve en IMU inclinadas o verticales |
| `spacing` | `0.25` | Separación en X entre las IMU en RViz2 (m) |
| `box_size` | `[0.12, 0.08, 0.03]` | Largo x, ancho y y alto z de las cajas (m) |
| `parent_frame` | `world` | Frame fijo |

Cajas: +x rojo, −x rojo oscuro, +y verde, −y verde oscuro, +z azul, −z azul oscuro.

## Lo observado (no es una medida calibrada)
* Con `mode:=total` y `tare:=true`, una apertura de la bisagra a unos 90° estimados a ojo dio cambios de 88,7°, 89,8° y 94,7° en tres muestras consecutivas del log. No se ha comparado con una escuadra ni con una referencia externa.
* Con `mode:=axis` y `hinge_axis:=y` el mismo giro daba unos 82° (el eje de giro relativo salió casi en x), y sin `tare` el valor en reposo era de unos 14° a 35° según la prueba.
* En reposo la rotación relativa entre las dos IMU de la bisagra era de ≈130°, casi toda en z: el rumbo de cada IMU viene del magnetómetro y no coincide entre sensores. `tare` lo cancela al arrancar, pero si el rumbo de una IMU deriva, el ángulo derivará aunque la bisagra esté quieta. Las pruebas se hicieron a ≈1 m de los equipos electrónicos.
* El ángulo sale negativo al abrir; para positivo, intercambia `imu_a` e `imu_b`.
* Sin probar: `hinge_axis:=x` con `mode:=total` y `yaw_tare` con la bisagra (con IMU planas y paralelas ayudó en imu2 e imu3, pero no en imu0 e imu1 sobre los cuadernos).

## Problemas típicos
* Pestañas 2 a 4 con `ModuleNotFoundError: rclpy._rclpy_pybind11`: `DISTRO` apunta a otra distro (Python 3.8). Usa la de Ubuntu 22.04.
* `.../install/local_setup.bash: No such file or directory`: el agente no está en `~/uros_ws` de esa distro; busca con `find ~ -maxdepth 4 -name local_setup.bash -path "*install*"`.
* El nodo no imprime «relativo»: estás ejecutando una versión antigua de `imu_tf.py` (ROS ignora los parámetros desconocidos sin avisar).
* Sin topics: `ros2 topic list --no-daemon` (el daemon no responde con WSL en modo `mirrored`); revisa también las dos reglas de cortafuegos y la IP.
* RViz2 sin ejes: el Fixed Frame debe ser `world`. Si falla `libOgreMain`, carga antes `source /opt/ros/humble/setup.bash`.
