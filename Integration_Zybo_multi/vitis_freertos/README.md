# FreeRTOS en el Cortex-A9 con la cadena en la PL (`Integration_Zybo_multi`)

Mismo bitstream que `Integration_Zybo_multi` (4 IMU, un núcleo Madgwick). El Cortex-A9 ejecuta FreeRTOS 10 en vez de código sin sistema operativo.

| Fichero | Qué hace | Necesita |
|---|---|---|
| `main_freertos_pl.c` | Configura la PL como `main_pl.c`, lee los cuaterniones cada 20 ms con lectura coherente y cada 2 s imprime la CPU libre (contador del gancho de inactividad frente a una referencia medida al arrancar) y un cuaternión por IMU | BSP `freertos10_xilinx` con `use_idle_hook`, montón ≥ 128 KB |
| `main_microros_udp.c` | Lo anterior, más lwIP (Ethernet del PS, IP estática) y un cliente micro-ROS por UDP: publica `/madgwick/q` (QuaternionStamped por IMU) y `/madgwick/cpu_free` | Además lwip213 (`SOCKET_API`), `libmicroros.a` y un agente; montón de 1 MB. Ver `../../microros/README.md` |

Macros (símbolos del compilador): `N_IMU` (1 a 4, por defecto 4), `ODR_HZ` (119 o 952), `PUB_MS` (100 o 20; solo micro-ROS), `STAT_S`, `LOG_ENABLE` (1 por defecto), `BOARD_IP`/`BOARD_NM`/`BOARD_GW`, `AGENT_IP`, `AGENT_PORT`, `Q_W_FIRST`.

Las placas deben estar quietas al arrancar: el programa mide el sesgo del giróscopo (~2 s) y el periodo real de cada IMU (3 s) antes de empezar a publicar. La referencia de CPU libre se mide con la PL aún sin adquirir.
