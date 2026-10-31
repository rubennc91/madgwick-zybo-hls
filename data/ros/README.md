# Medidas de micro-ROS (v0.7.0)

Programa: `Integration_Zybo_multi/vitis_freertos/main_microros_udp.c` (sin la repetición de la lectura incoherente, que se añadió después). 4 IMU quietas, cable directo a 1000 Mbps, agente en WSL2.

Los ficheros `consola_*.txt` son los extractos de la consola de la placa tal como se copiaron durante la medida (no son los logs completos). `topic_hz_resumen.md` resume la salida de `ros2 topic hz /madgwick/q`, también copiada a mano: no se conservó el volcado completo.

| Fichero | ODR | Mensajes/s (`PUB_MS`) |
|---|---|---|
| `consola_119_100ms.txt` | 119 Hz | 40 (100) |
| `consola_119_20ms.txt` | 119 Hz | 200 (20) |
| `consola_952_100ms.txt` | 952 Hz | 40 (100) |
| `consola_952_20ms.txt` | 952 Hz | 200 (20) |
| `consola_952_20ms_reintento.txt` | 952 Hz | 200 (20), con la lectura incoherente repetida una vez |

En `consola_952_20ms.txt` (sin repetición) hay 8 lecturas descartadas en 6 000; en `consola_952_20ms_reintento.txt` (con repetición) hay 0 en 9 600. La referencia de CPU de las dos ejecuciones a 952 Hz sin repetición fue 2,016e7 it/s y en la última, 4,762e7 it/s (mismo código y compilación; el porcentaje es el normalizado).
