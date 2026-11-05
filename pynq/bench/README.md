# Medidas de la v0.8.0 con 4 IMU (ROS 2 nativo): CPU libre, mensajes/s y latencia

Scripts para repetir y completar las medidas de la v0.8.0 y poder poner la fila equivalente a la Tabla XIII del paper (micro-ROS sobre FreeRTOS). Todo con **4 IMU quietas** (cada arranque del nodo calibra el giroscopio ~8 s).

## Preparación (una vez)
Placa (desde PowerShell, en la carpeta `pynq/`):
```
scp -r bench xilinx@192.168.2.99:~/
ssh xilinx@192.168.2.99 "sed -i 's/\r$//' ~/bench/*.sh ~/bench/*.py && chmod +x ~/bench/*.sh ~/bench/*.py"
```
PC (WSL2): en cada terminal, `source /opt/ros/humble/setup.bash`, `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` y `export CYCLONEDDS_URI=file://<ruta>/ros/cyclone_pc.xml` (como en la demo).

## Tabla de pruebas

| # | Prueba | Script (dónde) | Comando | Duración | Qué produce | Dónde va en el paper |
|---|---|---|---|---|---|---|
| 1+2+3 | Matriz: 4 IMU, ODR 952 y 119 Hz, `pub_hz` 10 y 50 (40 y 200 msg/s), 2 repeticiones, modos A (solo nodo) y B (nodo + bucle de fondo), con referencia sin nodo al inicio y al final | Placa: `bench_matrix.sh`. PC: `bench_sub.py` durante toda la prueba y luego `bench_pc_summary.py` | 1) PC: `python3 bench_sub.py ~/bench_sub.csv` (déjalo corriendo). 2) Placa: `sudo -i /home/xilinx/bench/bench_matrix.sh`. 3) Al acabar, Ctrl-C en el PC y `python3 bench_pc_summary.py ~/bench_sub.csv` | ≈ 25 min (16 ejecuciones de ≈ 86 s + 2 referencias de 35 s) | Placa: `summary.txt` y `summary.csv` (CPU libre por `/proc/stat` en modo A, CPU libre por bucle de fondo en modo B, msg/s publicados, incoherentes). PC: msg/s recibidos, pérdida, huecos máximos | Nuevas filas de la Tabla XIII (ODR, mensajes/s, CPU libre, tasa vista, errores) |
| 4 | Latencia de ida y vuelta con 4 IMU a 952 Hz, `pub_hz` 10, 50 y 100, y coste de leer `q` de la PL | Placa: `bench_latency.sh` (usa `lat_board.py`). PC: `lat_echo.py` | 1) PC: `python3 lat_echo.py` (déjalo). 2) Placa: `sudo -i /home/xilinx/bench/bench_latency.sh` | ≈ 5 min (3 × 90 s) | `lat_summary.txt` y un CSV por `pub_hz`: RTT media, mediana, p95, p99, máx, RTT/2, mediana por IMU, y µs por lectura de las 4 IMU | Frase de VII-I y trabajo futuro (la latencia pasa a estar medida) |

Orden: la prueba 1+2+3 primero y la 4 después (cada una arranca y para el nodo ella sola; no dejes otro nodo en marcha). Cada prueba se lanza con las IMU quietas. Mantén abierta la sesión SSH de la placa mientras corre.

## Variables y `sudo -i`
`sudo -i` descarta las variables de entorno, así que `CONDS=... sudo -i script` no las pasa. Para una sola condición:
```
sudo -i bash -c 'CONDS="119/50" REPS=3 MODES="A" /home/xilinx/bench/bench_matrix.sh'
sudo -i bash -c 'HZS="10" /home/xilinx/bench/bench_latency.sh'
```
El PC debe usar las mismas condiciones: `python3 bench_pc_summary.py bench_sub.csv --conds 119/50 --reps 3 --modes A`. `pub_hz` se pasa al nodo como decimal (`10.0`); como entero, `rclpy` lo rechaza (los scripts ya lo convierten).

## Resultados obtenidos (4 IMU, v0.8.0)
| ODR (Hz) | msg/s | CPU libre, bucle de fondo (%) | CPU libre, `/proc/stat` (%) | recibidos en el PC (msg/s) |
|---|---|---|---|---|
| 119 | 40 | 93,6 | 93,0 | 40,0 |
| 119 | 200 | 77,7 | 72,6 | 200,0 |
| 952 | 40 | 94,1 | 92,1 | 40,0 |
| 952 | 200 | 77,2 | 72,7 | 200,0 |

Latencia de ida y vuelta (952 Hz, 4 IMU, mediana): 4,8-5,1 ms el primer mensaje del grupo de cuatro y 17,6-17,7 ms el cuarto (se publican uno tras otro, ≈4 ms entre sí). **No uses `pub_hz` 100 en `bench_latency.sh`**: el proceso de medida de la placa (otro nodo de Python) no llega a recibir todos los mensajes devueltos y esa fila no es válida; usa 10 y 50.

## Cómo leer los resultados
* **CPU libre por `/proc/stat` (modo A)**: lo que mide el nodo y ya se midió antes; sirve de referencia con la tabla de v0.8.0.
* **CPU libre por bucle de fondo (modo B)**: un bucle de baja prioridad por núcleo; la velocidad con el nodo en marcha dividida entre la velocidad sin nodo. Es la definición del paper (VII-H: «velocidad de un bucle de fondo con y sin la adquisición»), con la diferencia de que aquí el sistema tiene Linux y dos núcleos. En modo B `/proc/stat` no vale (el bucle ocupa todo), por eso no se da.
* **Referencia sin nodo**: las dos medidas (inicio y final) deben coincidir; la deriva aparece en la primera línea del resumen. Si pasa del 3-5 %, repite (suele ser temperatura o algo más corriendo en la placa).
* **msg/s publicados (placa)** y **msg/s recibidos (PC)**: con `pub_hz` ≤ 100 por IMU deben ser 4 × `pub_hz`. La pérdida del PC compara con ese valor.
* **Huecos máximos**: a 10 Hz salen ≈ 100 ms porque las cuatro IMU se publican de golpe (ráfagas de 4), no es jitter.
* **Latencia**: la marca de tiempo del mensaje la pone el nodo de la placa y el RTT se calcula con el mismo reloj, así que no hace falta sincronizar el PC. Ida estimada = RTT/2 (camino supuesto simétrico, incluye el tiempo del eco en el PC y su DDS). **Latencia de la muestra al mensaje** ≈ lectura de `q` (medida por `lat_board.py`) + retardo de la PL (109-135 µs de adquisición + hasta 29,4 µs del núcleo, del paper) + RTT/2.

## Si algo falla
| Síntoma | Qué mirar |
|---|---|
| `bench_pc_summary.py` dice «NO COINCIDEN» | Falló alguna ejecución (p. ej. una IMU sin responder); mira los `node_*.log` de la placa y `runs.csv` |
| El nodo no llega a publicar (`nota` en el resumen) | Error de IMU o de `sudo -i`; ejecuta `nav_diag.py` |
| `lat_board.py`: «Sin mensajes en /bench/echo» | `lat_echo.py` no está corriendo en el PC o falta `CYCLONEDDS_URI` en esa terminal |
| `bad interpreter` / `^M` | Finales de línea CRLF: el `sed` de la preparación |

Los scripts de resumen y de recuento se probaron con datos simulados; no se han ejecutado sobre la placa.
