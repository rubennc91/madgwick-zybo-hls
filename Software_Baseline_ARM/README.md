# Línea base en software: el filtro Madgwick en el ARM con la misma entrada que el núcleo

Mide el **filtro** (no la adquisición SPI ni la salida) en el Cortex-A9 de la Zybo, bare-metal, con las **mismas 12 000 tramas crudas** que recibió el núcleo de la FPGA.

| Fichero | Qué es |
|---|---|
| `sw_baseline.c` | Filtro en C (float32, mismo orden de operaciones que el modelo del núcleo). Mide el tiempo por actualización (caché caliente y fría), la diferencia con el cuaternión del núcleo y la carga de CPU |
| `sw_input.h` | Entrada ya generada: 12 000 tramas de la IMU 0 de `base_45_uhp` (123,56 Hz, `dt` 8,093 ms) y el cuaternión del núcleo (`SW_QCORE`) |
| `sw_input_qcore.csv` | El mismo cuaternión del núcleo en CSV |
| `gen_sw_input.py` | Genera otro `sw_input.h` a partir de una captura cruda (`capture_raw.py`) |
| `results/` | Salidas por UART de las dos compilaciones medidas |

## Cómo medir (Vitis 2023.2)

1. Proyecto bare-metal `ps7_cortexa9_0` nuevo; añade `sw_baseline.c` y `sw_input.h` (la UART es la salida).
2. Compila en **Debug (-O0)** y en **Release (-O3)** y apunta ambos resultados. Comprueba en el log de compilación el flag que queda al final de la línea de `arm-none-eabi-gcc` (el último gana): flags de la FPU `-mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard`.
3. No hace falta `xtime_l.h` (el BSP del flujo SDT no lo trae: el temporizador global se lee directamente en `0xF8F00200`) ni `-lm` (la raíz cuadrada usa la instrucción `VSQRT`).
4. En PC se comprueba la lógica (los tiempos de PC no valen): `gcc -O2 -ffp-contract=off -DHOST sw_baseline.c -o sw -lm && ./sw`.

## Resultados (Zybo Z7-10, 666,7 MHz, reloj de medida 333,3 MHz = CPU/2)

| | -O0 caliente | -O0 fría | -O3 caliente | -O3 fría |
|---|---|---|---|---|
| Media (µs) | 2,60 | 3,28 | **0,82** | 1,21 |
| p99,9 (µs) | 2,69 | 3,53 | 1,14 | 1,56 |
| Máximo (µs) | 4,09 | 3,55 | 1,64 | 1,56 |
| Carga de CPU a 123,6 Hz, 1 IMU (media / peor) | 0,032 / 0,051 % | | 0,010 / 0,020 % | |
| Carga de CPU, 4 IMUs (media / peor) | 0,13 / 0,20 % | | 0,040 / 0,081 % | |

Diferencia con el cuaternión del núcleo (t > 5 s): media 4,3e-4°, máx. 2,8e-3° (igual que en PC; es mayor que la del núcleo frente a su modelo float32 porque aquí `1/sqrtf` sustituye a la semilla mágica + Newton-Raphson).

**Lectura:** el filtro no es lo que limita una implementación en software: una actualización cuesta 0,82 µs frente a los 29,4 µs (peor caso) del núcleo compartido. La ventaja de la cadena en hardware no es la velocidad de una actualización, sino sacar al procesador de la adquisición (SPI, sondeo de data-ready, salida) y el reparto determinista. Esta medida es **bare-metal y solo del filtro**; la latencia extremo a extremo con SPI desde el ARM queda como experimento aparte.
