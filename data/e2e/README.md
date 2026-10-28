# Medidas de extremo a extremo: adquisición y filtro en el ARM frente a la cadena en la PL

Salida de la UART (115200 8N1) de dos programas, con 2 IMU quietas, SPI en ARM a 3,125 MHz y fases A (sin salida), B (una línea cada ~100 ms) y C (una línea por trama, 5 s).

| Fichero | Programa | Diseño | Compilación | ODR |
|---|---|---|---|---|
| `sw_O0_119.log` | `Integration_Zybo_swspi/vitis_app/main.c` | `zybo_swspi` (AXI Quad SPI ×2) | Debug `-O0` | 119 Hz |
| `sw_O3_119.log` | idem | idem | Release `-O3` | 119 Hz |
| `sw_O3_952.log` | idem | idem | Release `-O3` | 952 Hz |
| `pl_119.log` | `Integration_Zybo_multi/vitis_app/main_pl.c` | `zybo_multi` (2 IMU) | Release `-O3` | 119 Hz |
| `pl_952.log` | idem | idem | Release `-O3` | 952 Hz |

La tasa real del chip es ≈ 1,4 % (119 Hz) y ≈ 2,3 % (952 Hz) superior a la nominal (oscilador del LSM9DS1); por eso salen 120,8 y 965,8 Hz.
En el programa del ARM, a 952 Hz la UART (2,3 ms por línea) bloquea el bucle en la fase C y se pierde ≈ 55 % de las tramas; en la PL no se pierde ninguna, pero la UART solo deja imprimir el 43 % de las líneas.

## 4 IMU (v0.6.1)

| Fichero | Programa | Diseño | SCK del AXI Quad SPI | ODR |
|---|---|---|---|---|
| `sw4_O3_119.log`, `sw4_O3_952.log` | `Integration_Zybo_swspi/vitis_app/main.c` (`N_IMU 4`) | `zybo_swspi` con `NIMU 4` | 3,125 MHz (ratio 16 × 2) | 119 / 952 Hz |
| `pl4_119.log`, `pl4_952.log` | `Integration_Zybo_multi/vitis_app/main_pl.c` (`N_IMU 4`) | `zybo_multi` | 2,5 MHz | 119 / 952 Hz |
| `sw4_O3_119_sck625.log`, `sw4_O3_952_sck625.log` | igual que `sw4_O3_*` | bitstream con ratio 16 × 1 por error | **6,25 MHz** | 119 / 952 Hz |

Los `*_sck625.log` se conservan como medida de qué cambia al doblar el SCK; la cabecera del firmware imprime «SPI 3.125 MHz» (texto fijo), pero el reloj real era el doble: la lectura de una trama pasa de ≈ 70 µs a ≈ 43 µs a 119 Hz.
En `sw4_O3_119.log` el temporizador se calibró con `usleep` (335,321 MHz en lugar de 333,333), así que las tasas en Hz salen ≈ 0,6 % altas (121,47 en lugar de 120,75 Hz en la IMU0); los tiempos en µs, ≈ 0,6 % bajos.

## Con interrupciones (v0.6.2)

| Fichero | Programa | Diseño | SCK | ODR |
|---|---|---|---|---|
| `irq4_O3_119.log`, `irq4_O3_952.log` | `Integration_Zybo_swspi/vitis_app/main_irq.c` (`N_IMU 4`, `POLL_DIV 2`) | `zybo_swspi` con `NIMU 4` | 3,125 MHz | 119 / 952 Hz |

Temporizador fijo a 333,333 MHz (no calibrado con `usleep`). Cada fase informa de la CPU en la rutina de interrupción y, en A y B, de la CPU libre medida con el bucle de fondo.
El contador «perdidas» de estos logs da falsos positivos (el umbral no incluye el retardo de la rutina hasta llegar a cada sensor); la tasa en Hz coincide con la nominal, así que no hay pérdida real.
