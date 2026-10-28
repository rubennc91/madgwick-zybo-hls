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
