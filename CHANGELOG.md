# Cambios

## v0.6.3 — criterio de tramas perdidas corregido en la línea base con interrupciones; CHANGELOG de v0.3.0

**Software**: `Integration_Zybo_swspi/vitis_app/main_irq.c` cuenta una trama perdida a partir del instante de entrada de la interrupción (múltiplo exacto del tick), no del instante de lectura. En la v0.6.2 el retardo de la rutina hasta llegar a cada sensor (hasta ≈ 350 µs con 4 IMU) daba falsos positivos. No cambia ningún tiempo medido.

**Datos**: `data/e2e/irq4_O3_119.log` e `irq4_O3_952.log` se repiten con el firmware nuevo (4 IMU, `POLL_DIV 2`). Resultado: 0 tramas perdidas en las tres fases, en las cuatro IMU, a 119 y a 952 Hz; las tasas, la CPU en la rutina (4,2 % y 28,8 %) y la CPU libre (95,8 % y 71,1 %) coinciden con los de la v0.6.2.

**Documentación**: la entrada «v0.3.0» de este fichero describía el primer diseño multi-IMU con `r2f_mux`, pero la etiqueta v0.3.0 corresponde al sistema dual (núcleo original y opt2 en la misma FPGA, captura CSV y análisis de error). Se corrige la entrada y el diseño multi-IMU con `r2f_mux` pasa a llamarse «primer diseño multi-IMU (anterior a v0.4.0, sin etiqueta)».

## v0.6.2 — línea base en el ARM con interrupciones

**Software**: `Integration_Zybo_swspi/vitis_app/main_irq.c` (nuevo). Mismo diseño `zybo_swspi` (`NIMU 4`) y mismo Madgwick que `main.c`, pero la adquisición se hace en una rutina de interrupción del temporizador privado del Cortex-A9 (GIC, IRQ 29) y no por sondeo. El periodo del temporizador es la mitad del periodo más corto de los sensores (`POLL_DIV 2`: ≈ 4,03 ms a 119 Hz, ≈ 0,50 ms a 952 Hz). El programa principal ejecuta el mismo bucle de fondo que `main_pl.c`, de modo que la CPU libre se mide igual que en la cadena en la PL. Macros `N_IMU`, `ODR_HZ`, `POLL_DIV`, `PHASE_SECONDS`, `PHASE_C_SECONDS`.

**Datos**: `data/e2e/irq4_O3_119.log` e `irq4_O3_952.log` (4 IMU quietas, Release `-O3`, SCK 3,125 MHz).

**Resultados**

| | ARM sondeo 119 Hz | ARM interrupciones 119 Hz | ARM sondeo 952 Hz | ARM interrupciones 952 Hz | PL |
|---|---|---|---|---|---|
| CPU ocupada por la cadena | 3,4 % | 4,2 % | 22,4 % | 28,8 % | 0 % |
| CPU libre para otra tarea | 0 % | 95,8 % | 0 % | 71,1 % | 100 % |
| Una línea por trama (UART 115200): tramas perdidas | 0 | 0 | ≈ 54 % | 0 (tasa medida = nominal) | 0 |
| Latencia dato → cuaternión (µs) | 71,7 | 71,7 + espera ≤ 1 tick | 57,9 | 57,9 + espera ≤ 1 tick | 136 / 109 + núcleo |
| Rutina de interrupción, media / máx. (µs) | | 168,9 / 352,5 | | 145,1 / 351,0 | |

- La rutina nunca desborda el periodo del temporizador (0 desbordes en las 3 fases). Con la UART imprimiendo una línea por trama, el ARM con interrupciones no pierde tramas (la UART se atiende en el programa principal, con prioridad menor).
- Coste frente a la PL: la CPU queda ocupada entre el 4 % y el 29 %, y cada dato espera hasta un tick antes de leerse; los intervalos entre tramas llegan a ≈ 1,5 periodos.
- El contador «perdidas» marca falsos positivos (p. ej. 2 en IMU3 a 119 Hz y hasta 58 a 952 Hz): el umbral `periodo + tick + 100 µs` no incluye el retardo de la propia rutina (hasta ≈ 350 µs) hasta llegar a ese sensor. Las tasas medidas coinciden con las nominales (p. ej. 992,8 Hz en IMU3 a 952 Hz), así que no se pierde ninguna trama.
- Es un temporizador, no una interrupción de dato listo (DRDY): el Pmod NAV lleva INT en el pin 7 del conector, que este diseño no conecta.

## v0.6.1 — línea base de extremo a extremo con 4 IMU

**Datos**: `data/e2e/` incluye las medidas con 4 IMU en el ARM (`sw4_O3_*`) y en la PL (`pl4_*`), y las dos del ARM con SCK de 6,25 MHz (`*_sck625`).

**Resultados** (4 IMU quietas, Release `-O3`; ARM con SCK 3,125 MHz)

| | ARM 119 Hz | ARM 952 Hz | PL 119 Hz | PL 952 Hz |
|---|---|---|---|---|
| Tramas perdidas (sin UART / una línea cada 100 ms) | 0 | 0 | 0 | 0 |
| Una línea por trama (UART 115200) | 0 | ≈ 54 % | 0 | 0 |
| Latencia dato → cuaternión (µs, media) | 71,3 | 57,9 | 136 + núcleo | 109 + núcleo |
| CPU ocupada por la cadena | 3,4 % | 22,4 % | 0 % | 0 % |

- La carga de CPU del ARM con 4 IMU a 952 Hz es 22,4 %: coincide con la extrapolación de la v0.6.0 (≈ 22 %), que pasa a ser una medida.
- Con el SCK al doble (6,25 MHz), la CPU baja a 3,4 % → 2,0 % (119 Hz) y 22,4 % → 13,8 % (952 Hz).
- La PL no pierde tramas con 4 IMU y deja la CPU libre al 100 %. La espera máxima en el stream a 952 Hz es de 23–39 µs. En la fase de una línea por trama a 952 Hz, 10 de 2216 lecturas del cuaternión salieron no coherentes (0,45 %).

**Notas**
- `sw4_O3_119.log` usa la calibración del temporizador con `usleep` (335,321 MHz): las tasas salen ≈ 0,6 % altas.

## v0.6.0 — línea base de extremo a extremo: adquisición y filtro en el ARM frente a la cadena en la PL

**Hardware (`Integration_Zybo_swspi/`, nuevo)**
- Diseño sin ninguna IP propia: Zynq PS + 2 × AXI Quad SPI (estándar, 2 esclavos: CS_AG = `ss[0]`, CS_M = `ss[1]`; SCK = 100 MHz / 32 = 3,125 MHz; modo SPI 3). Pmod NAV en JE (IMU 0) y JD (IMU 1). `create_project.tcl`, `build_project.tcl`, XDC y README.
- `test/host_sim.*`: simulador en el PC para ejecutar `main.c` sin placa.

**Software**
- `Integration_Zybo_swspi/vitis_app/main.c`: el ARM hace todo (SPI por registros, calibración, conversión, Madgwick idéntico a `Software_Baseline_ARM/sw_baseline.c`, UART). Mide por IMU el tiempo de SPI, el del filtro, la latencia desde el dato nuevo (GDA) hasta el cuaternión, el intervalo entre tramas y las tramas perdidas. Macros `ODR_HZ` (119/952), `PRINT_10HZ`, `PHASE_SECONDS`, `PHASE_C_SECONDS`. Incluye un diagnóstico (`spi_diag`) que lee `SPISSR` para comprobar el número de esclavos.
- `Integration_Zybo_multi/vitis_app/main_pl.c`: mismas fases y mismas tablas con la cadena en la PL; carga de CPU medida con un bucle de fondo idéntico en referencia, fase A y fase B, y una referencia final con la PL parada.

**Datos**: `data/e2e/` (5 logs con su README).

**Resultados** (2 IMU, medias; entre paréntesis el máximo)

| | SW 119 | SW 952 | PL 119 | PL 952 |
|---|---|---|---|---|
| Tasa medida (Hz) | 120,8 / 121,7 | 965,8 / 974,1 | 120,7 / 121,8 | 965,8 / 974,6 |
| Tramas perdidas | 0 | ≈ 55 % en la fase C (UART) | 0 | 0 |
| SPI (µs) | 70,4 (78,6) | 56,6 (79,0) | — | — |
| Filtro (µs) | 0,90 (1,49) | — | núcleo ≈ 29 µs | núcleo ≈ 29 µs |
| Latencia dato→cuaternión (µs) | 71,3 (80,0) | 57,5 (80,5) | 135 (171) + núcleo | 109 (151) + núcleo |
| Intervalo entre tramas (ms) | 8,20–8,37 | 0,97–1,13 | — | — |
| CPU ocupada por la cadena | 1,7 % | 11,2 % | 0 % | 0 % |

**Lectura**
- En el ARM la latencia es menor (SPI a 3,125 MHz en sondeo; la PL adquiere a 2,5 MHz), pero la cadena ocupa la CPU y a 952 Hz la UART a 115200 le hace perder tramas. Sondeo, no interrupciones: con interrupciones la CPU quedaría libre en parte, no medido.
- La cadena en la PL no consume CPU (cociente de la referencia final ≈ 1), no pierde tramas y no depende de la UART; el procesador queda libre para un sistema operativo o ROS (habilitado, no construido ni medido aquí).
- La carga con 4 IMU en el ARM (≈ 22 %) es una extrapolación, no una medida.

**Notas**
- `main_pl.c` usa 333,333 MHz fijos para el temporizador (la calibración con `usleep` varía ≈ ±0,7 % entre arranques).
- La fase C dura 5 s por el límite de 10 000 líneas de la consola de Vitis.

## v0.5.0 — temperatura, modo UHP, escalado de ODR, línea base ARM y calibración elipsoidal

Recoge también el trabajo de «v0.4.1», que no llegó a publicarse.

**Hardware (`PMOD_NAV_Ctrller/`)**
- `nav_spi_ctrl`: lectura de la temperatura del LSM9DS1 cada 128 tramas (≈ 1 s; dos bytes extra, ≈ 10 µs; no forma parte de la trama enviada al núcleo ni altera su cadencia). Registros `0xAC` TEMP_RAW (int16 con signo; T = 25 + raw/16 °C, solo variaciones fiables) y `0xB0` TEMP_COUNT. Generic `G_TEMP_EVERY` (0 = desactivado). **Hay que reempaquetar el IP (`package_nav_ip.tcl`) y regenerar el bitstream.**
- `nav_pkg`: parámetros `MAG_PERF` (modo de rendimiento del magnetómetro: 00 bajo consumo … 11 ultra alto) y `MAG_TEMP_COMP`. Por defecto `"00"` y `'0'` (como en v0.4.0). Los resultados del artículo con UHP usaron `"11"` y `'1'`; reducen el ruido por muestra ≈ 30 % pero no la deriva lenta del rumbo.

**Software (`Integration_Zybo_multi/vitis_app/main.c`)**
- `ODR_CODE` (3…6 = 119, 238, 476, 952 Hz); `dt` por IMU medido al arrancar con el contador de ciclos y escrito en el núcleo.
- Streaming binario `s` (visor en vivo) y streaming **crudo** `w` (`A5 5C`, 67 B por trama y por IMU: las 9 palabras enviadas al núcleo, su cuaternión y la temperatura) para repetir el filtro fuera de línea. UART a 921 600 baud.
- `MAG_PRESET` actualizada: IMU0 e IMU1 recalibradas sueltas con `m`; las IMU2 e IMU3 conservan el valor anterior. Con `ODR_CODE != 3` hay que usar `MAG_PRESET` (`m`/`M` asumen 119 Hz). El modo `w` exige `ODR_CODE 3`.

**Análisis (`analysis/`)**
- `capture_raw.py`: captura del streaming crudo a CSV (con temperatura; acepta `A5 5B` y `A5 5C`).
- `replay_variants.py`: reproduce el filtro fuera de línea (con y sin magnetómetro, otras ganancias) y analiza la temperatura.
- `viewer_live.py` / `plot_drift.py`: visor en vivo de dos IMU (ángulo relativo, bisagra) y análisis de su CSV.
- `mag_ellipsoid.py`: calibración del magnetómetro por elipsoide (hard-iron + soft-iron) con validación cruzada; solo fuera de línea (el firmware sigue aplicando únicamente el offset).

**Línea base en software (`Software_Baseline_ARM/`)**
- El filtro Madgwick en C en el Cortex-A9 con la misma entrada que el núcleo (12 000 tramas). Funciona en Vitis 2023.2 (flujo SDT) sin `xtime_l.h` ni `-lm`.

**Resultados**
- ARM (Release `-O3`): 0,82 µs por actualización de media (máx. 1,64 µs); Debug `-O0`: 2,60 µs. Carga de CPU a 123,6 Hz: 0,010 % por IMU. El núcleo de la FPGA tarda 29,4 µs (peor caso): la cadena no acelera el filtro, libera al procesador de la adquisición.
- ARM frente al núcleo, mismas palabras: 4,3e-4° de media, 2,8e-3° de máximo.
- Escalado de ODR (4 IMU, un núcleo): carga del núcleo del 1,4 % (119 Hz) al 11,4 % (952 Hz), 0 tramas perdidas, espera máxima en el árbitro 3,3 → 43 µs, diferencia con float32/float64 1,0e-4° → 6,4e-4°.
- Temperatura: un escalón de 19 °C en una IMU cambia su sesgo de giroscopio 0,25 °/s y el ángulo relativo < 0,1°. La deriva lenta del rumbo relativo (σ ≈ 0,1–2° según el sitio) es ruido de baja frecuencia del magnetómetro de 1–2 mG, no depende de la temperatura ni del modo de rendimiento.

**Notas**
- `MAG_PRESET`: recalibrar cada IMU suelta con `m` (no `M`, que asume cuerpo rígido).
- Los ensayos de un banco casero de ángulos relativos (rumbo con dos IMU) no son concluyentes (el cero varía 3,8–6,8° entre sesiones y el entorno magnético domina); se repetirán sobre una mesa óptica. No hay cambios de hardware aparte de los de `PMOD_NAV_Ctrller/`.


## v0.4.0 — 1 a 4 IMUs con un único núcleo, verificado en placa

**Hardware**
- Nuevo `Integration_Zybo_multi/axis_rr_mux4.vhd`: árbitro round-robin determinista por trama completa (TID = IMU). Sustituye a `r2f_mux` (HLS).
- Nuevo `HLS_madgwick_multi/r2f_tid`: raw2float con TID de paso (un solo puerto de entrada). `r2f_mux` queda obsoleto y se elimina.
- `madgwick_multi`: `dt` pasa de un registro común (0x18) a `dt_imu[4]` en 0x90 (un periodo de muestreo por IMU). El resto del mapa de registros no cambia.
- `nav_spi_ctrl` v2: registros de instantánea (0x74..0xA8): última trama enviada al núcleo, marcas de tiempo (contador de ciclos de 32 bits a 100 MHz) y espera máxima en `S_STREAM`.

**Software**
- `main.c`: lectura atómica de la instantánea (sin `usleep`), tabla `DT_IMU[]`, columnas `ok,ts_gda,ts_end,str_cyc` en el CSV, `dt_us` y `snap=1` en la cabecera. Con 1, 2 o 3 IMUs conectadas ya no hace falta habilitar las cuatro.
- `analyze_multi.py` / `replay.py`: reloj de hardware con desenvuelto de 32 bits, estadísticas por segmentos tolerantes a huecos del log, `dt` por IMU leído de la cabecera.

**Recursos (4 IMUs)**: 7 199 LUT (40,9 %), 6 980 FF (19,8 %), 8 DSP, 3 RAMB36; timing cumplido (WNS +0,142 / WHS +0,026 ns); potencia estimada 1,57 W (PS7 1,40 W).

**Resultados** (ver README): sin esperas en el stream (máx. 17,6 µs, antes 4–8 ms de mediana), sin pérdida de muestras del sensor, núcleo frente a float32 ≤ 1,7e-4°, 16 min sin errores ni deriva, coherencia entre IMUs 1,6 / 2,0 / 4,2° de media en la captura de 3 vueltas.

**Corregido**
- Con menos de 4 IMUs conectadas no llegaba ninguna trama al núcleo (el reparto HLS dependía de la entrada 3).
- Las IMU 0–2 esperaban en `S_STREAM` hasta 1–2 periodos y perdían muestras (≈ 15 % de intervalos de 16 ms en IMU0).
- Carrera en el registro de las capturas (los registros crudos se sobrescribían por la adquisición siguiente).

**Limitaciones conocidas**
- Margen de timing ajustado: WNS +0,142 ns a 100 MHz con 4 IMUs (cumple, pero sin holgura; ver `Integration_Zybo_multi/reports/`).
- Cada `nav_spi_ctrl` v2 ocupa ≈ 250 LUT / 468 FF más que en v0.3.0 por la instantánea.
- Coherencia entre IMUs sin referencia externa; residuo mayor en la IMU de JA (calibración del magnetómetro sin verificar).
- El `dt` por IMU está medido para las cuatro placas de las pruebas; hay que volver a medirlo si se cambian.
- Pérdida de unas 11 filas de 43 000 en el enlace serie en la captura de 90 s (el filtro no pierde tramas).

## v0.3.0 — sistema dual: Madgwick original y opt2 en la misma FPGA
Dos núcleos (original y opt2) alimentados con la misma trama, con captura CSV y análisis del error numérico: `Integration_Zybo_dual/` (diseño, `create_project.tcl`, `build_project.tcl`, `vitis_app/main.c`, XDC), `HLS_madgwick_stream_opt2/run_hls_madgwick_stream_opt2_ip2.tcl` y `analysis/` (`analyze_dual.py`, `replay.py`).

## Primer diseño multi-IMU (anterior a v0.4.0, sin etiqueta)
Diseño multi-IMU (4 Pmod NAV, un núcleo) con `r2f_mux` en HLS. Probado en C; limitaciones en el apartado «Por qué un árbitro en VHDL».
