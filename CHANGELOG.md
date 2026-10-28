# Cambios

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

## v0.3.0
Primera versión del diseño multi-IMU (4 Pmod NAV, un núcleo) con `r2f_mux` en HLS. Probada en C; limitaciones en el apartado «Por qué un árbitro en VHDL».
