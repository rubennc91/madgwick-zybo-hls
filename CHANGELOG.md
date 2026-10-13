# Cambios

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
