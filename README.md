# madgwick-zybo-hls

Filtro de orientación **Madgwick (MARG)** completamente en FPGA para una **Zybo Z7-10** (xc7z010clg400-1),
con un **Pmod NAV** (LSM9DS1 + LPS25HB) en el conector **JE**, y una variante de **1 a 4 Pmod NAV servidos por un único núcleo Madgwick** (v0.4.0).
Herramientas: Vivado / Vitis / Vitis HLS **2023.2.1**.

```
Pmod NAV (SPI) -> nav_spi_ctrl (VHDL, AXI4-Stream 9 x int32)
               -> raw2float_top (HLS, 9 x float)
               -> madgwick_stream_top (HLS, 1 cuaternión por trama, 119 Hz)
PS (Zynq) solo por AXI-Lite: configuración, calibración y lectura de q por UART.
```

## Contenido

| Carpeta | Qué hay |
|---|---|
| `PMOD_NAV_Ctrller/` | VHDL: `spi_engine`, `nav_spi_ctrl` (adquisición SPI, registros AXI-Lite, AXI-Stream), testbenches y scripts de simulación, `package_nav_ip.tcl` |
| `HLS_raw2float/` | Conversor int32 → float (escala del giróscopo según `FS_G`) |
| `HLS_madgwick_stream/` | Núcleo Madgwick en streaming + referencia `madgwick_f2.c/.h` + testbench |
| `HLS_madgwick_stream_opt/` | Misma función con operadores serializados (1 fmul, 1 fadd, 1 fcmp). Bit-exacta con la anterior en simulación; se conserva como referencia |
| `HLS_madgwick_stream_opt2/` | **Versión de bajo consumo**: un intérprete de programas en ROM con una sola unidad fadd/fmul/fcmp y rsqrt por Newton-Raphson. `gen_prog.py` genera `madgwick_prog.h` |
| `integration_zybo/` | Integración con el núcleo **original** (`HLS_madgwick_stream`): `create_project.tcl`, `build_project.tcl`, XDC y `vitis_app/main.c` |
| `Integration_Zybo_opt2/` | Misma integración con el núcleo **opt2** (proyecto `zybo_opt2`, `AXIS_CFG_PRESET 0x039`) |
| `analysis/` | `replay.py` (modelo del filtro en float32/float64), `analyze_dual.py` (error numérico de cada núcleo), `analyze_poses.py` (prueba de poses con un cubo, incl. escala del giroscopio) y `analyze_multi.py` (varias IMUs). Necesitan `numpy` y `matplotlib` |
| `data/` | Capturas CSV de la UART (texto, ~1 MB cada una) usadas en los resultados; ver `data/README.md` |
| `HLS_madgwick_multi/` | **Un solo núcleo para varias IMUs**: `madgwick_multi` (opt2 con un estado `q` y un `dt` por IMU, identificadas por TID) y `r2f_tid` (raw2float que conserva el TID). Incluye banco de pruebas en C (`test_pc.sh`) |
| `Integration_Zybo_multi/` | Integración de **1 a 4 Pmod NAV** (JE, JD, JC, JA) con un único núcleo Madgwick (proyecto `zybo_multi`). `axis_rr_mux4.vhd` es el árbitro round-robin de las 4 entradas; `report_multi.tcl` extrae timing, utilización y potencia |
| `Integration_Zybo_dual/` | **Los dos núcleos a la vez** (original y opt2) alimentados con la misma trama por un `axis_broadcaster`; `main.c` compara sus cuaterniones y captura un CSV para el análisis offline |
| `Software_Baseline_ARM/` | El mismo filtro Madgwick en el Cortex-A9 con la misma entrada que el núcleo: tiempo por actualización, diferencia numérica y carga de CPU (ver su README) |
| `analysis/capture_raw.py`, `replay_variants.py`, `viewer_live.py`, `plot_drift.py`, `mag_ellipsoid.py` | Captura cruda por UART, reproducción del filtro fuera de línea (temperatura, magnetómetro), visor en vivo de 2 IMU y calibración elipsoidal del magnetómetro |


## Reproducir desde cero

Ejecuta desde la raíz del repo (la estructura de carpetas debe mantenerse: los scripts usan rutas relativas).

1. **Empaquetar el IP VHDL**
   `cd PMOD_NAV_Ctrller && vivado -mode batch -source package_nav_ip.tcl`
2. **Sintetizar y exportar los núcleos HLS**
   `cd HLS_raw2float && vitis_hls -f run_hls_raw2float.tcl`
   `cd HLS_madgwick_stream && vitis_hls -f run_hls_madgwick_stream.tcl`
3. **Crear el proyecto Vivado y el block design**
   `cd integration_zybo && vivado -mode batch -source create_project.tcl`
4. **Bitstream y XSA**
   `vivado -mode batch -source build_project.tcl` (exporta `madgwick_zybo_z710.xsa`, que ya incluye el bitstream)

### Variante de bajo consumo (opt2)
Sustituye el paso 2 del núcleo Madgwick y la carpeta de integración:

```
cd HLS_madgwick_stream_opt2 && vitis_hls -f run_hls_madgwick_stream_opt2.tcl
cd ../Integration_Zybo_opt2 && vivado -mode batch -source create_project.tcl
vivado -mode batch -source build_project.tcl     # exporta zybo_opt2.xsa
```
El testbench y la referencia en C se toman de `HLS_madgwick_stream/`. Si cambias el algoritmo de `madgwick_f2.c`, regenera el programa con `python3 gen_prog.py`.

### Comparación en paralelo (original + opt2, misma entrada)
```
cd HLS_madgwick_stream_opt2 && vitis_hls -f run_hls_madgwick_stream_opt2_ip2.tcl   # mismo opt2, empaquetado como IP "mad_opt2"
cd ../Integration_Zybo_dual && vivado -mode batch -source create_project.tcl
vivado -mode batch -source build_project.tcl     # exporta zybo_dual.xsa
```
Mapa de direcciones: `nav_spi_ctrl` 0x40000000, original 0x40010000, `raw2float` 0x40020000, opt2 0x40030000.
En la UART: `l` captura 60 s (estado actual), `L` reinicia los dos filtros y captura desde la semilla, `d` repite el volcado. Durante la captura la terminal avisa de `MOV` / `QUIETO` / `LISTO - mueve` (esto último tras 8 s de reposo inicial y 3 s en cada pose), y cada fila lleva `t_us` (tiempo en µs desde el temporizador global del procesador, calibrado al arrancar; `gt_hz` en la cabecera). Para los experimentos se recomienda `L` (placa quieta los primeros segundos): empieza en un estado conocido y la reproducción en el PC es exacta; `l` sirve para seguir capturando con el filtro ya convergido. El volcado es un CSV (cabecera `#HDR` con ejes y offsets, una fila por trama con los 9 datos crudos, los dos cuaterniones en hexadecimal de float32, `ok` y `t_us`). Guarda la salida del terminal en un fichero (Tera Term: File > Log).

### Análisis de una captura
```
pip install numpy matplotlib
python analysis/analyze_dual.py data/captura.log figura.png
```
Imprime el error angular (grados) de cada núcleo respecto a la referencia en doble precisión y entre ellos. Detecta si la captura se hizo con `L` o con `l`. Como control, el modelo en float32 del PC reproduce la salida del núcleo original (diferencia <1e-4°).

> En Windows, Vivado limita las rutas a 260 caracteres; por eso el proyecto opt2 se llama `zybo_opt2`. Mantén el repo en una ruta corta.
5. **Aplicación**: en Vitis crea una plataforma desde el `.xsa` (rehazla cada vez que cambie), una app standalone vacía y copia `integration_zybo/vitis_app/main.c`. Añade `m` a las librerías del linker si falla `atan2f/sqrtf`.
   UART a 115200 8N1.

### Sin Vivado/HLS: binarios precompilados
En **Releases** hay `system_wrapper.xsa` (con bitstream), `system_wrapper.bit` y `app_component.elf` listos para programar la placa, para la versión original (v0.1.0). La versión opt2 se publica como `zybo_opt2.xsa`/`.bit`/`.elf` (v0.2.0).

## Recursos y timing: original frente a opt2

Sistema completo (PS + nav_spi_ctrl + raw2float + Madgwick + SmartConnect), post-implementación, xc7z010, reloj de 100 MHz:

| | Original | opt2 | Reducción |
|---|---|---|---|
| LUT (total) | 12 814 (72,8 %) | 2 943 (16,7 %) | -77 % |
| FF (total) | 8 575 (24,4 %) | 3 389 (9,6 %) | -60 % |
| DSP (total) | 51 (63,8 %) | 8 (10,0 %) | -84 % |
| RAMB36 (total) | 0 | 2 | +2 |
| **Núcleo Madgwick**: LUT / FF / DSP | 10 850 / 6 638 / 48 | 977 / 1 452 / 5 | -91 % / -78 % / -90 % |
| WNS / WHS | +0,424 ns / +0,012 ns | +0,691 ns / +0,016 ns | |

Resto del sistema (igual en ambas): `nav_spi_ctrl` 990 LUT / 613 FF, SmartConnect ~612 LUT / 709 FF, `raw2float` 348 LUT / 582 FF / 3 DSP.
Con la versión opt2 el Madgwick deja de ser el bloque dominante: el siguiente coste es el SmartConnect y `nav_spi_ctrl`.

Diferencia numérica: opt2 usa rsqrt por Newton-Raphson (error relativo <= 2,4e-7) en vez de los operadores frsqrt/fsqrt; la simulación C (`csim`) con la referencia pasa en ambas versiones.

## Validación con movimientos conocidos (cubo)

Sin máquina de ensayo, se fija la placa a un cubo y se giran 90° entre pausas de ≥3 s, apoyando el cubo en los cantos de la mesa. `analyze_poses.py` detecta las pausas (giroscopio quieto), promedia el cuaternión del último segundo de cada una y mide el ángulo girado entre pausas. Con `--gyro` añade el ángulo que da el giroscopio integrado solo (sin filtro ni acelerómetro).

```
python analysis/analyze_poses.py data/captura_yaw_3.log --nominal 90,90,90,90,90,90,90,90 --gyro
```

Protocolo: `L`, cubo quieto hasta el aviso `LISTO - mueve` (8 s), giro de 90°, quieto hasta el siguiente `LISTO - mueve` (3 s)... Yaw: cubo plano sobre la mesa, 4 giros hacia delante y 4 hacia atrás. Vuelcos: el cubo pasa de cara superior a una cara lateral y vuelve (nunca boca abajo: el cable y la placa quedarían aplastados).

Resultados (placa a mano sobre un cubo; la referencia de 90° tiene una incertidumbre de ±3° por el montaje manual):

| Prueba | Resultado |
|---|---|
| Yaw, 8 giros de 90° (`captura_yaw_3`) | Error del ángulo girado (filtro): media 3,5°, máximo 10,0° (un giro imperfecto). Cierre tras una vuelta completa: 1,9° |
| Original frente a opt2 en todas las pruebas | Idénticos hasta 1e-4° |
| Tasa real de muestreo (`t_us`) | 117,96 Hz (nominal 119 Hz; el filtro integra con dt = 1/119) |
| Escala del giroscopio, eje Z (8 giros, 720°) | 701,0° → 0,974 |
| Escala del giroscopio, eje Y (cara lateral, 720°) | 708,9° → 0,985 |

Hallazgos:
* El giroscopio integrado queda 1,5–2,6 % por debajo del giro real. Es pequeño y no se corrige (podría ser tolerancia de sensibilidad del LSM9DS1 o del montaje). En los vuelcos sobre una arista el déficit aparente fue ≈8 %, pero allí la referencia (el acelerómetro sin calibrar) y los golpes al apoyar el cubo no son fiables: no es una medida de la escala del giroscopio.
* **Arranque con la placa vertical:** tras `L` el filtro empieza en la semilla de placa plana (q = (-1, 0, 0, 0,2)). Si la placa está en una cara lateral, el error de inclinación es de 14° tras 9 s y tarda unos 25 s en converger. Hay que tenerlo en cuenta al analizar (descartar el arranque) o arrancar con la placa plana.
* El `ap_ctrl_hs` con auto-restart captura `reset` y `beta` al arrancar cada invocación: tras escribir `reset = 0`, la siguiente trama aún es de reset, y un cambio de `beta` se aplica dos tramas después. `replay.py` y `analyze_multi.py` lo tienen en cuenta.

## Varias IMUs con un único núcleo

```
Pmod NAV 0..3 -> nav_spi_ctrl_0..3 --AXI-Stream--> axis_rr_mux4 --(TID = IMU)--> r2f_tid --> madgwick_multi
                                                    (VHDL, round-robin por trama)  (HLS)       q_out[4*i..4*i+3], frame_cnt[i], dt_imu[i]
```

* `madgwick_multi` es el núcleo opt2 (mismo programa en ROM, mismos 5 DSP) con un estado `q` por IMU. Cada invocación lee una trama de 9 floats, toma el TID de la primera palabra, actualiza el estado de esa IMU y deja su cuaternión en `q_out`. `reset` es una **máscara** (bit i reinicia la IMU i); `beta` es común y **`dt` es propio de cada IMU** (`dt_imu[4]`, ver más abajo).
* `axis_rr_mux4` (VHDL) reparte por turnos las tramas completas (9 palabras) de las 4 entradas hacia una única salida, añadiendo el TID. Es determinista: una IMU sin datos no bloquea a las demás, y con 1, 2 o 3 IMUs conectadas funciona igual.
* `r2f_tid` convierte los enteros crudos a float (mismo escalado que `raw2float_top`) y copia el TID de entrada a la salida.
* Coste del núcleo: a 100 MHz una actualización tarda ≤ 2 937 ciclos (29,4 µs), es decir, ≈ 3,5 ms de cada 1000 ms por IMU a 119 Hz; el límite teórico del núcleo es de unas 280 IMUs a 119 Hz (cota calculada, no medida). Cada IMU adicional cuesta un `nav_spi_ctrl` y 5 pines.
* Mapa de direcciones: `nav_spi_ctrl_i` en 0x40000000 + 0x10000·i; `madgwick_multi` 0x40040000 (`beta` 0x10, `reset` 0x20, `q_out` 0x40, `frame_cnt` 0x80, **`dt_imu` 0x90**: 4 floats, en segundos); `r2f_tid` 0x40050000 (`fs_g` 0x10). Los offsets del HLS están fijados con `offset=` en los `#pragma`. (En v0.3.0 `dt` era un único registro en 0x18; ya no existe.)
* Pines: IMU0 en JE, IMU1 en JD, IMU2 en JC, IMU3 en JA (ver `zybo_z7_10_pmod_nav_x4.xdc`, pines de `Zybo-Z7-Master.xdc` de Digilent). En la Zybo Z7-10, JB es solo de la Z7-20 y JF está en MIO (PS), sin acceso directo desde la PL; JA es el Pmod del XADC y se usa aquí como E/S digital. En las pruebas, la placa de JA está montada girada 90° respecto a las otras tres.
* Calibración: el sesgo del giroscopio se mide al arrancar con todo quieto (`GYRO_AT_BOOT`, o `GYRO_PRESET`); el hard-iron del magnetómetro depende del montaje, no del arranque, y se fija con `MAG_PRESET`. La tecla `M` calibra los 4 magnetómetros a la vez girando el conjunto (cuerpo rígido); tras cada calibración la UART imprime los presets en formato C.

### `dt` por IMU

Cada LSM9DS1 muestrea con su propio reloj: el ODR nominal es 119 Hz, pero el periodo real de las cuatro placas de las pruebas es 8,280 / 8,307 / 8,480 / 8,057 ms (120,77 / 120,37 / 117,92 / 124,11 Hz), constante dentro de cada captura (variación < 0,02 ms) y estable en un registro de 16 minutos. Con un `dt` fijo de 1/119 s el giroscopio se integraría con un error de escala de +1,5 / +1,2 / −0,9 / +4,3 %. Por eso `dt` es un registro por IMU (`DT_IMU[]` en `main.c`). El periodo se mide con el contador de ciclos de `nav_spi_ctrl` (ver registros de instantánea) y `analyze_multi.py` imprime el valor medio de cada IMU para actualizarlo si se cambian las placas. El acelerómetro y el magnetómetro limitan la deriva del filtro aunque `dt` sea inexacto (por eso la coherencia entre IMUs cambia poco), pero el `dt` correcto importa para la exactitud absoluta del ángulo.

### Por qué un árbitro en VHDL y no en HLS (historia del primer diseño multi-IMU, anterior a la v0.4.0)

La primera versión repartía las 4 entradas con un bloque HLS (`r2f_mux`, lectura no bloqueante en round-robin). En placa funcionaba como rondas sincronizadas con la entrada 3: las IMU 0–2 esperaban en el estado `S_STREAM` hasta 1–2 periodos de muestreo (mediana de 7,9 / 7,1 / 4,0 ms), perdían muestras del sensor (≈15 % de intervalos de 16 ms) y con menos de 4 IMUs no llegaba ninguna trama al núcleo. No se llegó a identificar la causa dentro del RTL generado por HLS; se sustituyó por `axis_rr_mux4.vhd`, validado con un modelo de ciclos en Python (todos los subconjuntos de entradas activas) y en placa. Con el árbitro nuevo la espera máxima en `S_STREAM` es de 7,7–17,6 µs (0,1 µs típico).

### Instantánea hardware en `nav_spi_ctrl` (v2)

Para registrar cada trama sin carreras con la adquisición siguiente, `nav_spi_ctrl` guarda en registros la última trama enviada al núcleo y dos marcas de tiempo de un contador de ciclos de 32 bits a 100 MHz (envuelve cada 42,9 s; es común a todas las IMUs): `0x74` SNAP_SEQ, `0x78` SNAP_TS_GDA (instante del dato nuevo del giroscopio), `0x7C` SNAP_TS_END, `0x80..0xA0` SNAP_W0..W8 (las 9 palabras tal como se enviaron al núcleo), `0xA4` STREAM_LAST y `0xA8` STREAM_MAX (ciclos máximos en `S_STREAM`). `main.c` lee la instantánea de forma atómica (comprueba `SNAP_SEQ` antes y después) y la compara con `frame_cnt` del núcleo; la fila es válida (`ok=1`) solo si coinciden.

### Resultados en placa

Cuatro Pmod NAV (JE, JD, JC, JA), un único núcleo, 100 MHz. Capturas en `data/` (se analizan con `analyze_multi.py`):

| Captura | Duración | Giro acumulado | Pico \|ω\| | Coherencia media (máx.) IMU0–1 / 0–2 / 0–3 |
|---|---|---|---|---|
| `multi_3vueltas.log` | 60 s | ≈ 2 950° (yaw 845°, pitch 1 077°, roll 1 021°) | 332 °/s, sin saturar | 1,6° (6,3) / 2,0° (6,4) / 4,2° (12,4) |
| `multi_5500deg_60s.log` | 60 s | ≈ 5 500° | 538 °/s (5 muestras sobre el fondo de escala de 500 °/s) | 2,8° (8,7) / 5,1° (14,3) / 9,1° (17,7) |
| `multi_5460deg_90s.log` | 90 s | ≈ 5 460° (yaw 1 250°, pitch 1 790°, roll 2 040°) | 457 °/s, sin saturar | 1,9° (6,3) / 3,5° (12,1) / 6,9° (17,0) |
| `multi_reposo_16min.log` | ≈ 16 min | reposo | — | no es un CSV: líneas de estado cada ≈ 0,23 s |

* **Adquisición y reparto**: 0 huecos de muestras del sensor en ninguna IMU (la pérdida de unas 11 filas de 43 000 del 90 s se debe al enlace serie, y el filtro no pierde ninguna trama: `fc − cnt` constante); sin intervalos de 16 ms; espera máxima en `S_STREAM` de 17,6 µs; las cuatro IMUs avanzan a su propio ritmo (120,77 / 120,37 / 117,92 / 124,11 Hz).
* **Equivalencia numérica**: el núcleo coincide con el modelo en float32 (`replay.py`) con una diferencia máxima de 1,7e-4° en todas las capturas (con el `dt` de cada IMU), y con float64 de 1,7e-4°. Esto mide el error de cálculo del núcleo, **no** la exactitud de la orientación.
* **Estabilidad**: en el registro de ≈ 16 minutos, `err=0` y `fc == nav` hasta el final en las 16 376 líneas de estado (en 104 difieren en 1 porque `main.c` lee los dos contadores en instantes distintos); las razones entre las frecuencias de las IMUs se mantienen; con las placas quietas el yaw de cada IMU varía ±0,5° de t=56 s a t=960 s (sin deriva apreciable).
* **Coherencia entre IMUs** (`analyze_multi.py --rigid`): desviación de la rotación relativa IMU0→IMUj respecto a su media, con los cuatro sensores fijados rígidamente a la misma placa. La rotación media IMU0→IMU3 es ≈ 90–94°, que es el giro físico del montaje de JA, y se resta. Es una medida de **consistencia relativa**, no de exactitud absoluta: no hay referencia externa. La desviación crece con la velocidad de giro (más a 540 °/s que a 330 °/s) y se recupera al parar; con las placas quietas al final de la captura de 90 s queda en ≈ 1,2° / 2,2° / 5,1°. Los saltos bruscos que se ven en la gráfica coinciden con golpes del cable USB sobre las placas. Posibles causas del residuo (sin verificar): diferencias de sensibilidad del giroscopio entre placas (≤ 0,8 %), calibración del magnetómetro de cada placa, tolerancia mecánica del montaje.
* **Recursos, timing y potencia**: ver la tabla siguiente.

### Recursos, timing y potencia (4 IMUs, post-implementación)

Sistema completo (PS + 4 `nav_spi_ctrl` v2 + `axis_rr_mux4` + `r2f_tid` + `madgwick_multi` + SmartConnect), xc7z010, reloj de 100 MHz, Vivado 2023.2.1. Informes completos en `Integration_Zybo_multi/reports/` (generados con `report_multi.tcl`).

| Recurso | Usado | Disponible | % |
|---|---|---|---|
| Slice LUT | 7 199 | 17 600 | 40,9 |
| Slice FF | 6 980 | 35 200 | 19,8 |
| DSP48E1 | 8 | 80 | 10,0 |
| RAMB36 | 3 | 60 | 5,0 |

| Bloque | LUT | FF | DSP | RAMB36 |
|---|---|---|---|---|
| `madgwick_multi` (4 estados, 4 `dt`) | 952 | 1 140 | 5 | 3 |
| `nav_spi_ctrl` (cada una, ×4) | 1 239–1 246 | 1 081 | 0 | 0 |
| `r2f_tid` | 368 | 604 | 3 | 0 |
| `axis_rr_mux4` | 45 | 5 | 0 | 0 |
| SmartConnect AXI-Lite | 855 | 874 | 0 | 0 |

* **Timing** (100 MHz): se cumplen todas las restricciones. WNS +0,142 ns, WHS +0,026 ns, sin endpoints fallidos (17 522). Es un margen de setup menor que el de opt2 con una IMU (+0,691 ns): el diseño de 4 IMUs es más grande y no se ha optimizado el timing.
* **Potencia estimada**: 1,568 W en total, de los cuales 1,400 W son el PS7; la lógica programable aporta unos 0,047 W dinámicos y 0,121 W estáticos. La confianza del informe es baja (sin actividad de simulación); úsala solo como orden de magnitud.
* **Coste por IMU**: un `nav_spi_ctrl` (≈ 1 240 LUT / 1 081 FF) y 5 pines; el núcleo Madgwick, `r2f_tid` y el árbitro no crecen de forma apreciable. Las 4 `nav_spi_ctrl` ocupan el 69 % de las LUT del diseño.
* **Coste de la instantánea (v2)**: cada `nav_spi_ctrl` pasó de ≈ 990 LUT / 613 FF (primer diseño multi-IMU) a ≈ 1 240 LUT / 1 081 FF por los registros de instantánea y el contador de ciclos (+250 LUT, +468 FF por IMU). Es instrumentación para registrar y medir; un diseño sin ella sería más pequeño (no se ha sintetizado esa variante).
* **Temperatura**: `nav_spi_ctrl` lee la del LSM9DS1 cada ≈ 1 s en `0xAC` (`TEMP_RAW`, T = 25 + raw/16 °C; solo valen las variaciones) y cuenta las lecturas en `0xB0`. Tras reempaquetar el IP comprueba que `0xB0` crece ≈ 1/s en cada IMU.
* **Modo del magnetómetro**: `MAG_PERF` / `MAG_TEMP_COMP` en `nav_pkg.vhd` (por defecto bajo consumo, como antes). El modo ultra alto reduce el ruido por muestra ≈ 30 %; no cambia la deriva lenta del rumbo.
* **ODR**: `ODR_CODE` en `main.c` (3 = 119 Hz, 4 = 238, 5 = 476, 6 = 952).
* **Captura cruda** (`w` en la UART, `ODR_CODE 3`): `python analysis/capture_raw.py COM5 921600 captura_raw.csv --minutes 10`. Cada fila lleva las 9 palabras que recibió el núcleo, su cuaternión y la temperatura. `python analysis/replay_variants.py captura_raw.csv --png fig.png` repite el filtro en el PC (con y sin magnetómetro) y comprueba que coincide con el núcleo.
* **Visor en vivo** (`s`): `python analysis/viewer_live.py --port COM5 --baud 921600` (teclas T tara, A eje de la bisagra, Q salir).
* **Calibración elipsoidal**: con la IMU suelta, gira por todas las orientaciones durante 30–60 s en una captura cruda y ejecuta `python analysis/mag_ellipsoid.py captura_raw.csv --imu 1`. Da el offset (para `MAG_PRESET`) y la matriz soft-iron, con validación cruzada. La matriz todavía no se aplica en el firmware.

### Reproducir

```
cd HLS_madgwick_multi && vitis_hls -f run_hls_r2f_tid.tcl && vitis_hls -f run_hls_madgwick_multi.tcl
cd ../PMOD_NAV_Ctrller && vivado -mode batch -source package_nav_ip.tcl      # nav_spi_ctrl v2
cd ../Integration_Zybo_multi && vivado -mode batch -source create_project.tcl
vivado -mode batch -source build_project.tcl     # exporta zybo_multi.xsa
vivado -mode batch -source report_multi.tcl      # timing / utilizacion / potencia en ./reports/
python analysis/analyze_multi.py data/multi_5460deg_90s.log --rigid --png fig_multi.png
```
Prueba sin placa: `cd HLS_madgwick_multi && ./test_pc.sh` (4 IMUs sintéticas con cadencias y `dt` distintos, comparación bit a bit con 4 núcleos de una sola IMU: 9 600 valores, 0 diferencias).

Teclas del `main.c` multi: `0..3` selecciona IMU, `g` gyro (todas), `m` mag (la seleccionada), `M` mag (todas a la vez), `c` borrar offsets, `r` reset, `a`/`b` ejes 0x018/0x039, `l`/`L` captura (`LOG_SECONDS`, todas las IMUs; `l` conserva el estado del filtro, `L` lo reinicia), `d` volcar de nuevo. Ajusta al principio de `main.c`: `N_IMU` (1..4), `DT_IMU[]`, `GYRO_PRESET`/`MAG_PRESET` y `LOG_SECONDS`. La captura se guarda en RAM y se vuelca por UART al terminar, así que el cable solo hace falta para alimentar la placa y lanzarla. Guarda la salida del terminal en un fichero (Tera Term: File > Log).

## Conexión del Pmod NAV (JE)

| Pmod NAV | Zybo (JE) | Pin |
|---|---|---|
| CS_AG | JE1 | V12 |
| MOSI | JE2 | W16 |
| MISO | JE3 | J15 |
| SCK | JE4 | H15 |
| CS_M | JE9 | T17 |

`CS_ALT` (JE10, Y17) debe quedar a nivel alto (el XDC pone `BITSTREAM.CONFIG.UNUSEDPIN PULLUP`). SPI a 2,5 MHz.

## Uso y calibración

Al arrancar, `main.c` lee WHO_AM_I (A/G `0x68`, mag `0x3D`), calibra el sesgo del giróscopo (placa quieta, ~3 s) y la **calibración del magnetómetro** (30 s girando la placa por todas las orientaciones; ajuste de esfera con rechazo de picos y atípicos). Después detecta automáticamente la orientación de ejes. Avisa con pitidos (BEL en la terminal) y el LED.

Teclas por UART: `g` recalibrar gyro, `m` recalibrar mag, `c` borrar offsets, `r` reiniciar filtro, `1`/`2` elegir `AXIS_CFG` 0x018 / 0x039.

Valores validados para esta placa y montaje:
* `AXIS_CFG = 0x039` (mag: X negada; gyro: tres ejes negados).
* Rango del giróscopo 500 dps (`FS_G_SELECTED = 1`, igual en `nav_spi_ctrl` y `raw2float`).
* Los offsets del magnetómetro dependen del lugar y del montaje: vuelve a calibrar con `m` si cambian.

Registros de `nav_spi_ctrl` (base `0x40000000`): `0x00` CTRL, `0x04` ODR_CFG, `0x08` RANGE_CFG, `0x0C` APPLY_CFG, `0x20..0x28` offset gyro, `0x2C..0x34` offset mag, `0x38` AXIS_CFG, `0x40` STATUS, `0x44` SAMPLE_CNT, `0x48` WHO_AM_I, `0x50..0x70` muestra cruda; desde v0.4.0 también `0x74..0xA8` (instantánea, ver «Varias IMUs»).

## Estado

* Prueba de concepto validada: adquisición a 119 Hz, ejes, calibración, escala del giróscopo (giro de 360°) e independencia del yaw respecto a la inclinación.
* Versión opt2 verificada en placa: sin errores, |q|² = 1 en todas las lecturas, 119 Hz, giro de 360° completo sin saltos. La primera lectura tras el arranque muestra el estado inicial del filtro, q = (-1, 0, 0, 0,2), con |q|² = 1,04; es el valor de semilla del diseño, igual que en la versión original.
* Sistema dual (original + opt2 con la misma entrada) verificado en placa: diferencias ≤ 1e-4° respecto a doble precisión y entre ellos, sin acumulación.
* **v0.4.0**: diseño de 1 a 4 IMUs con un único núcleo verificado en placa (árbitro VHDL, `dt` por IMU, instantánea hardware): sin esperas en el stream, sin pérdida de muestras, error numérico ≤ 1,7e-4°, 16 minutos sin errores ni deriva. Con 4 IMUs: 7 199 LUT (40,9 %), 6 980 FF, 8 DSP, 3 RAMB36; timing cumplido (WNS +0,142 ns a 100 MHz).
* v0.4.1: lectura de la temperatura del chip cada 128 tramas (sin afectar al flujo), modo de alto rendimiento del magnetómetro, ODR de 119 a 952 Hz con 4 IMUs (carga del núcleo de 1,4 % a 11,4 %, sin tramas perdidas) y captura cruda para análisis fuera de línea.
* v0.5.0: línea base del filtro en el ARM (Software_Baseline_ARM): 0,82 µs por actualización con -O3 (0,010 % de una CPU por sensor a 119 Hz); la ventaja de la cadena en hardware no es la velocidad del filtro sino sacar al procesador de la adquisición. Calibración elipsoidal del magnetómetro (analysis/mag_ellipsoid.py).
* v0.6.0: línea base de extremo a extremo (`Integration_Zybo_swspi`: el ARM adquiere por SPI, filtra y escribe por la UART). Con 2 IMU a 119 Hz ocupa el 1,7 % de la CPU, a 952 Hz el 11,2 % y pierde ≈ 55 % de las tramas con la UART a 115200; la cadena en la PL no usa CPU ni pierde tramas, con latencia propia mayor (135 µs frente a 71 µs a 119 Hz). Datos en `data/e2e/`.
* v0.6.1: la línea base de extremo a extremo con 4 IMU. En el ARM (SCK 3,125 MHz, sondeo) la cadena ocupa el 3,4 % de la CPU a 119 Hz y el 22,4 % a 952 Hz; la cadena en la PL deja la CPU libre al 100 % y no pierde tramas. Datos en `data/e2e/`.
* v0.6.2: línea base en el ARM con interrupciones (`main_irq.c`, temporizador privado + GIC). Con 4 IMU deja libre el 95,8 % de la CPU a 119 Hz y el 71,1 % a 952 Hz (28,8 % ocupada), sin perder tramas con una línea por trama por la UART; la cadena en la PL sigue sin usar CPU. Datos en `data/e2e/`.
* v0.6.3: criterio de tramas perdidas de `main_irq.c` corregido (0 tramas perdidas en todas las fases con 4 IMU a 119 y 952 Hz); la entrada de v0.3.0 del CHANGELOG ahora describe el sistema dual de la etiqueta.
* Pendiente: calibración del magnetómetro de la IMU de JA (mayor residuo de coherencia); capturas sin cable y con referencia externa.

## Licencia
MIT, ver `LICENSE`.
