# madgwick-zybo-hls

Filtro de orientación **Madgwick (MARG)** completamente en FPGA para una **Zybo Z7-10** (xc7z010clg400-1),
con un **Pmod NAV** (LSM9DS1 + LPS25HB) en el conector **JE**.
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
| `HLS_madgwick_multi/` | **Un solo núcleo para varias IMUs**: `madgwick_multi` (opt2 con un estado `q` por IMU, identificadas por TID) y `r2f_mux` (raw2float + reparto round-robin de 4 entradas). Incluye banco de pruebas en C (`test_pc.sh`) |
| `Integration_Zybo_multi/` | Integración de **4 Pmod NAV** (JE, JD, JC, JB) con un único núcleo Madgwick (proyecto `zybo_multi`) |
| `Integration_Zybo_dual/` | **Los dos núcleos a la vez** (original y opt2) alimentados con la misma trama por un `axis_broadcaster`; `main.c` compara sus cuaterniones y captura un CSV para el análisis offline |

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
Pmod NAV 0..3 -> nav_spi_ctrl_0..3 --AXI-Stream--> r2f_mux --(TID = IMU)--> madgwick_multi
                                                       (raw2float + round-robin)   q_out[4*i..4*i+3], frame_cnt[i]
```

* `madgwick_multi` es el núcleo opt2 (mismo programa en ROM, mismos 5 DSP) con un estado `q` por IMU. Cada invocación lee una trama de 9 floats, toma el TID de la primera palabra, actualiza el estado de esa IMU y deja su cuaternión en `q_out`. `reset` es una **máscara** (bit i reinicia la IMU i); `beta` y `dt` son comunes.
* `r2f_mux` convierte los enteros crudos a float (mismo escalado que `raw2float_top`) y reparte por turnos las tramas de 4 entradas hacia una única salida con TID.
* Coste: a 100 MHz una actualización tarda ≤ 2 937 ciclos (29,4 µs), es decir, ≈ 3,5 ms de cada 1000 ms por IMU a 119 Hz; el límite teórico del núcleo es de unas 280 IMUs a 119 Hz (cota teórica; no medida). Cada IMU adicional cuesta un `nav_spi_ctrl` y 5 pines.
* Mapa de direcciones: `nav_spi_ctrl_i` en 0x40000000 + 0x10000·i; `madgwick_multi` 0x40040000 (`beta` 0x10, `dt` 0x18, `reset` 0x20, `q_out` 0x40, `frame_cnt` 0x80); `r2f_mux` 0x40050000 (`fs_g` 0x10). Los offsets del HLS están fijados con `offset=` en los `#pragma`.
* Pines: IMU0 en JE, IMU1 en JD, IMU2 en JC, IMU3 en JB (ver `zybo_z7_10_pmod_nav_x4.xdc`; comprueba los pines con la hoja maestra de Digilent antes de conectar).

Prueba sin placa: `cd HLS_madgwick_multi && ./test_pc.sh`. Genera 4 IMUs sintéticas con cadencias distintas, las pasa por el mux y compara la salida de `madgwick_multi` con 4 núcleos de una sola IMU: 9 600 valores, 0 diferencias (bit a bit).

Reproducir:
```
cd HLS_madgwick_multi && vitis_hls -f run_hls_r2f_mux.tcl && vitis_hls -f run_hls_madgwick_multi.tcl
cd ../Integration_Zybo_multi && vivado -mode batch -source create_project.tcl
vivado -mode batch -source build_project.tcl     # exporta zybo_multi.xsa
python analysis/analyze_multi.py data/captura_multi.log --rigid --png fig_multi.png
```
Teclas del `main.c` multi: `0..3` selecciona IMU, `g` gyro (todas), `m` mag (la seleccionada), `c` borrar offsets, `r` reset, `a`/`b` ejes 0x018/0x039, `l`/`L` captura (30 s, todas las IMUs), `d` volcar de nuevo.

> Estado: simulado en C (bit a bit) y con análisis offline probado con datos sintéticos; **pendiente de probar en placa**.

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

Registros de `nav_spi_ctrl` (base `0x40000000`): `0x00` CTRL, `0x04` ODR_CFG, `0x08` RANGE_CFG, `0x0C` APPLY_CFG, `0x20..0x28` offset gyro, `0x2C..0x34` offset mag, `0x38` AXIS_CFG, `0x40` STATUS, `0x44` SAMPLE_CNT, `0x48` WHO_AM_I, `0x50..0x70` muestra cruda.

## Estado

* Prueba de concepto validada: adquisición a 119 Hz, ejes, calibración, escala del giróscopo (giro de 360°) e independencia del yaw respecto a la inclinación.
* Versión opt2 verificada en placa: sin errores, |q|² = 1 en todas las lecturas, 119 Hz, giro de 360° completo sin saltos. La primera lectura tras el arranque muestra el estado inicial del filtro, q = (-1, 0, 0, 0,2), con |q|² = 1,04; es el valor de semilla del diseño, igual que en la versión original.
* Sistema dual (original + opt2 con la misma entrada) verificado en placa: diferencias ≤ 1e-4° respecto a doble precisión y entre ellos, sin acumulación.
* Siguiente: probar en placa el diseño de 4 IMUs con un núcleo (`Integration_Zybo_multi`) y medir cuántas IMUs soporta el núcleo.

## Licencia
MIT, ver `LICENSE`.
