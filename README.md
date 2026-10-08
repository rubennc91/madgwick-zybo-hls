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
* Siguiente: soporte de varias IMU con un único núcleo Madgwick (estado q por IMU) y reducción del resto del sistema (SmartConnect, `nav_spi_ctrl`).

## Licencia
MIT, ver `LICENSE`.
