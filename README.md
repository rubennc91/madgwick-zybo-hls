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
| `HLS_madgwick_stream_opt/` | Misma función con operadores serializados (menos DSP/LUT). Bit-exacta con la anterior en simulación; síntesis pendiente de medir |
| `integration_zybo/` | `create_project.tcl`, `build_project.tcl`, XDC y `vitis_app/main.c` |

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
5. **Aplicación**: en Vitis crea una plataforma desde el `.xsa` (rehazla cada vez que cambie), una app standalone vacía y copia `integration_zybo/vitis_app/main.c`. Añade `m` a las librerías del linker si falla `atan2f/sqrtf`.
   UART a 115200 8N1.

### Sin Vivado/HLS: binarios precompilados
En **Releases** hay `system_wrapper.xsa` (con bitstream), `system_wrapper.bit` y `app_component.elf` listos para programar la placa.

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
* Siguiente: optimización de recursos (LUT 69 %, DSP 64 % antes de la versión serializada) y soporte de varias IMU con un único núcleo Madgwick.

## Licencia
MIT, ver `LICENSE`.
