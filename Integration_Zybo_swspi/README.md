# Línea base en software de extremo a extremo (borrador, v0.6)

**Estado: preparado pero NO ejecutado en Vivado ni en la placa.** `main.c` se ha probado en PC contra un modelo del AXI Quad SPI y del LSM9DS1 (`test/`); el diseño de bloques puede necesitar algún retoque (ver «Antes de sintetizar»).

## Qué mide y por qué

La medida de `Software_Baseline_ARM/` (v0.5.0) dice que el **filtro** cuesta 0,82 µs en el ARM. No dice cuánto tarda una cadena de software completa. Aquí el ARM hace todo, como el sistema de Galindo: lee los LSM9DS1 por SPI, convierte, filtra y saca el resultado por la UART. En la PL solo hay un **AXI Quad SPI por Pmod NAV** (2 chip selects, modo SPI 3, SCK = 100 MHz / 32 = 3,125 MHz; la cadena en hardware usa 2,5 MHz), sin `nav_spi_ctrl`, árbitro ni núcleo.

Con las mismas placas, la misma ODR (119 Hz) y el mismo filtro, `main.c` mide por IMU y en tres fases de 30 s (sin salida, una línea cada 12 tramas = 10 Hz, una línea por trama):

* tiempo de las lecturas SPI de una trama, del filtro, y de la **detección del dato nuevo al cuaternión**;
* intervalo entre tramas (jitter) y **tramas perdidas**;
* fracción de CPU de SPI + filtro (con sondeo, la CPU está ocupada el 100 % mientras tanto).

Se espera que la lectura SPI (decenas de µs) domine sobre el filtro (µs) y que, con salida por la UART a 115 200 baud (≈ 9 ms por línea de 100 caracteres), aparezcan tramas perdidas en la fase C. Lo que salga es lo que se publica.

## Pasos

1. `vivado -mode batch -source create_project.tcl` (cambia `NIMU` si no usas 2 placas). Pines: IMU0 en JE, IMU1 en JD (los mismos que `Integration_Zybo_multi`).
2. **Antes de sintetizar**: abre `zybo_swspi/zybo_swspi.gen/sources_1/bd/system/hdl/system_wrapper.v` y comprueba que los puertos se llaman `spi_<i>_io0_io` (MOSI), `spi_<i>_io1_io` (MISO), `spi_<i>_sck_io` y `spi_<i>_ss_io[1:0]` (CS_AG, CS_M). Si no, ajusta `zybo_z7_10_swspi.xdc`. Comprueba también en el block design que el AXI Quad SPI está en modo estándar con 2 SS y SCK ratio 32.
3. `source build_project.tcl` → `zybo_swspi.xsa`. En Vitis: plataforma desde el `.xsa`, app bare-metal con `vitis_app/main.c`. UART a 115 200 8N1. Si la dirección de los AXI Quad SPI no es `0x40000000 + 0x10000·i`, cambia `QSPI_BASE`.
4. Compila en **Release (-O3)** (igual que la línea base del filtro) y ejecuta con las placas **quietas** (se mide el sesgo del giroscopio al arrancar).

## Notas de diseño

* Los registros del AXI Quad SPI se manejan directamente (sin el driver `XSpi`): es el límite inferior de tiempo del software; con el driver sería peor. Mismo orden de lectura que la máquina de estados de la PL (estado A/G → giro → acel → estado mag → mag).
* La configuración del sensor, los ejes (`AXIS_CFG 0x039`), el `MAG_PRESET`, el sesgo del giroscopio al arrancar y el `dt` por IMU son los de la cadena en hardware, para que los cuaterniones sean comparables.
* El modelo de PC (`test/`): `gcc -O2 -DHOST -DPHASE_SECONDS=2 -ffp-contract=off -Itest vitis_app/main.c test/host_sim.c -o sim -lm && ./sim`. Los tiempos de PC no valen.

## Resultado a anotar para el artículo

| | Cadena en hardware | Software (ARM, sondeo) |
|---|---|---|
| Lectura SPI de una trama (µs) | ≈ 100 (en la PL, sin CPU) | media / máx. |
| Filtro (µs) | 29,4 (máx.) | media / máx. |
| Detección → cuaternión (µs) | media / máx. | media / máx. |
| Intervalo entre tramas: min / máx. (µs) | — | min / máx. |
| Tramas perdidas (fase A / B / C) | 0 | n / n / n |
| CPU en la ruta de datos | no | sí (100 % con sondeo) |
