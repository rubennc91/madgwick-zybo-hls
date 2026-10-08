## -----------------------------------------------------------------------
## zybo_z7_10_pmod_nav.xdc
##
## Pmod NAV (LSM9DS1) conectado al conector JE de la Zybo Z7-10.
## Pinout del Pmod NAV (tabla de Digilent, cabecera J1, 12 pines):
##   1 CS_A/G   2 SDI(MOSI)   3 SDO(MISO)   4 SCK    5 GND   6 VCC
##   7 INT      8 DRDY_M      9 CS_M       10 CS_ALT 11 GND  12 VCC
## Pines de JE en la Zybo Z7-10 (manual de referencia):
##   1 V12   2 W16   3 J15   4 H15   5 GND   6 VCC
##   7 V13   8 U17   9 T17  10 Y17  11 GND  12 VCC
## Como los dos conectores tienen la misma numeracion, la correspondencia
## es directa pin a pin.
## -----------------------------------------------------------------------

## --- CS_A/G  (Pmod pin 1 -> JE1) ------------------------------------------
set_property PACKAGE_PIN V12  [get_ports {spi_cs_n_0[0]}]
set_property IOSTANDARD  LVCMOS33 [get_ports {spi_cs_n_0[0]}]

## --- MOSI = SDI (Pmod pin 2 -> JE2) ---------------------------------------
set_property PACKAGE_PIN W16  [get_ports spi_mosi_0]
set_property IOSTANDARD  LVCMOS33 [get_ports spi_mosi_0]

## --- MISO = SDO (Pmod pin 3 -> JE3) ---------------------------------------
set_property PACKAGE_PIN J15  [get_ports spi_miso_0]
set_property IOSTANDARD  LVCMOS33 [get_ports spi_miso_0]

## --- SCLK = SCK (Pmod pin 4 -> JE4) ---------------------------------------
set_property PACKAGE_PIN H15  [get_ports spi_sclk_0]
set_property IOSTANDARD  LVCMOS33 [get_ports spi_sclk_0]

## --- CS_M (Pmod pin 9 -> JE9) ---------------------------------------------
set_property PACKAGE_PIN T17  [get_ports {spi_cs_n_0[1]}]
set_property IOSTANDARD  LVCMOS33 [get_ports {spi_cs_n_0[1]}]

## --- CS_ALT (Pmod pin 10 -> JE10, Y17): altimetro LPS25HB, NO usado ------------
## Si CS_ALT queda flotante/a nivel bajo, el LPS25HB (WHO_AM_I = 0xBD) esta
## seleccionado a la vez que el LSM9DS1 y ambos pelean por MISO: 0x68 & 0xBD = 0x28.
## Vivado deja por defecto los pines sin usar en pull-down (UNUSEDPIN = Pulldown),
## lo que mantiene CS_ALT activo (CS es activo bajo). Con pull-up queda deshabilitado.
set_property BITSTREAM.CONFIG.UNUSEDPIN PULLUP [current_design]
