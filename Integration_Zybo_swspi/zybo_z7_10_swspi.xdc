# Pmod NAV en la Zybo Z7-10 para la linea base en software (AXI Quad SPI). Mismos pines que zybo_z7_10_pmod_nav_x4.xdc.
# Puertos generados por make_bd_intf_pins_external: spi_<i>_io0_io = MOSI, spi_<i>_io1_io = MISO, spi_<i>_sck_io, spi_<i>_ss_io[0] = CS_AG, [1] = CS_M.
# SI VIVADO USA OTROS NOMBRES: abre system_wrapper.v y ajusta esta lista. Solo se necesitan NIMU Pmod; borra los de las IMU que no uses.

# ---- IMU0: Pmod JE ----
set_property PACKAGE_PIN V12 [get_ports {spi_0_ss_io[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_0_ss_io[0]}]
set_property PACKAGE_PIN W16 [get_ports {spi_0_io0_io}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_0_io0_io}]
set_property PACKAGE_PIN J15 [get_ports {spi_0_io1_io}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_0_io1_io}]
set_property PACKAGE_PIN H15 [get_ports {spi_0_sck_io}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_0_sck_io}]
set_property PACKAGE_PIN T17 [get_ports {spi_0_ss_io[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_0_ss_io[1]}]

# ---- IMU1: Pmod JD ----
set_property PACKAGE_PIN T14 [get_ports {spi_1_ss_io[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_1_ss_io[0]}]
set_property PACKAGE_PIN T15 [get_ports {spi_1_io0_io}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_1_io0_io}]
set_property PACKAGE_PIN P14 [get_ports {spi_1_io1_io}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_1_io1_io}]
set_property PACKAGE_PIN R14 [get_ports {spi_1_sck_io}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_1_sck_io}]
set_property PACKAGE_PIN V17 [get_ports {spi_1_ss_io[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_1_ss_io[1]}]

# ---- IMU2: Pmod JC ----
#set_property PACKAGE_PIN V15 [get_ports {spi_2_ss_io[0]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {spi_2_ss_io[0]}]
#set_property PACKAGE_PIN W15 [get_ports {spi_2_io0_io}]
#set_property IOSTANDARD LVCMOS33 [get_ports {spi_2_io0_io}]
#set_property PACKAGE_PIN T11 [get_ports {spi_2_io1_io}]
#set_property IOSTANDARD LVCMOS33 [get_ports {spi_2_io1_io}]
#set_property PACKAGE_PIN T10 [get_ports {spi_2_sck_io}]
#set_property IOSTANDARD LVCMOS33 [get_ports {spi_2_sck_io}]
#set_property PACKAGE_PIN T12 [get_ports {spi_2_ss_io[1]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {spi_2_ss_io[1]}]

# ---- IMU3: Pmod JA (XADC, E/S digital) ----
#set_property PACKAGE_PIN N15 [get_ports {spi_3_ss_io[0]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {spi_3_ss_io[0]}]
#set_property PACKAGE_PIN L14 [get_ports {spi_3_io0_io}]
#set_property IOSTANDARD LVCMOS33 [get_ports {spi_3_io0_io}]
#set_property PACKAGE_PIN K16 [get_ports {spi_3_io1_io}]
#set_property IOSTANDARD LVCMOS33 [get_ports {spi_3_io1_io}]
#set_property PACKAGE_PIN K14 [get_ports {spi_3_sck_io}]
#set_property IOSTANDARD LVCMOS33 [get_ports {spi_3_sck_io}]
#set_property PACKAGE_PIN J16 [get_ports {spi_3_ss_io[1]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {spi_3_ss_io[1]}]

set_property BITSTREAM.CONFIG.UNUSEDPIN PULLUP [current_design]
