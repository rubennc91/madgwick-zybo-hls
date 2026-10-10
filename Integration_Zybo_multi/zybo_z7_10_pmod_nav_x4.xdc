# Pmod NAV x4 en la Zybo Z7-10. Cada Pmod usa los pines 1 (CS_AG), 2 (MOSI), 3 (MISO), 4 (SCLK) y 9 (CS_M)
# de su conector. IMU0 = JE (igual que en los otros proyectos), IMU1 = JD, IMU2 = JC, IMU3 = JA (XADC; se usa como E/S digital).
# JB es solo de la Zybo Z7-20 (el banco 13 no existe en la Z7-10) y JF es de MIO (PS): no llegan a la PL.
# Pines de Zybo-Z7-Master.xdc (Digilent).
# Los Pmod NAV se alimentan a 3,3 V del propio conector (LVCMOS33).

# ---- IMU0: Pmod JE ----
set_property PACKAGE_PIN V12  [get_ports {spi_cs_n_0[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_cs_n_0[0]}]
set_property PACKAGE_PIN W16  [get_ports {spi_mosi_0}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_mosi_0}]
set_property PACKAGE_PIN J15  [get_ports {spi_miso_0}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_miso_0}]
set_property PACKAGE_PIN H15  [get_ports {spi_sclk_0}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_sclk_0}]
set_property PACKAGE_PIN T17  [get_ports {spi_cs_n_0[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_cs_n_0[1]}]

# ---- IMU1: Pmod JD ----
set_property PACKAGE_PIN T14  [get_ports {spi_cs_n_1[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_cs_n_1[0]}]
set_property PACKAGE_PIN T15  [get_ports {spi_mosi_1}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_mosi_1}]
set_property PACKAGE_PIN P14  [get_ports {spi_miso_1}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_miso_1}]
set_property PACKAGE_PIN R14  [get_ports {spi_sclk_1}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_sclk_1}]
set_property PACKAGE_PIN V17  [get_ports {spi_cs_n_1[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_cs_n_1[1]}]

# ---- IMU2: Pmod JC ----
set_property PACKAGE_PIN V15  [get_ports {spi_cs_n_2[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_cs_n_2[0]}]
set_property PACKAGE_PIN W15  [get_ports {spi_mosi_2}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_mosi_2}]
set_property PACKAGE_PIN T11  [get_ports {spi_miso_2}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_miso_2}]
set_property PACKAGE_PIN T10  [get_ports {spi_sclk_2}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_sclk_2}]
set_property PACKAGE_PIN T12  [get_ports {spi_cs_n_2[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_cs_n_2[1]}]

# ---- IMU3: Pmod JA (XADC) ----
set_property PACKAGE_PIN N15  [get_ports {spi_cs_n_3[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_cs_n_3[0]}]
set_property PACKAGE_PIN L14  [get_ports {spi_mosi_3}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_mosi_3}]
set_property PACKAGE_PIN K16  [get_ports {spi_miso_3}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_miso_3}]
set_property PACKAGE_PIN K14  [get_ports {spi_sclk_3}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_sclk_3}]
set_property PACKAGE_PIN J16  [get_ports {spi_cs_n_3[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_cs_n_3[1]}]

set_property BITSTREAM.CONFIG.UNUSEDPIN PULLUP [current_design]
