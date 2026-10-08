-------------------------------------------------------------------------------
-- nav_pkg.vhd
-- Constantes y tipos comunes para el controlador SPI del LSM9DS1 (Pmod NAV).
-- Direcciones de registro sacadas del datasheet ST LSM9DS1 y verificadas
-- contra las macros del driver de Digilent (PmodNAV.h).
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

package nav_pkg is

    ---------------------------------------------------------------------
    -- Selector de subdispositivo SPI (comparten MISO/MOSI/SCLK, CS propio)
    ---------------------------------------------------------------------
    constant CS_AG  : integer := 0;   -- Acelerometro + giroscopio
    constant CS_MAG : integer := 1;   -- Magnetometro

    ---------------------------------------------------------------------
    -- Direcciones de registro (6 bits, sin los bits RW/MS -- el byte de
    -- direccion SPI del LSM9DS1 es RW(1) & MS(1) & ADDR(6), 8 bits total)
    ---------------------------------------------------------------------
    -- Accel + Gyro
    constant REG_AG_WHO_AM_I    : std_logic_vector(5 downto 0) := "001111"; -- 0x0F
    constant REG_AG_CTRL_REG1_G : std_logic_vector(5 downto 0) := "010000"; -- 0x10
    constant REG_AG_STATUS_REG  : std_logic_vector(5 downto 0) := "010111"; -- 0x17
    constant REG_AG_OUT_X_L_G   : std_logic_vector(5 downto 0) := "011000"; -- 0x18
    constant REG_AG_CTRL_REG6_XL: std_logic_vector(5 downto 0) := "100000"; -- 0x20
    constant REG_AG_CTRL_REG8   : std_logic_vector(5 downto 0) := "100010"; -- 0x22
    constant REG_AG_OUT_X_L_XL  : std_logic_vector(5 downto 0) := "101000"; -- 0x28

    -- Magnetometro
    constant REG_M_WHO_AM_I     : std_logic_vector(5 downto 0) := "001111"; -- 0x0F
    constant REG_M_CTRL_REG1_M  : std_logic_vector(5 downto 0) := "100000"; -- 0x20
    constant REG_M_CTRL_REG3_M  : std_logic_vector(5 downto 0) := "100010"; -- 0x22
    constant REG_M_CTRL_REG5_M  : std_logic_vector(5 downto 0) := "100100"; -- 0x24
    constant REG_M_STATUS_REG_M : std_logic_vector(5 downto 0) := "100111"; -- 0x27
    constant REG_M_OUT_X_L_M    : std_logic_vector(5 downto 0) := "101000"; -- 0x28

    -- Valores de WHO_AM_I esperados (datasheet LSM9DS1)
    constant WHOAMI_AG_EXPECTED : std_logic_vector(7 downto 0) := x"68";
    constant WHOAMI_M_EXPECTED  : std_logic_vector(7 downto 0) := x"3D";

    -- Bits R/W y MS del byte de direccion SPI (formato LSM9DS1)
    constant SPI_RW_READ  : std_logic := '1';
    constant SPI_RW_WRITE : std_logic := '0';
    constant SPI_MS_INC   : std_logic := '1';  -- auto-incremento para rafaga
    constant SPI_MS_SINGLE: std_logic := '0';

    ---------------------------------------------------------------------
    -- Tipo de operacion del micro-secuenciador de configuracion
    ---------------------------------------------------------------------
    type op_kind_t is (OP_WRITE_STATIC, OP_WRITE_DYNAMIC, OP_DONE);

    type cfg_op_t is record
        kind : op_kind_t;
        cs   : integer range 0 to 1;
        addr : std_logic_vector(5 downto 0);
        data : std_logic_vector(7 downto 0); -- solo valido si kind = OP_WRITE_STATIC
    end record;

    type cfg_program_t is array (natural range <>) of cfg_op_t;

    -- Programa de configuracion, en orden de ejecucion.
    -- Los pasos OP_WRITE_DYNAMIC toman el byte de datos de un registro
    -- AXI-Lite (ver nav_spi_ctrl.vhd, proceso de configuracion) en vez
    -- del campo "data" de esta tabla.
    constant CFG_PROGRAM : cfg_program_t(0 to 5) := (
        0 => (OP_WRITE_DYNAMIC, CS_AG,  REG_AG_CTRL_REG1_G,  (others => '0')), -- ODR_G  + FS_G
        1 => (OP_WRITE_DYNAMIC, CS_AG,  REG_AG_CTRL_REG6_XL, (others => '0')), -- ODR_XL + FS_XL
        2 => (OP_WRITE_STATIC,  CS_AG,  REG_AG_CTRL_REG8,    x"44"),           -- BDU=1, IF_ADD_INC=1
        3 => (OP_WRITE_DYNAMIC, CS_MAG, REG_M_CTRL_REG1_M,   (others => '0')), -- ODR_M
        4 => (OP_WRITE_STATIC,  CS_MAG, REG_M_CTRL_REG3_M,   x"00"),           -- modo continuo
        5 => (OP_WRITE_STATIC,  CS_MAG, REG_M_CTRL_REG5_M,   x"40")            -- BDU_M=1
    );

    ---------------------------------------------------------------------
    -- Mapa de registros AXI-Lite (offsets de palabra de 32 bits)
    ---------------------------------------------------------------------
    -- Control (R/W)
    constant REGOFF_CTRL      : integer := 16#00#; -- bit0 ENABLE, bit1 SOFT_RESET(pulso),
                                                     -- bit2 IRQ_EN, bit3 CLEAR_ERR(pulso)
    constant REGOFF_ODR_CFG   : integer := 16#04#; -- [2:0]=ODR_G [5:3]=ODR_XL [8:6]=ODR_M
    constant REGOFF_RANGE_CFG : integer := 16#08#; -- [1:0]=FS_G  [3:2]=FS_XL
    constant REGOFF_APPLY_CFG : integer := 16#0C#; -- bit0: pulso -> re-ejecuta CFG_PROGRAM

    -- Estado (solo lectura; bits de error son "sticky", se limpian con CLEAR_ERR)
    constant REGOFF_STATUS      : integer := 16#40#;
    constant STAT_BIT_BUSY         : integer := 0;
    constant STAT_BIT_CONFIG_DONE  : integer := 1;
    constant STAT_BIT_AG_DETECTED  : integer := 2;
    constant STAT_BIT_MAG_DETECTED : integer := 3;
    constant STAT_BIT_ERROR        : integer := 4; -- OR de los siguientes
    constant STAT_BIT_ERR_AG_WHOAMI : integer := 5;
    constant STAT_BIT_ERR_MAG_WHOAMI: integer := 6;
    constant STAT_BIT_ERR_TIMEOUT   : integer := 7;
    constant STAT_BIT_ERR_OVERRUN   : integer := 8;

    constant REGOFF_SAMPLE_COUNT : integer := 16#44#; -- contador libre de muestras completadas

end package nav_pkg;
