-------------------------------------------------------------------------------
-- nav_spi_ctrl.vhd
-- Controlador autonomo del Pmod NAV (LSM9DS1): configura el sensor, sondea
-- DRDY y saca muestras en bruto (gx,gy,gz,ax,ay,az,mx,my,mz) por AXI4-Stream,
-- sin intervencion del PS en el bucle de muestreo.
--
-- Interfaces:
--   - S_AXI_*   : AXI4-Lite esclavo (control + estado, ver nav_pkg.vhd)
--   - M_AXIS_*  : AXI4-Stream maestro, TDATA de 32 bits, 9 palabras por
--                 muestra (word 0..8 = gx,gy,gz,ax,ay,az,mx,my,mz en cuentas
--                 crudas de 16 bits con signo, extendidas a 32), TLAST en
--                 la novena palabra.
--   - spi_*     : bus fisico compartido hacia el Pmod (2 CS: AG y MAG)
--   - irq       : pulso/nivel de interrupcion (opcional, ver mas abajo)
--
-- NOTA: la conversion de cuentas crudas a unidades fisicas (dps, g, gauss)
-- queda pendiente aguas abajo (o bien en el propio nucleo Madgwick si se
-- adapta a punto fijo, o en un bloque intermedio). Aqui solo se entregan
-- las cuentas crudas, ya que el factor de escala depende del rango (FS_G/
-- FS_XL) configurado y conviene mantenerlo en un unico sitio.
--
-- AVISO: primer borrador, sin simular. Antes de sintetizar, verificarlo con
-- un testbench (GHDL o el simulador de Vivado) y con un analizador logico
-- en las primeras pruebas en placa.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.nav_pkg.all;

entity nav_spi_ctrl is
    generic (
        G_CLK_FREQ_HZ    : integer := 100_000_000;
        G_SPI_CLK_DIV    : integer := 40;     -- SCLK = clk / G_SPI_CLK_DIV (2.5 MHz @ 100 MHz; 10 MHz es el limite del LSM9DS1 y no deja margen con cable/Pmod)
        G_POR_WAIT_MS    : integer := 20;     -- espera de arranque del LSM9DS1
        G_POLL_TIMEOUT_MS: integer := 500;    -- umbral "no llega DRDY" -> ERR_OVERRUN (no fatal)
        G_XFER_TIMEOUT_US: integer := 100     -- umbral "spi_engine no responde" -> ERR_SPI_TIMEOUT (fatal)
    );
    port (
        clk    : in std_logic;
        rstn   : in std_logic;

        -- AXI4-Lite esclavo -------------------------------------------------
        s_axi_awaddr  : in  std_logic_vector(7 downto 0);
        s_axi_awvalid : in  std_logic;
        s_axi_awready : out std_logic;
        s_axi_wdata   : in  std_logic_vector(31 downto 0);
        s_axi_wstrb   : in  std_logic_vector(3 downto 0);
        s_axi_wvalid  : in  std_logic;
        s_axi_wready  : out std_logic;
        s_axi_bresp   : out std_logic_vector(1 downto 0);
        s_axi_bvalid  : out std_logic;
        s_axi_bready  : in  std_logic;
        s_axi_araddr  : in  std_logic_vector(7 downto 0);
        s_axi_arvalid : in  std_logic;
        s_axi_arready : out std_logic;
        s_axi_rdata   : out std_logic_vector(31 downto 0);
        s_axi_rresp   : out std_logic_vector(1 downto 0);
        s_axi_rvalid  : out std_logic;
        s_axi_rready  : in  std_logic;

        -- AXI4-Stream maestro (datos) ---------------------------------------
        m_axis_tdata  : out std_logic_vector(31 downto 0);
        m_axis_tvalid : out std_logic;
        m_axis_tlast  : out std_logic;
        m_axis_tready : in  std_logic;

        -- Interrupcion --------------------------------------------------------
        irq : out std_logic;

        -- Bus fisico SPI compartido (hacia el bridge del Pmod) ---------------
        spi_sclk : out std_logic;
        spi_mosi : out std_logic;
        spi_miso : in  std_logic;
        spi_cs_n : out std_logic_vector(1 downto 0)
    );
end entity nav_spi_ctrl;

architecture rtl of nav_spi_ctrl is

    ---------------------------------------------------------------------
    -- Registros AXI-Lite
    ---------------------------------------------------------------------
    signal reg_enable    : std_logic := '0';
    signal reg_irq_en    : std_logic := '0';
    signal pulse_softrst : std_logic := '0';
    signal pulse_clr_err : std_logic := '0';
    signal pulse_apply   : std_logic := '0';

    signal odr_g   : std_logic_vector(2 downto 0) := "011"; -- 119 Hz por defecto
    signal odr_xl  : std_logic_vector(2 downto 0) := "011";
    signal odr_m   : std_logic_vector(2 downto 0) := "101"; -- 20 Hz por defecto
    signal fs_g    : std_logic_vector(1 downto 0) := "00";
    signal fs_xl   : std_logic_vector(1 downto 0) := "00";

    signal st_busy          : std_logic := '0';
    signal st_config_done   : std_logic := '0';
    signal st_ag_detected   : std_logic := '0';
    signal st_mag_detected  : std_logic := '0';
    signal st_err_ag_who    : std_logic := '0';
    signal st_err_mag_who   : std_logic := '0';
    signal st_err_timeout   : std_logic := '0';
    signal st_err_overrun   : std_logic := '0';
    signal sample_count     : unsigned(31 downto 0) := (others => '0');
    signal dbg_who_ag       : std_logic_vector(7 downto 0) := (others => '0');
    signal dbg_who_m        : std_logic_vector(7 downto 0) := (others => '0');

    signal st_error : std_logic;

    ---------------------------------------------------------------------
    -- AXI-Lite: logica de escritura/lectura (patron simplificado)
    ---------------------------------------------------------------------
    signal axi_awready_r, axi_wready_r, axi_bvalid_r : std_logic := '0';
    signal axi_arready_r, axi_rvalid_r                : std_logic := '0';
    signal axi_rdata_r : std_logic_vector(31 downto 0) := (others => '0');
    signal waddr_latched : std_logic_vector(7 downto 0) := (others => '0');

    ---------------------------------------------------------------------
    -- Motor SPI
    ---------------------------------------------------------------------
    signal spi_start     : std_logic := '0';
    signal spi_cs_sel    : integer range 0 to 1 := 0;
    signal spi_addr_byte : std_logic_vector(7 downto 0) := (others => '0');
    signal spi_n_data    : integer range 0 to 6 := 0;
    signal spi_tx_data   : std_logic_vector(7 downto 0) := (others => '0');
    signal spi_busy      : std_logic;
    signal spi_done      : std_logic;
    signal spi_rx0, spi_rx1, spi_rx2, spi_rx3, spi_rx4, spi_rx5 : std_logic_vector(7 downto 0);

    ---------------------------------------------------------------------
    -- Secuenciador principal
    ---------------------------------------------------------------------
    type state_t is (
        S_POR,
        S_WHOAMI_AG_GO, S_WHOAMI_AG_WAIT,
        S_WHOAMI_MAG_GO, S_WHOAMI_MAG_WAIT,
        S_CFG_GO, S_CFG_WAIT,
        S_POLL_AG_GO, S_POLL_AG_WAIT,
        S_BURST_G_GO, S_BURST_G_WAIT,
        S_BURST_XL_GO, S_BURST_XL_WAIT,
        S_POLL_MAG_GO, S_POLL_MAG_WAIT,
        S_BURST_M_GO, S_BURST_M_WAIT,
        S_STREAM,
        S_ERROR
    );
    signal state : state_t := S_POR;

    signal por_cnt      : unsigned(31 downto 0) := (others => '0');
    constant POR_CYCLES : integer := (G_CLK_FREQ_HZ / 1000) * G_POR_WAIT_MS;

    signal xfer_to_cnt      : unsigned(31 downto 0) := (others => '0');
    constant XFER_TO_CYCLES : integer := (G_CLK_FREQ_HZ / 1_000_000) * G_XFER_TIMEOUT_US;

    signal poll_to_cnt      : unsigned(31 downto 0) := (others => '0');
    constant POLL_TO_CYCLES : integer := (G_CLK_FREQ_HZ / 1000) * G_POLL_TIMEOUT_MS;

    signal cfg_idx : integer range 0 to CFG_PROGRAM'length := 0;

    type raw_array_t is array (0 to 8) of std_logic_vector(15 downto 0);
    signal raw_sample : raw_array_t := (others => (others => '0'));
    signal stream_idx : integer range 0 to 8 := 0;

    -- Offsets de calibracion (escribibles por AXI-Lite en caliente):
    --   0x20 gx  0x24 gy  0x28 gz   (sesgo del giroscopio, cuentas crudas)
    --   0x2C mx  0x30 my  0x34 mz   (hard-iron del magnetometro, cuentas crudas)
    -- Se RESTAN de la muestra justo antes del stream; los registros debug
    -- 0x50..0x70 siguen mostrando la muestra sin compensar (para calibrar).
    type off_array_t is array (0 to 5) of signed(15 downto 0);
    signal cal_off : off_array_t := (others => (others => '0'));
    type out_array_t is array (0 to 8) of std_logic_vector(31 downto 0);
    signal out_sample : out_array_t := (others => (others => '0'));

    -- '1' cuando ya se ha leido al menos una muestra del magnetometro
    -- (evita enviar mag = 0 -> normalizacion 1/0 en Madgwick).
    signal mag_valid : std_logic := '0';

    -- AXIS_CFG (0x38): orientacion de ejes aplicada tras restar los offsets.
    --   [2:0]  signos del magnetometro (x,y,z), 1 = negar   (se aplican tras la permutacion)
    --   [5:3]  signos del giroscopio  (x,y,z), 1 = negar
    --   [8:6]  permutacion del magnetometro: out(i) = raw(perm(i))
    --          0:(x,y,z) 1:(x,z,y) 2:(y,x,z) 3:(y,z,x) 4:(z,x,y) 5:(z,y,x)
    --   [11:9] signos del acelerometro (x,y,z)
    -- Por defecto: gyro x,y negados (el giroscopio del LSM9DS1 va al reves que el
    -- acelerometro en X e Y), resto sin cambios.
    signal axis_cfg : std_logic_vector(11 downto 0) := "000" & "000" & "011" & "000";

begin

    ---------------------------------------------------------------------
    -- Muestra de salida: (cruda - offset) con permutacion / signos (AXIS_CFG)
    -- Los offsets estan en el dominio CRUDO del sensor (ejes fisicos).
    ---------------------------------------------------------------------
    process (raw_sample, cal_off, axis_cfg)
        variable v    : signed(31 downto 0);
        variable perm : integer range 0 to 7;
        type perm_t is array (0 to 2) of integer range 0 to 2;
        variable p    : perm_t;
    begin
        -- giroscopio
        for i in 0 to 2 loop
            v := resize(signed(raw_sample(i)), 32) - resize(cal_off(i), 32);
            if axis_cfg(3 + i) = '1' then v := -v; end if;
            out_sample(i) <= std_logic_vector(v);
        end loop;
        -- acelerometro
        for i in 0 to 2 loop
            v := resize(signed(raw_sample(3 + i)), 32);
            if axis_cfg(9 + i) = '1' then v := -v; end if;
            out_sample(3 + i) <= std_logic_vector(v);
        end loop;
        -- magnetometro
        perm := to_integer(unsigned(axis_cfg(8 downto 6)));
        case perm is
            when 1      => p := (0, 2, 1);
            when 2      => p := (1, 0, 2);
            when 3      => p := (1, 2, 0);
            when 4      => p := (2, 0, 1);
            when 5      => p := (2, 1, 0);
            when others => p := (0, 1, 2);
        end case;
        for i in 0 to 2 loop
            v := resize(signed(raw_sample(6 + p(i))), 32) - resize(cal_off(3 + p(i)), 32);
            if axis_cfg(i) = '1' then v := -v; end if;
            out_sample(6 + i) <= std_logic_vector(v);
        end loop;
    end process;

    ---------------------------------------------------------------------
    -- AXI4-Lite esclavo (simplificado: aw/w capturados de forma
    -- independiente, un registro de retorno de lectura combinacional)
    ---------------------------------------------------------------------
    s_axi_awready <= axi_awready_r;
    s_axi_wready  <= axi_wready_r;
    s_axi_bvalid  <= axi_bvalid_r;
    s_axi_bresp   <= "00";
    s_axi_arready <= axi_arready_r;
    s_axi_rvalid  <= axi_rvalid_r;
    s_axi_rresp   <= "00";
    s_axi_rdata   <= axi_rdata_r;

    process (clk, rstn)
        variable wr_word : integer;
    begin
        if rstn = '0' then
            axi_awready_r <= '0';
            axi_wready_r  <= '0';
            axi_bvalid_r  <= '0';
            axi_arready_r <= '0';
            axi_rvalid_r  <= '0';
            reg_enable    <= '0';
            reg_irq_en    <= '0';
            odr_g  <= "011"; odr_xl <= "011"; odr_m <= "101";
            fs_g   <= "00";  fs_xl  <= "00";
            cal_off <= (others => (others => '0'));
            axis_cfg <= "000" & "000" & "011" & "000";
            pulse_softrst <= '0';
            pulse_clr_err <= '0';
            pulse_apply   <= '0';

        elsif rising_edge(clk) then
            pulse_softrst <= '0';
            pulse_clr_err <= '0';
            pulse_apply   <= '0';

            -- Escritura ------------------------------------------------
            if axi_awready_r = '0' and s_axi_awvalid = '1' and s_axi_wvalid = '1' then
                axi_awready_r <= '1';
                axi_wready_r  <= '1';
                waddr_latched <= s_axi_awaddr;
            else
                axi_awready_r <= '0';
                axi_wready_r  <= '0';
            end if;

            if axi_awready_r = '1' and axi_wready_r = '1' then
                wr_word := to_integer(unsigned(waddr_latched));
                case wr_word is
                    when REGOFF_CTRL =>
                        reg_enable    <= s_axi_wdata(0);
                        pulse_softrst <= s_axi_wdata(1);
                        reg_irq_en    <= s_axi_wdata(2);
                        pulse_clr_err <= s_axi_wdata(3);
                    when REGOFF_ODR_CFG =>
                        odr_g  <= s_axi_wdata(2 downto 0);
                        odr_xl <= s_axi_wdata(5 downto 3);
                        odr_m  <= s_axi_wdata(8 downto 6);
                    when REGOFF_RANGE_CFG =>
                        fs_g  <= s_axi_wdata(1 downto 0);
                        fs_xl <= s_axi_wdata(3 downto 2);
                    when REGOFF_APPLY_CFG =>
                        pulse_apply <= s_axi_wdata(0);
                    when 16#20# | 16#24# | 16#28# | 16#2C# | 16#30# | 16#34# =>
                        cal_off((wr_word - 16#20#) / 4) <= signed(s_axi_wdata(15 downto 0));
                    when 16#38# =>
                        axis_cfg <= s_axi_wdata(11 downto 0);
                    when others =>
                        null;
                end case;
                axi_bvalid_r <= '1';
            elsif s_axi_bready = '1' then
                axi_bvalid_r <= '0';
            end if;

            -- Lectura ----------------------------------------------------
            if axi_arready_r = '0' and s_axi_arvalid = '1' then
                axi_arready_r <= '1';
                case to_integer(unsigned(s_axi_araddr)) is
                    when REGOFF_CTRL =>
                        axi_rdata_r <= (31 downto 4 => '0') & '0' & reg_irq_en & '0' & reg_enable;
                    when REGOFF_ODR_CFG =>
                        axi_rdata_r <= (31 downto 9 => '0') & odr_m & odr_xl & odr_g;
                    when REGOFF_RANGE_CFG =>
                        axi_rdata_r <= (31 downto 4 => '0') & fs_xl & fs_g;
                    when REGOFF_STATUS =>
                        axi_rdata_r <= (others => '0');
                        axi_rdata_r(STAT_BIT_BUSY)          <= st_busy;
                        axi_rdata_r(STAT_BIT_CONFIG_DONE)   <= st_config_done;
                        axi_rdata_r(STAT_BIT_AG_DETECTED)   <= st_ag_detected;
                        axi_rdata_r(STAT_BIT_MAG_DETECTED)  <= st_mag_detected;
                        axi_rdata_r(STAT_BIT_ERROR)         <= st_error;
                        axi_rdata_r(STAT_BIT_ERR_AG_WHOAMI) <= st_err_ag_who;
                        axi_rdata_r(STAT_BIT_ERR_MAG_WHOAMI)<= st_err_mag_who;
                        axi_rdata_r(STAT_BIT_ERR_TIMEOUT)   <= st_err_timeout;
                        axi_rdata_r(STAT_BIT_ERR_OVERRUN)   <= st_err_overrun;
                    when REGOFF_SAMPLE_COUNT =>
                        axi_rdata_r <= std_logic_vector(sample_count);
                    when 16#20# | 16#24# | 16#28# | 16#2C# | 16#30# | 16#34# =>
                        axi_rdata_r <= std_logic_vector(resize(
                            cal_off((to_integer(unsigned(s_axi_araddr)) - 16#20#) / 4), 32));
                    when 16#38# =>
                        axi_rdata_r <= (31 downto 12 => '0') & axis_cfg;
                    when 16#48# =>   -- DEBUG: WHO_AM_I leido [7:0]=A/G [15:8]=Mag
                        axi_rdata_r <= (31 downto 16 => '0') & dbg_who_m & dbg_who_ag;
                    when 16#50# | 16#54# | 16#58# | 16#5C# | 16#60# | 16#64# | 16#68# | 16#6C# | 16#70# =>
                        -- DEBUG: ultima muestra cruda (gx,gy,gz,ax,ay,az,mx,my,mz), signo extendido
                        axi_rdata_r <= std_logic_vector(resize(signed(
                            raw_sample((to_integer(unsigned(s_axi_araddr)) - 16#50#) / 4)), 32));
                    when others =>
                        axi_rdata_r <= (others => '0');
                end case;
            elsif s_axi_rready = '1' then
                axi_arready_r <= '0';
            end if;

            if axi_arready_r = '1' then
                axi_rvalid_r <= '1';
            elsif s_axi_rready = '1' then
                axi_rvalid_r <= '0';
            end if;
        end if;
    end process;

    st_error <= st_err_ag_who or st_err_mag_who or st_err_timeout or st_err_overrun;
    irq <= st_error and reg_irq_en;

    ---------------------------------------------------------------------
    -- Motor SPI compartido
    ---------------------------------------------------------------------
    u_spi : entity work.spi_engine
        generic map (
            G_CLK_DIV   => G_SPI_CLK_DIV,
            G_MAX_BYTES => 7
        )
        port map (
            clk       => clk,
            rstn      => rstn,
            start     => spi_start,
            cs_sel    => spi_cs_sel,
            addr_byte => spi_addr_byte,
            n_data    => spi_n_data,
            tx_data   => spi_tx_data,
            busy      => spi_busy,
            done      => spi_done,
            rx_data0  => spi_rx0,
            rx_data1  => spi_rx1,
            rx_data2  => spi_rx2,
            rx_data3  => spi_rx3,
            rx_data4  => spi_rx4,
            rx_data5  => spi_rx5,
            spi_sclk  => spi_sclk,
            spi_mosi  => spi_mosi,
            spi_miso  => spi_miso,
            spi_cs_n  => spi_cs_n
        );

    ---------------------------------------------------------------------
    -- Secuenciador principal
    ---------------------------------------------------------------------
    process (clk, rstn)
        variable cfg_byte : std_logic_vector(7 downto 0);
    begin
        if rstn = '0' then
            state <= S_POR;
            por_cnt <= (others => '0');
            spi_start <= '0';
            st_busy <= '0';
            st_config_done  <= '0';
            st_ag_detected  <= '0';
            st_mag_detected <= '0';
            st_err_ag_who   <= '0';
            st_err_mag_who  <= '0';
            st_err_timeout  <= '0';
            st_err_overrun  <= '0';
            sample_count <= (others => '0');
            m_axis_tvalid <= '0';
            m_axis_tlast  <= '0';
            mag_valid     <= '0';

        elsif rising_edge(clk) then
            spi_start <= '0';

            -- Limpieza de errores por software ---------------------------------
            if pulse_clr_err = '1' then
                st_err_ag_who  <= '0';
                st_err_mag_who <= '0';
                st_err_timeout <= '0';
                st_err_overrun <= '0';
            end if;

            case state is

                ---------------------------------------------------------
                when S_POR =>
                    if reg_enable = '0' then
                        -- Espera a que el PS escriba ENABLE=1 antes de arrancar
                        -- (desacopla el arranque del reset de la PL).
                        por_cnt <= (others => '0');
                        st_busy <= '0';
                    else
                        st_busy <= '1';
                        -- Guarda con POR_CYCLES <= 1 primero: evita comparar
                        -- por_cnt (unsigned) contra "POR_CYCLES - 1" cuando esa
                        -- resta daria -1 (p.ej. G_POR_WAIT_MS=0 en simulacion).
                        if POR_CYCLES <= 1 then
                            por_cnt <= (others => '0');
                            state   <= S_WHOAMI_AG_GO;
                        elsif por_cnt = POR_CYCLES - 1 then
                            por_cnt <= (others => '0');
                            state   <= S_WHOAMI_AG_GO;
                        else
                            por_cnt <= por_cnt + 1;
                        end if;
                    end if;

                ---------------------------------------------------------
                when S_WHOAMI_AG_GO =>
                    spi_cs_sel    <= CS_AG;
                    spi_addr_byte <= SPI_RW_READ & SPI_MS_SINGLE & REG_AG_WHO_AM_I;
                    spi_n_data    <= 1;
                    spi_start     <= '1';
                    xfer_to_cnt   <= (others => '0');
                    state         <= S_WHOAMI_AG_WAIT;

                when S_WHOAMI_AG_WAIT =>
                    if spi_done = '1' then
                        dbg_who_ag <= spi_rx0;
                        if spi_rx0 = WHOAMI_AG_EXPECTED then
                            st_ag_detected <= '1';
                        else
                            st_err_ag_who <= '1';
                        end if;
                        state <= S_WHOAMI_MAG_GO;
                    elsif xfer_to_cnt = XFER_TO_CYCLES - 1 then
                        st_err_timeout <= '1';
                        state <= S_ERROR;
                    else
                        xfer_to_cnt <= xfer_to_cnt + 1;
                    end if;

                ---------------------------------------------------------
                when S_WHOAMI_MAG_GO =>
                    spi_cs_sel    <= CS_MAG;
                    spi_addr_byte <= SPI_RW_READ & SPI_MS_SINGLE & REG_M_WHO_AM_I;
                    spi_n_data    <= 1;
                    spi_start     <= '1';
                    xfer_to_cnt   <= (others => '0');
                    state         <= S_WHOAMI_MAG_WAIT;

                when S_WHOAMI_MAG_WAIT =>
                    if spi_done = '1' then
                        dbg_who_m <= spi_rx0;
                        if spi_rx0 = WHOAMI_M_EXPECTED then
                            st_mag_detected <= '1';
                        else
                            st_err_mag_who <= '1';
                        end if;
                        cfg_idx <= 0;
                        state   <= S_CFG_GO;
                    elsif xfer_to_cnt = XFER_TO_CYCLES - 1 then
                        st_err_timeout <= '1';
                        state <= S_ERROR;
                    else
                        xfer_to_cnt <= xfer_to_cnt + 1;
                    end if;

                ---------------------------------------------------------
                -- Micro-secuenciador de configuracion (tabla CFG_PROGRAM)
                ---------------------------------------------------------
                when S_CFG_GO =>
                    if cfg_idx = CFG_PROGRAM'length then
                        st_config_done <= '1';
                        st_busy        <= '0';
                        state          <= S_POLL_AG_GO;
                    else
                        spi_cs_sel    <= CFG_PROGRAM(cfg_idx).cs;
                        spi_addr_byte <= SPI_RW_WRITE & SPI_MS_SINGLE & CFG_PROGRAM(cfg_idx).addr;
                        spi_n_data    <= 1;

                        -- CTRL_REG1_G / CTRL_REG6_XL: [7:5]=ODR [4:3]=FS [2:0]=000 (reservado+BW)
                        -- CTRL_REG1_M: [7]=0 [6:5]=OM=00 [4:2]=ODR_M [1:0]=00
                        case cfg_idx is
                            when 0 => cfg_byte := odr_g  & fs_g  & "000";
                            when 1 => cfg_byte := odr_xl & fs_xl & "000";
                            when 3 => cfg_byte := '0' & "00" & odr_m & "00";
                            when others => cfg_byte := CFG_PROGRAM(cfg_idx).data;
                        end case;
                        spi_tx_data <= cfg_byte;

                        spi_start   <= '1';
                        xfer_to_cnt <= (others => '0');
                        state       <= S_CFG_WAIT;
                    end if;

                when S_CFG_WAIT =>
                    if spi_done = '1' then
                        cfg_idx <= cfg_idx + 1;
                        state   <= S_CFG_GO;
                    elsif xfer_to_cnt = XFER_TO_CYCLES - 1 then
                        st_err_timeout <= '1';
                        state <= S_ERROR;
                    else
                        xfer_to_cnt <= xfer_to_cnt + 1;
                    end if;

                ---------------------------------------------------------
                -- Bucle de adquisicion: sondeo DRDY + rafagas + stream
                ---------------------------------------------------------
                when S_POLL_AG_GO =>
                    if reg_enable = '0' then
                        state <= S_POLL_AG_GO;  -- en pausa si ENABLE=0
                    elsif pulse_apply = '1' then
                        cfg_idx <= 0;
                        state   <= S_CFG_GO;    -- reconfigurar ODR/rango en caliente
                    else
                        spi_cs_sel    <= CS_AG;
                        spi_addr_byte <= SPI_RW_READ & SPI_MS_SINGLE & REG_AG_STATUS_REG;
                        spi_n_data    <= 1;
                        spi_start     <= '1';
                        xfer_to_cnt   <= (others => '0');
                        state         <= S_POLL_AG_WAIT;
                    end if;

                when S_POLL_AG_WAIT =>
                    if spi_done = '1' then
                        if spi_rx0(1) = '1' then -- GDA: dato de giro (y accel, misma ODR) listo
                            poll_to_cnt <= (others => '0');
                            state <= S_BURST_G_GO;
                        else
                            if poll_to_cnt = POLL_TO_CYCLES - 1 then
                                st_err_overrun <= '1'; -- no fatal: seguimos sondeando
                                poll_to_cnt <= (others => '0');
                            else
                                poll_to_cnt <= poll_to_cnt + 1;
                            end if;
                            state <= S_POLL_AG_GO;
                        end if;
                    elsif xfer_to_cnt = XFER_TO_CYCLES - 1 then
                        st_err_timeout <= '1';
                        state <= S_ERROR;
                    else
                        xfer_to_cnt <= xfer_to_cnt + 1;
                    end if;

                when S_BURST_G_GO =>
                    spi_cs_sel    <= CS_AG;
                    -- A/G: la direccion es de 7 bits (bit 6 = A6), NO hay bit MS; el
                    -- autoincremento lo da IF_ADD_INC de CTRL_REG8 (activo por defecto).
                    spi_addr_byte <= SPI_RW_READ & SPI_MS_SINGLE & REG_AG_OUT_X_L_G;
                    spi_n_data    <= 6;
                    spi_start     <= '1';
                    xfer_to_cnt   <= (others => '0');
                    state         <= S_BURST_G_WAIT;

                when S_BURST_G_WAIT =>
                    if spi_done = '1' then
                        -- Muestra CRUDA del sensor. Los signos / permutacion de ejes se
                        -- aplican despues (out_sample) segun el registro AXIS_CFG.
                        raw_sample(0) <= spi_rx1 & spi_rx0; -- gx
                        raw_sample(1) <= spi_rx3 & spi_rx2; -- gy
                        raw_sample(2) <= spi_rx5 & spi_rx4; -- gz
                        state <= S_BURST_XL_GO;
                    elsif xfer_to_cnt = XFER_TO_CYCLES - 1 then
                        st_err_timeout <= '1';
                        state <= S_ERROR;
                    else
                        xfer_to_cnt <= xfer_to_cnt + 1;
                    end if;

                when S_BURST_XL_GO =>
                    spi_cs_sel    <= CS_AG;
                    -- A/G: la direccion es de 7 bits (bit 6 = A6), NO hay bit MS; el
                    -- autoincremento lo da IF_ADD_INC de CTRL_REG8 (activo por defecto).
                    spi_addr_byte <= SPI_RW_READ & SPI_MS_SINGLE & REG_AG_OUT_X_L_XL;
                    spi_n_data    <= 6;
                    spi_start     <= '1';
                    xfer_to_cnt   <= (others => '0');
                    state         <= S_BURST_XL_WAIT;

                when S_BURST_XL_WAIT =>
                    if spi_done = '1' then
                        raw_sample(3) <= spi_rx1 & spi_rx0; -- ax
                        raw_sample(4) <= spi_rx3 & spi_rx2; -- ay
                        raw_sample(5) <= spi_rx5 & spi_rx4; -- az
                        state <= S_POLL_MAG_GO;
                    elsif xfer_to_cnt = XFER_TO_CYCLES - 1 then
                        st_err_timeout <= '1';
                        state <= S_ERROR;
                    else
                        xfer_to_cnt <= xfer_to_cnt + 1;
                    end if;

                when S_POLL_MAG_GO =>
                    spi_cs_sel    <= CS_MAG;
                    spi_addr_byte <= SPI_RW_READ & SPI_MS_SINGLE & REG_M_STATUS_REG_M;
                    spi_n_data    <= 1;
                    spi_start     <= '1';
                    xfer_to_cnt   <= (others => '0');
                    state         <= S_POLL_MAG_WAIT;

                when S_POLL_MAG_WAIT =>
                    if spi_done = '1' then
                        if spi_rx0(3) = '1' then -- ZYXDA
                            poll_to_cnt <= (others => '0');
                            state <= S_BURST_M_GO;
                        elsif mag_valid = '1' then
                            -- Sin dato nuevo de mag: se reutiliza la ultima muestra
                            -- y la trama sale a la cadencia del A/G (119 Hz).
                            poll_to_cnt <= (others => '0');
                            stream_idx  <= 0;
                            state       <= S_STREAM;
                        else
                            if poll_to_cnt = POLL_TO_CYCLES - 1 then
                                st_err_overrun <= '1';
                                poll_to_cnt <= (others => '0');
                            else
                                poll_to_cnt <= poll_to_cnt + 1;
                            end if;
                            state <= S_POLL_MAG_GO;
                        end if;
                    elsif xfer_to_cnt = XFER_TO_CYCLES - 1 then
                        st_err_timeout <= '1';
                        state <= S_ERROR;
                    else
                        xfer_to_cnt <= xfer_to_cnt + 1;
                    end if;

                when S_BURST_M_GO =>
                    spi_cs_sel    <= CS_MAG;
                    spi_addr_byte <= SPI_RW_READ & SPI_MS_INC & REG_M_OUT_X_L_M;
                    spi_n_data    <= 6;
                    spi_start     <= '1';
                    xfer_to_cnt   <= (others => '0');
                    state         <= S_BURST_M_WAIT;

                when S_BURST_M_WAIT =>
                    if spi_done = '1' then
                        raw_sample(6) <= spi_rx1 & spi_rx0; -- mx
                        raw_sample(7) <= spi_rx3 & spi_rx2; -- my
                        raw_sample(8) <= spi_rx5 & spi_rx4; -- mz
                        mag_valid  <= '1';
                        stream_idx <= 0;
                        state <= S_STREAM;
                    elsif xfer_to_cnt = XFER_TO_CYCLES - 1 then
                        st_err_timeout <= '1';
                        state <= S_ERROR;
                    else
                        xfer_to_cnt <= xfer_to_cnt + 1;
                    end if;

                ---------------------------------------------------------
                when S_STREAM =>
                    -- Handshake AXI4-Stream correcto: tdata/tlast se actualizan
                    -- JUNTO con stream_idx y SOLO cuando se completa una
                    -- transferencia (tvalid='1' y tready='1'). Con tready fijo
                    -- a '1' (raw2float a II=1) la version anterior reenviaba
                    -- la palabra 0 dos veces, perdia la palabra 8 (mz) y nunca
                    -- mostraba TLAST.
                    if m_axis_tvalid = '0' then
                        m_axis_tdata  <= out_sample(stream_idx);
                        m_axis_tvalid <= '1';
                        if stream_idx = 8 then
                            m_axis_tlast <= '1';
                        else
                            m_axis_tlast <= '0';
                        end if;
                    elsif m_axis_tready = '1' then
                        if stream_idx = 8 then
                            m_axis_tvalid <= '0';
                            m_axis_tlast  <= '0';
                            sample_count  <= sample_count + 1;
                            state         <= S_POLL_AG_GO;
                        else
                            stream_idx   <= stream_idx + 1;
                            m_axis_tdata <= out_sample(stream_idx + 1);
                            if stream_idx + 1 = 8 then
                                m_axis_tlast <= '1';
                            else
                                m_axis_tlast <= '0';
                            end if;
                        end if;
                    end if;

                ---------------------------------------------------------
                when S_ERROR =>
                    st_busy       <= '0';
                    m_axis_tvalid <= '0';
                    -- Se sale de aqui solo con SOFT_RESET (pulse_softrst),
                    -- ya gestionado de forma global mas arriba.
                    state <= S_ERROR;

                -- S_RESET_REGS esta declarado en el tipo pero no se usa
                -- (hueco reservado); este "when others" solo cubre eso y
                -- sirve de red de seguridad si se anade algun estado nuevo
                -- sin darle rama propia.
                when others =>
                    null;

            end case;

            -- Soft reset: va DESPUES del case para que gane a las asignaciones
            -- de 'state' de arriba (si no, el case lo sobrescribe en el mismo ciclo).
            if pulse_softrst = '1' then
                state <= S_POR;
                por_cnt <= (others => '0');
                st_config_done <= '0';
                st_ag_detected <= '0';
                st_mag_detected <= '0';
                m_axis_tvalid  <= '0';
                mag_valid      <= '0';
            end if;
        end if;
    end process;

end architecture rtl;