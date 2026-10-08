-------------------------------------------------------------------------------
-- tb_nav_spi_ctrl.vhd
-- Testbench de la FSM completa (nav_spi_ctrl) con un modelo de esclavo SPI
-- que se comporta como un LSM9DS1: responde al WHO_AM_I de cada subdispositivo,
-- marca DRDY siempre listo (para no alargar la simulacion esperando ODR real)
-- y devuelve patrones de prueba fijos y reconocibles en las rafagas de
-- OUT_X_L_G / OUT_X_L_XL / OUT_X_L_M.
--
-- Comprueba:
--   1) Que tras habilitar (ENABLE=1) el estado pasa a CONFIG_DONE=1,
--      AG_DETECTED=1, MAG_DETECTED=1, sin bits de error.
--   2) Que las 9 palabras que salen por M_AXIS en la primera muestra
--      coinciden con el patron de prueba inyectado por el esclavo, en el
--      orden esperado (gx,gy,gz,ax,ay,az,mx,my,mz), con TLAST en la novena.
--   3) Que SAMPLE_COUNT se incrementa.
--
-- NO cubre (dejado para una siguiente iteracion si se quiere mas cobertura):
--   - inyeccion de WHO_AM_I erroneo / timeout SPI / recuperacion con SOFT_RESET
--   - verificacion byte a byte de los registros de configuracion escritos
--
-- AVISO: no se ha podido compilar en este entorno. Revisar con el simulador
-- (Vivado xsim) antes de confiar en el resultado.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.nav_pkg.all;

entity tb_nav_spi_ctrl is
end entity tb_nav_spi_ctrl;

architecture sim of tb_nav_spi_ctrl is

    constant CLK_PERIOD : time := 10 ns; -- 100 MHz
    signal clk      : std_logic := '0';
    signal rstn     : std_logic := '0';
    signal sim_done : boolean   := false;

    -- AXI4-Lite
    signal s_axi_awaddr  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_axi_awvalid : std_logic := '0';
    signal s_axi_awready : std_logic;
    signal s_axi_wdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal s_axi_wstrb   : std_logic_vector(3 downto 0) := "1111";
    signal s_axi_wvalid  : std_logic := '0';
    signal s_axi_wready  : std_logic;
    signal s_axi_bresp   : std_logic_vector(1 downto 0);
    signal s_axi_bvalid  : std_logic;
    signal s_axi_bready  : std_logic := '0';
    signal s_axi_araddr  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_axi_arvalid : std_logic := '0';
    signal s_axi_arready : std_logic;
    signal s_axi_rdata   : std_logic_vector(31 downto 0);
    signal s_axi_rresp   : std_logic_vector(1 downto 0);
    signal s_axi_rvalid  : std_logic;
    signal s_axi_rready  : std_logic := '0';

    -- AXI4-Stream
    signal m_axis_tdata  : std_logic_vector(31 downto 0);
    signal m_axis_tvalid : std_logic;
    signal m_axis_tlast  : std_logic;
    signal m_axis_tready : std_logic := '1'; -- siempre listo para simplificar

    signal irq : std_logic;

    -- Bus SPI fisico
    signal spi_sclk : std_logic;
    signal spi_mosi : std_logic;
    signal spi_miso : std_logic := '0';
    signal spi_cs_n : std_logic_vector(1 downto 0);

    -- Ultimo registro escrito visto por el modelo de esclavo (para inspeccion
    -- manual en las ondas; no se comprueba por assert en este primer borrador)
    signal mon_last_wr_dev  : integer := -1;
    signal mon_last_wr_addr : std_logic_vector(5 downto 0) := (others => '0');
    signal mon_last_wr_data : std_logic_vector(7 downto 0) := (others => '0');

    -- Contador de muestras completas vistas en M_AXIS (para comparar con
    -- SAMPLE_COUNT leido por AXI-Lite)
    signal axis_words_seen : integer := 0;

    type word_array_t is array (0 to 8) of std_logic_vector(31 downto 0);
    constant EXPECTED_WORDS : word_array_t := (
        0 => x"00001111",  -- gx
        1 => x"00001222",  -- gy
        2 => x"00001333",  -- gz
        3 => x"00001444",  -- ax
        4 => x"00001555",  -- ay
        5 => x"00001666",  -- az
        6 => x"00001777",  -- mx
        7 => x"00001888",  -- my
        8 => x"00001999"   -- mz
    );

    ---------------------------------------------------------------------
    -- Modelo del contenido de registros del LSM9DS1 (solo lo necesario
    -- para que la FSM progrese y se pueda comprobar el camino de datos)
    ---------------------------------------------------------------------
    function lsm9ds1_reg(dev : integer; addr : std_logic_vector(5 downto 0); byte_n : integer)
        return std_logic_vector is
    begin
        if dev = CS_AG and addr = REG_AG_WHO_AM_I then
            return WHOAMI_AG_EXPECTED;
        elsif dev = CS_MAG and addr = REG_M_WHO_AM_I then
            return WHOAMI_M_EXPECTED;
        elsif dev = CS_AG and addr = REG_AG_STATUS_REG then
            return "00000010"; -- GDA=1
        elsif dev = CS_MAG and addr = REG_M_STATUS_REG_M then
            return "00001000"; -- ZYXDA=1
        elsif dev = CS_AG and addr = REG_AG_OUT_X_L_G then
            case byte_n is
                when 0 => return x"11"; when 1 => return x"11";  -- gx = 0x1111
                when 2 => return x"22"; when 3 => return x"12";  -- gy = 0x1222
                when 4 => return x"33"; when 5 => return x"13";  -- gz = 0x1333
                when others => return x"00";
            end case;
        elsif dev = CS_AG and addr = REG_AG_OUT_X_L_XL then
            case byte_n is
                when 0 => return x"44"; when 1 => return x"14";  -- ax = 0x1444
                when 2 => return x"55"; when 3 => return x"15";  -- ay = 0x1555
                when 4 => return x"66"; when 5 => return x"16";  -- az = 0x1666
                when others => return x"00";
            end case;
        elsif dev = CS_MAG and addr = REG_M_OUT_X_L_M then
            case byte_n is
                when 0 => return x"77"; when 1 => return x"17";  -- mx = 0x1777
                when 2 => return x"88"; when 3 => return x"18";  -- my = 0x1888
                when 4 => return x"99"; when 5 => return x"19";  -- mz = 0x1999
                when others => return x"00";
            end case;
        else
            return x"00";  -- escrituras de configuracion: valor de retorno irrelevante
        end if;
    end function;

begin

    clk <= not clk after CLK_PERIOD / 2 when not sim_done else clk;

    dut : entity work.nav_spi_ctrl
        generic map (
            G_CLK_FREQ_HZ     => 100_000_000,
            G_SPI_CLK_DIV     => 4,     -- rapido, solo para que la sim no tarde
            G_POR_WAIT_MS     => 0,     -- sin espera de arranque en sim (nav_spi_ctrl.vhd
                                         -- ya lo trata como caso especial y la salta)
            G_POLL_TIMEOUT_MS => 1,
            G_XFER_TIMEOUT_US => 50
        )
        port map (
            clk           => clk,
            rstn          => rstn,
            s_axi_awaddr  => s_axi_awaddr,
            s_axi_awvalid => s_axi_awvalid,
            s_axi_awready => s_axi_awready,
            s_axi_wdata   => s_axi_wdata,
            s_axi_wstrb   => s_axi_wstrb,
            s_axi_wvalid  => s_axi_wvalid,
            s_axi_wready  => s_axi_wready,
            s_axi_bresp   => s_axi_bresp,
            s_axi_bvalid  => s_axi_bvalid,
            s_axi_bready  => s_axi_bready,
            s_axi_araddr  => s_axi_araddr,
            s_axi_arvalid => s_axi_arvalid,
            s_axi_arready => s_axi_arready,
            s_axi_rdata   => s_axi_rdata,
            s_axi_rresp   => s_axi_rresp,
            s_axi_rvalid  => s_axi_rvalid,
            s_axi_rready  => s_axi_rready,
            m_axis_tdata  => m_axis_tdata,
            m_axis_tvalid => m_axis_tvalid,
            m_axis_tlast  => m_axis_tlast,
            m_axis_tready => m_axis_tready,
            irq           => irq,
            spi_sclk      => spi_sclk,
            spi_mosi      => spi_mosi,
            spi_miso      => spi_miso,
            spi_cs_n      => spi_cs_n
        );

    ---------------------------------------------------------------------
    -- Modelo de esclavo SPI (LSM9DS1)
    ---------------------------------------------------------------------
    slave_model : process
        variable dev       : integer range 0 to 1;
        variable first_byte: std_logic_vector(7 downto 0);
        variable rw, ms    : std_logic;
        variable addr      : std_logic_vector(5 downto 0);
        variable byte_idx  : integer;
        variable shreg_in  : std_logic_vector(7 downto 0);
        variable resp_byte : std_logic_vector(7 downto 0) := (others => '0');
    begin
        loop
            spi_miso <= '0';
            wait until spi_cs_n(0) = '0' or spi_cs_n(1) = '0';
            dev := 0;
            if spi_cs_n(1) = '0' then dev := 1; end if;
            byte_idx := 0;

            loop
                shreg_in := (others => '0');
                for b in 0 to 7 loop
                    -- Flanco de bajada primero: aqui es donde el esclavo deja
                    -- listo el bit de MISO para que el maestro lo muestree en
                    -- la siguiente subida (igual que en tb_spi_engine.vhd).
                    wait until falling_edge(spi_sclk) or spi_cs_n(dev) = '1';
                    exit when spi_cs_n(dev) = '1';

                    if byte_idx > 0 and rw = '1' then
                        spi_miso <= resp_byte(7 - b);
                    end if;

                    wait until rising_edge(spi_sclk) or spi_cs_n(dev) = '1';
                    exit when spi_cs_n(dev) = '1';
                    shreg_in := shreg_in(6 downto 0) & spi_mosi;
                end loop;
                exit when spi_cs_n(dev) = '1';

                if byte_idx = 0 then
                    first_byte := shreg_in;
                    rw   := first_byte(7);
                    ms   := first_byte(6);
                    addr := first_byte(5 downto 0);
                    resp_byte := lsm9ds1_reg(dev, addr, 0);
                else
                    if rw = '0' then
                        mon_last_wr_dev  <= dev;
                        mon_last_wr_addr <= addr;
                        mon_last_wr_data <= shreg_in;
                    end if;
                    resp_byte := lsm9ds1_reg(dev, addr, byte_idx);
                end if;

                byte_idx := byte_idx + 1;
            end loop;
        end loop;
    end process;

    ---------------------------------------------------------------------
    -- Monitor de AXI4-Stream: comprueba cada palabra contra lo esperado
    -- y cuenta muestras completas (TLAST)
    ---------------------------------------------------------------------
    axis_monitor : process
        variable idx : integer := 0;
    begin
        loop
            wait until rising_edge(clk);
            if m_axis_tvalid = '1' and m_axis_tready = '1' then
                assert m_axis_tdata = EXPECTED_WORDS(idx)
                    report "FALLO AXIS: palabra " & integer'image(idx) &
                           " no coincide con el patron esperado"
                    severity error;

                if idx = 8 then
                    assert m_axis_tlast = '1'
                        report "FALLO AXIS: falta TLAST en la novena palabra"
                        severity error;
                    idx := 0;
                    axis_words_seen <= axis_words_seen + 1;
                else
                    assert m_axis_tlast = '0'
                        report "FALLO AXIS: TLAST inesperado antes de la novena palabra"
                        severity error;
                    idx := idx + 1;
                end if;
            end if;
        end loop;
    end process;

    ---------------------------------------------------------------------
    -- Procedimientos AXI4-Lite (maestro simplificado, un acceso a la vez)
    ---------------------------------------------------------------------
    stim : process

        procedure axi_write(addr : integer; data : std_logic_vector(31 downto 0)) is
        begin
            wait until rising_edge(clk);
            s_axi_awaddr  <= std_logic_vector(to_unsigned(addr, 8));
            s_axi_awvalid <= '1';
            s_axi_wdata   <= data;
            s_axi_wvalid  <= '1';
            wait until rising_edge(clk) and s_axi_awready = '1' and s_axi_wready = '1';
            wait until rising_edge(clk);
            s_axi_awvalid <= '0';
            s_axi_wvalid  <= '0';
            s_axi_bready  <= '1';
            wait until rising_edge(clk) and s_axi_bvalid = '1';
            wait until rising_edge(clk);
            s_axi_bready <= '0';
        end procedure;

        procedure axi_read(addr : integer; result : out std_logic_vector(31 downto 0)) is
        begin
            wait until rising_edge(clk);
            s_axi_araddr  <= std_logic_vector(to_unsigned(addr, 8));
            s_axi_arvalid <= '1';
            wait until rising_edge(clk) and s_axi_arready = '1';
            wait until rising_edge(clk);
            s_axi_arvalid <= '0';
            s_axi_rready  <= '1';
            wait until rising_edge(clk) and s_axi_rvalid = '1';
            result := s_axi_rdata;
            wait until rising_edge(clk);
            s_axi_rready <= '0';
        end procedure;

        variable rd : std_logic_vector(31 downto 0);

    begin
        rstn <= '0';
        wait for 100 ns;
        rstn <= '1';
        wait for 100 ns;

        -- Habilita la FSM (ENABLE=1, IRQ_EN=1)
        axi_write(REGOFF_CTRL, x"00000005");

        -- Espera a la primera muestra completa en AXI-Stream
        wait until axis_words_seen = 1 for 200 us;
        assert axis_words_seen >= 1
            report "FALLO: no se vio ninguna muestra completa por M_AXIS en 200 us"
            severity error;

        -- Comprueba el registro de estado
        axi_read(REGOFF_STATUS, rd);
        assert rd(STAT_BIT_CONFIG_DONE)  = '1' report "FALLO: CONFIG_DONE no se activo" severity error;
        assert rd(STAT_BIT_AG_DETECTED)  = '1' report "FALLO: AG_DETECTED no se activo" severity error;
        assert rd(STAT_BIT_MAG_DETECTED) = '1' report "FALLO: MAG_DETECTED no se activo" severity error;
        assert rd(STAT_BIT_ERROR)        = '0' report "FALLO: hay un bit de error activo sin motivo" severity error;
        report "Lectura de STATUS: " & to_hstring(rd);

        -- Deja correr un poco mas para ver varias muestras y comprobar el contador
        wait until axis_words_seen = 3 for 200 us;
        axi_read(REGOFF_SAMPLE_COUNT, rd);
        report "SAMPLE_COUNT leido = " & integer'image(to_integer(unsigned(rd)));
        assert to_integer(unsigned(rd)) >= 3
            report "FALLO: SAMPLE_COUNT no avanza al ritmo esperado"
            severity error;

        report "Si no ha aparecido ningun ERROR arriba, la FSM pasa esta prueba basica";
        wait for 1 us;
        sim_done <= true;
        wait;
    end process;

end architecture sim;