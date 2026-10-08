-------------------------------------------------------------------------------
-- tb_spi_engine.vhd
-- Testbench minimo para spi_engine: comprueba una escritura de 1 byte y una
-- lectura en rafaga de 6 bytes contra un esclavo SPI de prueba que siempre
-- devuelve el mismo patron (0xA5). Sirve para validar el timing del motor
-- (modo SPI 3) de forma aislada, antes de probar la FSM completa.
--
-- AVISO: no se ha podido compilar en este entorno (sin GHDL/Vivado
-- disponibles). Revisar con el simulador antes de fiarse del resultado.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_spi_engine is
end entity tb_spi_engine;

architecture sim of tb_spi_engine is

    constant CLK_PERIOD : time := 10 ns; -- 100 MHz
    signal clk      : std_logic := '0';
    signal rstn     : std_logic := '0';
    signal sim_done : boolean   := false;

    signal start     : std_logic := '0';
    signal cs_sel    : integer range 0 to 1 := 0;
    signal addr_byte : std_logic_vector(7 downto 0) := (others => '0');
    signal n_data    : integer range 0 to 6 := 0;
    signal tx_data   : std_logic_vector(7 downto 0) := (others => '0');
    signal busy, done : std_logic;
    signal rx0, rx1, rx2, rx3, rx4, rx5 : std_logic_vector(7 downto 0);

    signal spi_sclk : std_logic;
    signal spi_mosi : std_logic;
    signal spi_miso : std_logic := '0';
    signal spi_cs_n : std_logic_vector(1 downto 0);

begin

    clk <= not clk after CLK_PERIOD / 2 when not sim_done else clk;

    dut : entity work.spi_engine
        generic map (
            G_CLK_DIV   => 4,  -- divisor pequeno para que la simulacion vaya rapido
            G_MAX_BYTES => 7
        )
        port map (
            clk       => clk,
            rstn      => rstn,
            start     => start,
            cs_sel    => cs_sel,
            addr_byte => addr_byte,
            n_data    => n_data,
            tx_data   => tx_data,
            busy      => busy,
            done      => done,
            rx_data0  => rx0,
            rx_data1  => rx1,
            rx_data2  => rx2,
            rx_data3  => rx3,
            rx_data4  => rx4,
            rx_data5  => rx5,
            spi_sclk  => spi_sclk,
            spi_mosi  => spi_mosi,
            spi_miso  => spi_miso,
            spi_cs_n  => spi_cs_n
        );

    ---------------------------------------------------------------------
    -- Esclavo SPI de prueba: mientras CS esta activo, en cada byte de
    -- datos devuelve 0xA5 bit a bit (MSB primero). No distingue entre
    -- escritura/lectura: el maestro simplemente ignora MISO cuando escribe.
    ---------------------------------------------------------------------
    slave_model : process
        constant TEST_BYTE : std_logic_vector(7 downto 0) := x"A5";
    begin
        loop
            spi_miso <= '0';
            wait until spi_cs_n(0) = '0' or spi_cs_n(1) = '0';
            loop
                for b in 7 downto 0 loop
                    wait until falling_edge(spi_sclk) or spi_cs_n = "11";
                    exit when spi_cs_n = "11";
                    spi_miso <= TEST_BYTE(b);
                    wait until rising_edge(spi_sclk) or spi_cs_n = "11";
                    exit when spi_cs_n = "11";
                end loop;
                exit when spi_cs_n = "11";
            end loop;
        end loop;
    end process;

    ---------------------------------------------------------------------
    -- Estimulo
    ---------------------------------------------------------------------
    stim : process
    begin
        rstn <= '0';
        wait for 100 ns;
        rstn <= '1';
        wait for 100 ns;

        -- Escritura: direccion 0x22 (CTRL_REG8), 1 byte de datos
        wait until rising_edge(clk);
        cs_sel    <= 0;
        addr_byte <= '0' & '0' & "100010";  -- RW=0 (escritura), MS=0, addr=0x22
        n_data    <= 1;
        tx_data   <= x"44";
        start     <= '1';
        wait until rising_edge(clk);
        start <= '0';
        wait until done = '1';
        report "Escritura de 1 byte completada (revisar en onda que CS/SCLK/MOSI tengan el patron esperado)";

        wait for 200 ns;

        -- Lectura en rafaga de 6 bytes: direccion 0x28, RW=1, MS=1
        wait until rising_edge(clk);
        cs_sel    <= 1;
        addr_byte <= '1' & '1' & "101000";
        n_data    <= 6;
        start     <= '1';
        wait until rising_edge(clk);
        start <= '0';
        wait until done = '1';

        assert (rx0 = x"A5") and (rx1 = x"A5") and (rx2 = x"A5")
           and (rx3 = x"A5") and (rx4 = x"A5") and (rx5 = x"A5")
            report "FALLO: los 6 bytes leidos no coinciden con el patron de prueba 0xA5"
            severity error;
        report "Si no ha aparecido ningun ERROR arriba, la lectura en rafaga es correcta";

        wait for 200 ns;
        sim_done <= true;
        wait;
    end process;

end architecture sim;
