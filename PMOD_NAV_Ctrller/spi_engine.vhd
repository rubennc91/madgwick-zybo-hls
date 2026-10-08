-------------------------------------------------------------------------------
-- spi_engine.vhd
-- Motor SPI generico de bajo nivel para el bus compartido del Pmod NAV.
-- Modo SPI 3 (CPOL=1, CPHA=1): reloj en reposo a nivel alto, MOSI cambia
-- en flanco de bajada de SCLK, MISO se captura en flanco de subida.
--
-- Transaccion = 1 byte de direccion + N bytes de datos (N=0..6), todo bajo
-- el mismo CS. El byte de direccion lo prepara quien instancia este motor
-- (con los bits RW/MS ya puestos). Para escrituras, tx_data trae el byte
-- a escribir en tx_data(0). Para lecturas, tx_data se ignora (se envian
-- ceros) y los bytes leidos aparecen en rx_data.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity spi_engine is
    generic (
        G_CLK_DIV   : integer := 10;  -- divisor de clk de sistema -> SCLK (10 => 100MHz/10=10MHz)
        G_MAX_BYTES : integer := 7    -- 1 direccion + hasta 6 bytes de datos (rafaga XYZ)
    );
    port (
        clk       : in  std_logic;
        rstn      : in  std_logic;

        -- Control de transaccion
        start     : in  std_logic;                         -- pulso de 1 ciclo para iniciar
        cs_sel    : in  integer range 0 to 1;               -- 0=CS_AG, 1=CS_MAG
        addr_byte : in  std_logic_vector(7 downto 0);       -- byte de direccion (con RW/MS)
        n_data    : in  integer range 0 to G_MAX_BYTES - 1; -- nº de bytes de datos tras la direccion
        tx_data   : in  std_logic_vector(7 downto 0);       -- byte a escribir (si n_data=1 y es escritura)
        busy      : out std_logic;
        done      : out std_logic;                          -- pulso de 1 ciclo al terminar
        rx_data0  : out std_logic_vector(7 downto 0);
        rx_data1  : out std_logic_vector(7 downto 0);
        rx_data2  : out std_logic_vector(7 downto 0);
        rx_data3  : out std_logic_vector(7 downto 0);
        rx_data4  : out std_logic_vector(7 downto 0);
        rx_data5  : out std_logic_vector(7 downto 0);

        -- Pines fisicos (a mapear al bridge del Pmod)
        spi_sclk  : out std_logic;
        spi_mosi  : out std_logic;
        spi_miso  : in  std_logic;
        spi_cs_n  : out std_logic_vector(1 downto 0)
    );
end entity spi_engine;

architecture rtl of spi_engine is

    type state_t is (S_IDLE, S_CS_SETUP, S_XFER_LOW, S_XFER_HIGH, S_CS_HOLD, S_DONE);
    signal state : state_t := S_IDLE;

    signal div_cnt    : integer range 0 to G_CLK_DIV - 1 := 0;
    signal byte_idx   : integer range 0 to G_MAX_BYTES - 1 := 0;
    signal bit_idx    : integer range 0 to 7 := 0;
    signal shift_out  : std_logic_vector(7 downto 0) := (others => '0');

    type rx_array_t is array (0 to G_MAX_BYTES - 2) of std_logic_vector(7 downto 0);
    signal rx_bytes   : rx_array_t := (others => (others => '0'));

    signal n_data_r   : integer range 0 to G_MAX_BYTES - 1 := 0;
    signal cs_sel_r   : integer range 0 to 1 := 0;
    signal total_bytes: integer range 1 to G_MAX_BYTES := 1;

begin

    process (clk, rstn)
    begin
        if rstn = '0' then
            state     <= S_IDLE;
            spi_sclk  <= '1';           -- reposo a nivel alto (CPOL=1)
            spi_cs_n  <= (others => '1');
            busy      <= '0';
            done      <= '0';
            div_cnt   <= 0;
            byte_idx  <= 0;
            bit_idx   <= 0;

        elsif rising_edge(clk) then
            done <= '0';

            case state is

                when S_IDLE =>
                    spi_sclk <= '1';
                    if start = '1' then
                        cs_sel_r    <= cs_sel;
                        n_data_r    <= n_data;
                        total_bytes <= 1 + n_data;
                        shift_out   <= addr_byte;
                        byte_idx    <= 0;
                        bit_idx     <= 0;
                        div_cnt     <= 0;
                        busy        <= '1';
                        spi_cs_n    <= (others => '1');
                        state       <= S_CS_SETUP;
                    else
                        busy <= '0';
                    end if;

                when S_CS_SETUP =>
                    -- Activa el CS del subdispositivo seleccionado y espera
                    -- un ciclo de reloj dividido de margen antes del primer flanco.
                    spi_cs_n(cs_sel_r) <= '0';
                    if div_cnt = G_CLK_DIV - 1 then
                        div_cnt <= 0;
                        state   <= S_XFER_LOW;
                    else
                        div_cnt <= div_cnt + 1;
                    end if;

                when S_XFER_LOW =>
                    -- Flanco de bajada: SCLK a '0' y se pone el bit en MOSI.
                    if div_cnt = G_CLK_DIV - 1 then
                        div_cnt  <= 0;
                        spi_sclk <= '0';
                        spi_mosi <= shift_out(7);
                        state    <= S_XFER_HIGH;
                    else
                        div_cnt <= div_cnt + 1;
                    end if;

                when S_XFER_HIGH =>
                    -- Flanco de subida: SCLK a '1' y se captura MISO.
                    if div_cnt = G_CLK_DIV - 1 then
                        div_cnt  <= 0;
                        spi_sclk <= '1';
                        shift_out <= shift_out(6 downto 0) & spi_miso;

                        if bit_idx = 7 then
                            bit_idx <= 0;
                            -- Guarda el byte recibido (salvo el de direccion, byte_idx=0)
                            if byte_idx > 0 then
                                rx_bytes(byte_idx - 1) <=
                                    shift_out(6 downto 0) & spi_miso;
                            end if;

                            if byte_idx = total_bytes - 1 then
                                state <= S_CS_HOLD;
                            else
                                byte_idx  <= byte_idx + 1;
                                shift_out <= tx_data; -- bytes de datos tras la direccion
                                state     <= S_XFER_LOW;
                            end if;
                        else
                            bit_idx <= bit_idx + 1;
                            state   <= S_XFER_LOW;
                        end if;
                    else
                        div_cnt <= div_cnt + 1;
                    end if;

                when S_CS_HOLD =>
                    if div_cnt = G_CLK_DIV - 1 then
                        div_cnt  <= 0;
                        spi_cs_n <= (others => '1');
                        state    <= S_DONE;
                    else
                        div_cnt <= div_cnt + 1;
                    end if;

                when S_DONE =>
                    busy  <= '0';
                    done  <= '1';
                    state <= S_IDLE;

            end case;
        end if;
    end process;

    rx_data0 <= rx_bytes(0);
    rx_data1 <= rx_bytes(1);
    rx_data2 <= rx_bytes(2);
    rx_data3 <= rx_bytes(3);
    rx_data4 <= rx_bytes(4);
    rx_data5 <= rx_bytes(5);

end architecture rtl;
