-- axis_rr_mux4: arbitro AXI4-Stream de 4 entradas a 1 salida, round-robin por TRAMA.
--
-- * Cada entrada es un nav_spi_ctrl (tdata 32 bits, tvalid, tlast, tready).
-- * Cuando esta libre, mira las entradas empezando por la siguiente a la ultima atendida y
--   elige la primera con tvalid=1; entonces esa entrada queda "enganchada" y sus palabras
--   pasan tal cual (tdata/tvalid/tlast <-> tready) hasta la palabra con tlast. Despues se
--   libera y se vuelve a arbitrar.
-- * La salida lleva TID = indice de la entrada atendida (0..3).
-- * No depende de que haya datos en las demas entradas: con 1, 2, 3 o 4 IMUs conectadas
--   cada trama se atiende en cuanto llega (como mucho espera a la trama que se este
--   transfiriendo en ese momento y a las que tengan turno antes).
-- * Un ciclo de reloj de arbitraje por trama; tvalid/tdata de salida no dependen de tready.
--
-- Uso en Vivado: modulo de referencia (create_bd_cell -type module -reference axis_rr_mux4).
-- Los atributos X_INTERFACE_* hacen que Vivado vea los puertos como buses AXI4-Stream
-- (S0_AXIS..S3_AXIS esclavos, M_AXIS maestro) y como reloj/reset.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity axis_rr_mux4 is
    port (
        aclk    : in  std_logic;
        aresetn : in  std_logic;

        s0_axis_tdata  : in  std_logic_vector(31 downto 0);
        s0_axis_tvalid : in  std_logic;
        s0_axis_tlast  : in  std_logic;
        s0_axis_tready : out std_logic;

        s1_axis_tdata  : in  std_logic_vector(31 downto 0);
        s1_axis_tvalid : in  std_logic;
        s1_axis_tlast  : in  std_logic;
        s1_axis_tready : out std_logic;

        s2_axis_tdata  : in  std_logic_vector(31 downto 0);
        s2_axis_tvalid : in  std_logic;
        s2_axis_tlast  : in  std_logic;
        s2_axis_tready : out std_logic;

        s3_axis_tdata  : in  std_logic_vector(31 downto 0);
        s3_axis_tvalid : in  std_logic;
        s3_axis_tlast  : in  std_logic;
        s3_axis_tready : out std_logic;

        m_axis_tdata   : out std_logic_vector(31 downto 0);
        m_axis_tvalid  : out std_logic;
        m_axis_tlast   : out std_logic;
        m_axis_tid     : out std_logic_vector(1 downto 0);
        m_axis_tready  : in  std_logic
    );

    attribute X_INTERFACE_INFO : string;
    attribute X_INTERFACE_PARAMETER : string;

    attribute X_INTERFACE_INFO of aclk    : signal is "xilinx.com:signal:clock:1.0 aclk CLK";
    attribute X_INTERFACE_PARAMETER of aclk : signal is
        "ASSOCIATED_BUSIF S0_AXIS:S1_AXIS:S2_AXIS:S3_AXIS:M_AXIS, ASSOCIATED_RESET aresetn";
    attribute X_INTERFACE_INFO of aresetn : signal is "xilinx.com:signal:reset:1.0 aresetn RST";
    attribute X_INTERFACE_PARAMETER of aresetn : signal is "POLARITY ACTIVE_LOW";

    attribute X_INTERFACE_INFO of s0_axis_tdata  : signal is "xilinx.com:interface:axis:1.0 S0_AXIS TDATA";
    attribute X_INTERFACE_INFO of s0_axis_tvalid : signal is "xilinx.com:interface:axis:1.0 S0_AXIS TVALID";
    attribute X_INTERFACE_INFO of s0_axis_tlast  : signal is "xilinx.com:interface:axis:1.0 S0_AXIS TLAST";
    attribute X_INTERFACE_INFO of s0_axis_tready : signal is "xilinx.com:interface:axis:1.0 S0_AXIS TREADY";

    attribute X_INTERFACE_INFO of s1_axis_tdata  : signal is "xilinx.com:interface:axis:1.0 S1_AXIS TDATA";
    attribute X_INTERFACE_INFO of s1_axis_tvalid : signal is "xilinx.com:interface:axis:1.0 S1_AXIS TVALID";
    attribute X_INTERFACE_INFO of s1_axis_tlast  : signal is "xilinx.com:interface:axis:1.0 S1_AXIS TLAST";
    attribute X_INTERFACE_INFO of s1_axis_tready : signal is "xilinx.com:interface:axis:1.0 S1_AXIS TREADY";

    attribute X_INTERFACE_INFO of s2_axis_tdata  : signal is "xilinx.com:interface:axis:1.0 S2_AXIS TDATA";
    attribute X_INTERFACE_INFO of s2_axis_tvalid : signal is "xilinx.com:interface:axis:1.0 S2_AXIS TVALID";
    attribute X_INTERFACE_INFO of s2_axis_tlast  : signal is "xilinx.com:interface:axis:1.0 S2_AXIS TLAST";
    attribute X_INTERFACE_INFO of s2_axis_tready : signal is "xilinx.com:interface:axis:1.0 S2_AXIS TREADY";

    attribute X_INTERFACE_INFO of s3_axis_tdata  : signal is "xilinx.com:interface:axis:1.0 S3_AXIS TDATA";
    attribute X_INTERFACE_INFO of s3_axis_tvalid : signal is "xilinx.com:interface:axis:1.0 S3_AXIS TVALID";
    attribute X_INTERFACE_INFO of s3_axis_tlast  : signal is "xilinx.com:interface:axis:1.0 S3_AXIS TLAST";
    attribute X_INTERFACE_INFO of s3_axis_tready : signal is "xilinx.com:interface:axis:1.0 S3_AXIS TREADY";

    attribute X_INTERFACE_INFO of m_axis_tdata   : signal is "xilinx.com:interface:axis:1.0 M_AXIS TDATA";
    attribute X_INTERFACE_INFO of m_axis_tvalid  : signal is "xilinx.com:interface:axis:1.0 M_AXIS TVALID";
    attribute X_INTERFACE_INFO of m_axis_tlast   : signal is "xilinx.com:interface:axis:1.0 M_AXIS TLAST";
    attribute X_INTERFACE_INFO of m_axis_tid     : signal is "xilinx.com:interface:axis:1.0 M_AXIS TID";
    attribute X_INTERFACE_INFO of m_axis_tready  : signal is "xilinx.com:interface:axis:1.0 M_AXIS TREADY";
end entity axis_rr_mux4;

architecture rtl of axis_rr_mux4 is
    type data_arr_t is array (0 to 3) of std_logic_vector(31 downto 0);
    signal s_data  : data_arr_t;
    signal s_valid : std_logic_vector(3 downto 0);
    signal s_last  : std_logic_vector(3 downto 0);

    signal locked   : std_logic := '0';                      -- '1' mientras se transfiere una trama
    signal sel      : unsigned(1 downto 0) := (others => '0'); -- entrada enganchada
    signal last_sel : unsigned(1 downto 0) := "11";           -- ultima entrada atendida (prioridad rotatoria)
begin
    s_data  <= (s0_axis_tdata, s1_axis_tdata, s2_axis_tdata, s3_axis_tdata);
    s_valid <= s3_axis_tvalid & s2_axis_tvalid & s1_axis_tvalid & s0_axis_tvalid;
    s_last  <= s3_axis_tlast  & s2_axis_tlast  & s1_axis_tlast  & s0_axis_tlast;

    -- Camino de datos: pasa la entrada enganchada
    m_axis_tdata  <= s_data(to_integer(sel));
    m_axis_tlast  <= s_last(to_integer(sel));
    m_axis_tvalid <= s_valid(to_integer(sel)) when locked = '1' else '0';
    m_axis_tid    <= std_logic_vector(sel);

    s0_axis_tready <= m_axis_tready when (locked = '1' and sel = to_unsigned(0, 2)) else '0';
    s1_axis_tready <= m_axis_tready when (locked = '1' and sel = to_unsigned(1, 2)) else '0';
    s2_axis_tready <= m_axis_tready when (locked = '1' and sel = to_unsigned(2, 2)) else '0';
    s3_axis_tready <= m_axis_tready when (locked = '1' and sel = to_unsigned(3, 2)) else '0';

    -- Arbitraje
    process (aclk)
        variable idx : integer range 0 to 3;
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                locked   <= '0';
                sel      <= (others => '0');
                last_sel <= "11";
            else
                if locked = '0' then
                    -- primera entrada con tvalid a partir de la siguiente a la ultima atendida
                    for k in 1 to 4 loop
                        idx := (to_integer(last_sel) + k) mod 4;
                        if s_valid(idx) = '1' then
                            sel    <= to_unsigned(idx, 2);
                            locked <= '1';
                            exit;
                        end if;
                    end loop;
                else
                    -- fin de trama: ultima palabra aceptada
                    if s_valid(to_integer(sel)) = '1' and m_axis_tready = '1' and s_last(to_integer(sel)) = '1' then
                        locked   <= '0';
                        last_sel <= sel;
                    end if;
                end if;
            end if;
        end if;
    end process;
end architecture rtl;
