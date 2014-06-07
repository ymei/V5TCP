----------------------------------------------------------------------------------
-- Company:  LBNL
-- Engineer: Yuan Mei
-- 
-- Create Date:    10:59:38 08/24/2013 
-- Design Name: 
-- Module Name:    emac - Behavioral 
-- Project Name: 
-- Target Devices: xc5vlx50tff1136-1;
-- Tool versions: 
-- Description: 
--   Interface to ethernet MAC
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
USE IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
LIBRARY UNISIM;
USE UNISIM.VComponents.ALL;

ENTITY emac IS
  PORT (
    -- 125MHz clock for MAC
    GTX_CLK_0           : IN  std_logic;
    -- 200MHz reference clock for RGMII IODELAYs
    REFCLK              : IN  std_logic;
    -- Asynchronous Reset
    RESET               : IN  std_logic;
    -- GMII Interface - EMAC0
    GMII_TXD_0          : OUT std_logic_vector(7 DOWNTO 0);
    GMII_TX_EN_0        : OUT std_logic;
    GMII_TX_ER_0        : OUT std_logic;
    GMII_TX_CLK_0       : OUT std_logic;
    GMII_RXD_0          : IN  std_logic_vector(7 DOWNTO 0);
    GMII_RX_DV_0        : IN  std_logic;
    GMII_RX_ER_0        : IN  std_logic;
    GMII_RX_CLK_0       : IN  std_logic;
    MII_TX_CLK_0        : IN  std_logic;
    GMII_COL_0          : IN  std_logic;
    GMII_CRS_0          : IN  std_logic;
    PHY_RST_n           : OUT std_logic;
    -- Local link Receiver Interface - EMAC0
    RX_LL_DATA_0        : OUT std_logic_vector(7 DOWNTO 0);
    RX_LL_SOF_N_0       : OUT std_logic;
    RX_LL_EOF_N_0       : OUT std_logic;
    RX_LL_SRC_RDY_N_0   : OUT std_logic;
    RX_LL_DST_RDY_N_0   : IN  std_logic;
    -- Local link Transmitter Interface - EMAC0
    TX_LL_DATA_0        : IN  std_logic_vector(7 DOWNTO 0);
    TX_LL_SOF_N_0       : IN  std_logic;
    TX_LL_EOF_N_0       : IN  std_logic;
    TX_LL_SRC_RDY_N_0   : IN  std_logic;
    TX_LL_DST_RDY_N_0   : OUT std_logic
  );
END emac;

ARCHITECTURE Behavioral OF emac IS

  -- Component Declaration for the TEMAC wrapper with 
  -- Local Link FIFO.
  COMPONENT V5EMAC_locallink IS
    PORT(
      -- EMAC0 Clocking
      -- TX Client Clock output from EMAC0
      TX_CLIENT_CLK_OUT_0 : OUT std_logic;
      -- RX Client Clock output from EMAC0
      RX_CLIENT_CLK_OUT_0 : OUT std_logic;
      -- TX PHY Clock output from EMAC0
      TX_PHY_CLK_OUT_0    : OUT std_logic;
      -- EMAC0 TX Client Clock input from BUFG
      TX_CLIENT_CLK_0     : IN  std_logic;
      -- EMAC0 RX Client Clock input from BUFG
      RX_CLIENT_CLK_0     : IN  std_logic;
      -- EMAC0 TX PHY Clock input from BUFG
      TX_PHY_CLK_0        : IN  std_logic;
      -- Speed indicator for EMAC0
      -- Used in clocking circuitry in example_design file.
      EMAC0SPEEDIS10100   : OUT std_logic;

      -- Local link Receiver Interface - EMAC0
      RX_LL_CLOCK_0       : IN  std_logic;
      RX_LL_RESET_0       : IN  std_logic;
      RX_LL_DATA_0        : OUT std_logic_vector(7 DOWNTO 0);
      RX_LL_SOF_N_0       : OUT std_logic;
      RX_LL_EOF_N_0       : OUT std_logic;
      RX_LL_SRC_RDY_N_0   : OUT std_logic;
      RX_LL_DST_RDY_N_0   : IN  std_logic;
      RX_LL_FIFO_STATUS_0 : OUT std_logic_vector(3 DOWNTO 0);

      -- Local link Transmitter Interface - EMAC0
      TX_LL_CLOCK_0     : IN  std_logic;
      TX_LL_RESET_0     : IN  std_logic;
      TX_LL_DATA_0      : IN  std_logic_vector(7 DOWNTO 0);
      TX_LL_SOF_N_0     : IN  std_logic;
      TX_LL_EOF_N_0     : IN  std_logic;
      TX_LL_SRC_RDY_N_0 : IN  std_logic;
      TX_LL_DST_RDY_N_0 : OUT std_logic;

      -- Client Receiver Interface - EMAC0
      EMAC0CLIENTRXDVLD         : OUT std_logic;
      EMAC0CLIENTRXFRAMEDROP    : OUT std_logic;
      EMAC0CLIENTRXSTATS        : OUT std_logic_vector(6 DOWNTO 0);
      EMAC0CLIENTRXSTATSVLD     : OUT std_logic;
      EMAC0CLIENTRXSTATSBYTEVLD : OUT std_logic;

      -- Client Transmitter Interface - EMAC0
      CLIENTEMAC0TXIFGDELAY     : IN  std_logic_vector(7 DOWNTO 0);
      EMAC0CLIENTTXSTATS        : OUT std_logic;
      EMAC0CLIENTTXSTATSVLD     : OUT std_logic;
      EMAC0CLIENTTXSTATSBYTEVLD : OUT std_logic;

      -- MAC Control Interface - EMAC0
      CLIENTEMAC0PAUSEREQ : IN std_logic;
      CLIENTEMAC0PAUSEVAL : IN std_logic_vector(15 DOWNTO 0);

      -- Clock Signals - EMAC0
      GTX_CLK_0 : IN std_logic;

      -- GMII Interface - EMAC0
      GMII_TXD_0    : OUT std_logic_vector(7 DOWNTO 0);
      GMII_TX_EN_0  : OUT std_logic;
      GMII_TX_ER_0  : OUT std_logic;
      GMII_TX_CLK_0 : OUT std_logic;
      GMII_RXD_0    : IN  std_logic_vector(7 DOWNTO 0);
      GMII_RX_DV_0  : IN  std_logic;
      GMII_RX_ER_0  : IN  std_logic;
      GMII_RX_CLK_0 : IN  std_logic;

      MII_TX_CLK_0 : IN std_logic;
      GMII_COL_0   : IN std_logic;
      GMII_CRS_0   : IN std_logic;

      -- Asynchronous Reset
      RESET : IN std_logic
    );
  END COMPONENT;

-----------------------------------------------------------------------
-- Signal Declarations
-----------------------------------------------------------------------
  -- Global asynchronous reset
  SIGNAL reset_i : std_logic;

  -- client interface clocking signals - EMAC0
  SIGNAL ll_clk_0_i : std_logic;

  -- address swap transmitter connections - EMAC0
  SIGNAL tx_ll_data_0_i      : std_logic_vector(7 DOWNTO 0);
  SIGNAL tx_ll_sof_n_0_i     : std_logic;
  SIGNAL tx_ll_eof_n_0_i     : std_logic;
  SIGNAL tx_ll_src_rdy_n_0_i : std_logic;
  SIGNAL tx_ll_dst_rdy_n_0_i : std_logic;

  -- address swap receiver connections - EMAC0
  SIGNAL rx_ll_data_0_i      : std_logic_vector(7 DOWNTO 0);
  SIGNAL rx_ll_sof_n_0_i     : std_logic;
  SIGNAL rx_ll_eof_n_0_i     : std_logic;
  SIGNAL rx_ll_src_rdy_n_0_i : std_logic;
  SIGNAL rx_ll_dst_rdy_n_0_i : std_logic;

  -- create a synchronous reset in the transmitter clock domain
  SIGNAL ll_pre_reset_0_i : std_logic_vector(5 DOWNTO 0);
  SIGNAL ll_reset_0_i     : std_logic;

  ATTRIBUTE async_reg                     : string;
  ATTRIBUTE async_reg OF ll_pre_reset_0_i : SIGNAL IS "true";

  -- Reference clock for RGMII IODELAYs
  SIGNAL refclk_ibufg_i : std_logic;
  SIGNAL refclk_bufg_i  : std_logic;

  -- EMAC0 Clocking signals

  -- GMII input clocks to wrappers
  SIGNAL tx_clk_0            : std_logic;
  SIGNAL rx_clk_0_i          : std_logic;
  SIGNAL gmii_rx_clk_0_delay : std_logic;

  -- IDELAY controller
  SIGNAL idelayctrl_reset_0_r : std_logic_vector(12 DOWNTO 0);
  SIGNAL idelayctrl_reset_0_i : std_logic;

  -- Setting attribute for RGMII/GMII IDELAY
  -- For more information on IDELAYCTRL and IDELAY, please refer to
  -- the Virtex-5 User Guide.
  ATTRIBUTE syn_noprune             : boolean;
  ATTRIBUTE syn_noprune OF dlyctrl0 : LABEL IS true;

  -- GMII client clocks
  SIGNAL tx_client_clk_0_o : std_logic;
  SIGNAL tx_client_clk_0   : std_logic;
  SIGNAL rx_client_clk_0_o : std_logic;
  SIGNAL rx_client_clk_0   : std_logic;
  -- GMII PHY clocks
  SIGNAL tx_phy_clk_0_o    : std_logic;
  SIGNAL tx_phy_clk_0      : std_logic;

  -- Speed indication from EMAC wrappers
  SIGNAL speed_vector_0_i : std_logic;

  ATTRIBUTE buffer_type                : string;
  SIGNAL gtx_clk_0_i                   : std_logic;
  ATTRIBUTE buffer_type OF gtx_clk_0_i : SIGNAL IS "none";

BEGIN

  PHY_RST_n <= NOT RESET;
  reset_i   <= RESET;

  -- Local link Receiver Interface - EMAC0
  RX_LL_DATA_0        <= rx_ll_data_0_i;
  RX_LL_SOF_N_0       <= rx_ll_sof_n_0_i;
  RX_LL_EOF_N_0       <= rx_ll_eof_n_0_i;
  RX_LL_SRC_RDY_N_0   <= rx_ll_src_rdy_n_0_i;
  rx_ll_dst_rdy_n_0_i <= RX_LL_DST_RDY_N_0;
  -- Local link Transmitter Interface - EMAC0
  tx_ll_data_0_i      <= TX_LL_DATA_0;
  tx_ll_sof_n_0_i     <= TX_LL_SOF_N_0;
  tx_ll_eof_n_0_i     <= TX_LL_EOF_N_0;
  tx_ll_src_rdy_n_0_i <= TX_LL_SRC_RDY_N_0;
  TX_LL_DST_RDY_N_0   <= tx_ll_dst_rdy_n_0_i;

  -- EMAC0 Clocking

  -- Use IDELAY on GMII_RX_CLK_0 to move the clock into
  -- alignment with the data

  -- Instantiate IDELAYCTRL for the IDELAY in Fixed Tap Delay Mode
  dlyctrl0 : IDELAYCTRL PORT MAP (
    RDY    => OPEN,
    REFCLK => refclk_bufg_i,
    RST    => idelayctrl_reset_0_i
  );

  delay0rstgen : PROCESS (refclk_bufg_i, reset_i)
  BEGIN
    IF (reset_i = '1') THEN
      idelayctrl_reset_0_r(0)           <= '0';
      idelayctrl_reset_0_r(12 DOWNTO 1) <= (OTHERS => '1');
    ELSIF refclk_bufg_i'event AND refclk_bufg_i = '1' THEN
      idelayctrl_reset_0_r(0)           <= '0';
      idelayctrl_reset_0_r(12 DOWNTO 1) <= idelayctrl_reset_0_r(11 DOWNTO 0);
    END IF;
  END PROCESS delay0rstgen;

  idelayctrl_reset_0_i <= idelayctrl_reset_0_r(12);

  -- Please modify the value of the IOBDELAYs according to your design.
  -- For more information on IDELAYCTRL and IODELAY, please refer to
  -- the Virtex-5 User Guide.
  gmii_rxc0_delay : IODELAY
    GENERIC MAP (
      IDELAY_TYPE    => "FIXED",
      IDELAY_VALUE   => 0,
      DELAY_SRC      => "I",
      SIGNAL_PATTERN => "CLOCK"
    )
    PORT MAP (
      IDATAIN => GMII_RX_CLK_0,
      ODATAIN => '0',
      DATAOUT => gmii_rx_clk_0_delay,
      DATAIN  => '0',
      C       => '0',
      T       => '0',
      CE      => '0',
      INC     => '0',
      RST     => '0'
    );

  -- Put the PHY clocks from the EMAC through BUFGs.
  -- Used to clock the PHY side of the EMAC wrappers.
  bufg_phy_tx_0 : BUFG PORT MAP (I => tx_phy_clk_0_o, O => tx_phy_clk_0);
  bufg_phy_rx_0 : BUFG PORT MAP (I => gmii_rx_clk_0_delay, O => rx_clk_0_i);

  -- Put the client clocks from the EMAC through BUFGs.
  -- Used to clock the client side of the EMAC wrappers.
  bufg_client_tx_0 : BUFG PORT MAP (I => tx_client_clk_0_o, O => tx_client_clk_0);
  bufg_client_rx_0 : BUFG PORT MAP (I => rx_client_clk_0_o, O => rx_client_clk_0);

  ll_clk_0_i <= tx_client_clk_0;

  -- Enable PHYEMAC0MIITXCLK to continue toggling even when MII_TX_CLK_0 signal 
  -- stops toggling due when TEMAC changes to 1 Gbps mode
  bufg_tx_0 : BUFGMUX PORT MAP (
    I0 => gtx_clk_0_i,
    I1 => MII_TX_CLK_0,
    S  => speed_vector_0_i,
    O  => tx_clk_0
  );

  ------------------------------------------------------------------------
  -- Instantiate the EMAC Wrapper with LL FIFO 
  -- (V5EMAC_locallink.v)
  ------------------------------------------------------------------------
  v5_emac_ll : V5EMAC_locallink
    PORT MAP (
      -- EMAC0 Clocking
      -- TX Client Clock output from EMAC0
      TX_CLIENT_CLK_OUT_0 => tx_client_clk_0_o,
      -- RX Client Clock output from EMAC0
      RX_CLIENT_CLK_OUT_0 => rx_client_clk_0_o,
      -- TX PHY Clock output from EMAC0
      TX_PHY_CLK_OUT_0    => tx_phy_clk_0_o,
      -- EMAC0 TX Client Clock input from BUFG
      TX_CLIENT_CLK_0     => tx_client_clk_0,
      -- EMAC0 RX Client Clock input from BUFG
      RX_CLIENT_CLK_0     => rx_client_clk_0,
      -- EMAC0 TX PHY Clock input from BUFG
      TX_PHY_CLK_0        => tx_phy_clk_0,
      -- Speed indicator for EMAC0
      -- Used in clocking circuitry.
      EMAC0SPEEDIS10100   => speed_vector_0_i,
      -- Local link Receiver Interface - EMAC0
      RX_LL_CLOCK_0       => ll_clk_0_i,
      RX_LL_RESET_0       => ll_reset_0_i,
      RX_LL_DATA_0        => rx_ll_data_0_i,
      RX_LL_SOF_N_0       => rx_ll_sof_n_0_i,
      RX_LL_EOF_N_0       => rx_ll_eof_n_0_i,
      RX_LL_SRC_RDY_N_0   => rx_ll_src_rdy_n_0_i,
      RX_LL_DST_RDY_N_0   => rx_ll_dst_rdy_n_0_i,
      RX_LL_FIFO_STATUS_0 => OPEN,

      -- Unused Receiver signals - EMAC0
      EMAC0CLIENTRXDVLD         => OPEN,
      EMAC0CLIENTRXFRAMEDROP    => OPEN,
      EMAC0CLIENTRXSTATS        => OPEN,
      EMAC0CLIENTRXSTATSVLD     => OPEN,
      EMAC0CLIENTRXSTATSBYTEVLD => OPEN,

      -- Local link Transmitter Interface - EMAC0
      TX_LL_CLOCK_0     => ll_clk_0_i,
      TX_LL_RESET_0     => ll_reset_0_i,
      TX_LL_DATA_0      => tx_ll_data_0_i,
      TX_LL_SOF_N_0     => tx_ll_sof_n_0_i,
      TX_LL_EOF_N_0     => tx_ll_eof_n_0_i,
      TX_LL_SRC_RDY_N_0 => tx_ll_src_rdy_n_0_i,
      TX_LL_DST_RDY_N_0 => tx_ll_dst_rdy_n_0_i,

      -- Unused Transmitter signals - EMAC0
      CLIENTEMAC0TXIFGDELAY     => (OTHERS => '0'),
      EMAC0CLIENTTXSTATS        => OPEN,
      EMAC0CLIENTTXSTATSVLD     => OPEN,
      EMAC0CLIENTTXSTATSBYTEVLD => OPEN,

      -- MAC Control Interface - EMAC0
      CLIENTEMAC0PAUSEREQ => '0',
      CLIENTEMAC0PAUSEVAL => (OTHERS => '0'),

      -- Clock Signals - EMAC0
      GTX_CLK_0     => gtx_clk_0_i,
      -- GMII Interface - EMAC0
      GMII_TXD_0    => GMII_TXD_0,
      GMII_TX_EN_0  => GMII_TX_EN_0,
      GMII_TX_ER_0  => GMII_TX_ER_0,
      GMII_TX_CLK_0 => GMII_TX_CLK_0,
      GMII_RXD_0    => GMII_RXD_0,
      GMII_RX_DV_0  => GMII_RX_DV_0,
      GMII_RX_ER_0  => GMII_RX_ER_0,
      GMII_RX_CLK_0 => rx_clk_0_i,

      MII_TX_CLK_0 => tx_clk_0,
      GMII_COL_0   => GMII_COL_0,
      GMII_CRS_0   => GMII_CRS_0,

      -- Asynchronous Reset
      RESET => reset_i
    );

  -- Create synchronous reset in the transmitter clock domain.
  gen_ll_reset_emac0 : PROCESS (ll_clk_0_i, reset_i)
  BEGIN
    IF reset_i = '1' THEN
      ll_pre_reset_0_i <= (OTHERS => '1');
      ll_reset_0_i     <= '1';
    ELSIF ll_clk_0_i'event AND ll_clk_0_i = '1' THEN
      ll_pre_reset_0_i(0)          <= '0';
      ll_pre_reset_0_i(5 DOWNTO 1) <= ll_pre_reset_0_i(4 DOWNTO 0);
      ll_reset_0_i                 <= ll_pre_reset_0_i(5);
    END IF;
  END PROCESS gen_ll_reset_emac0;

  ------------------------------------------------------------------------
  -- REFCLK used for RGMII IODELAYCTRL primitive - Need to supply a 200MHz clock
  ------------------------------------------------------------------------
  --refclk_ibufg : IBUFG port map(I => REFCLK, O => refclk_ibufg_i);
  --refclk_bufg  : BUFG  port map(I => refclk_ibufg_i, O => refclk_bufg_i);
  refclk_bufg_i <= REFCLK;

  ----------------------------------------------------------------------
  -- Cause the tools to automatically choose a GC pin for the
  -- GTX_CLK_0 line.
  ----------------------------------------------------------------------
  --gtx_clk0_ibuf : IBUFG port map (I => GTX_CLK_0, O => gtx_clk_0_i);
  gtx_clk_0_i <= GTX_CLK_0;

END Behavioral;
