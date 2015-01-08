--------------------------------------------------------------------------------
--
-- Company: LBNL
-- Engineer: Yuan Mei
-- 
-- Create Date:    02:00:18 08/23/2013 
-- Design Name:    V5TCP
-- Module Name:    top - Behavioral 
-- Project Name: 
-- Target Devices: Virtex-5 xc5vlx50t-1ff1136
-- Tool versions: ISE 14.5
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
--------------------------------------------------------------------------------
--
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
USE ieee.numeric_std.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
LIBRARY UNISIM;
USE UNISIM.VComponents.ALL;

ENTITY top IS
  GENERIC (
    includeChipscope : boolean := true
  );
  PORT (
    clk_xtal      : IN    std_logic;
    rst_sys_n     : IN    std_logic;
    GCLK4_P       : IN    std_logic;
    GCLK4_N       : IN    std_logic;
    GCLK5_P       : IN    std_logic;
    GCLK5_N       : IN    std_logic;
    --
    LED8Bit       : OUT   std_logic_vector(7 DOWNTO 0);
    DIPSw8Bit     : IN    std_logic_vector(7 DOWNTO 0);
    BTN           : IN    std_logic_vector(6 DOWNTO 0);
    --
    JA            : INOUT std_logic_vector(7 DOWNTO 0);
    JB            : INOUT std_logic_vector(7 DOWNTO 0);
    JC            : INOUT std_logic_vector(7 DOWNTO 0);
    JD            : INOUT std_logic_vector(7 DOWNTO 0);
    --
    UART_RX_PIN   : IN    std_logic;
    UART_TX_PIN   : OUT   std_logic;
    -- GMII Interface - EMAC0
    GMII_TXD_0    : OUT   std_logic_vector(7 DOWNTO 0);
    GMII_TX_EN_0  : OUT   std_logic;
    GMII_TX_ER_0  : OUT   std_logic;
    GMII_TX_CLK_0 : OUT   std_logic;
    GMII_RXD_0    : IN    std_logic_vector(7 DOWNTO 0);
    GMII_RX_DV_0  : IN    std_logic;
    GMII_RX_ER_0  : IN    std_logic;
    GMII_RX_CLK_0 : IN    std_logic;
    MII_TX_CLK_0  : IN    std_logic;
    GMII_COL_0    : IN    std_logic;
    GMII_CRS_0    : IN    std_logic;
    PHY_RST_n     : OUT   std_logic;
    -- VHDCI
    VHDCI1P       : INOUT std_logic_vector(19 DOWNTO 0);
    VHDCI1N       : INOUT std_logic_vector(19 DOWNTO 0);
    VHDCI2P       : INOUT std_logic_vector(19 DOWNTO 0);
    VHDCI2N       : INOUT std_logic_vector(19 DOWNTO 0)
  );
END top;

ARCHITECTURE Behavioral OF top IS
  -- Components
  COMPONENT clock_generator
    PORT (
      -- Clock in ports
      CLKIN_IN        : IN  std_logic;  -- 100MHz In
      -- Clock out ports
      CLK0_OUT        : OUT std_logic;  -- 100MHz
      CLK2X_OUT       : OUT std_logic;  -- 200MHz
      CLKDV_OUT       : OUT std_logic;  -- 50MHz
      CLKFX_OUT       : OUT std_logic;  -- 125MHz
      CLKIN_IBUFG_OUT : OUT std_logic;
      -- Status and control signals
      RST_IN          : IN  std_logic;
      LOCKED_OUT      : OUT std_logic
    );
  END COMPONENT;
  COMPONENT GlobalResetter
    PORT (
    FORCE_RST_n : IN  std_logic := '1';
    CLK         : IN  std_logic;        -- system clock
    DCM_LOCKED  : IN  std_logic;
    CLK_RST     : OUT std_logic;
    GLOBAL_RST  : OUT std_logic
  );
  END COMPONENT;
  ---------------------------------------------< gig_eth
  COMPONENT gig_eth
    PORT (
      -- asynchronous reset
      RESET                : IN  std_logic;
      -- clocks
      CLK125               : IN  std_logic;  -- 125MHz
      REFCLK               : IN  std_logic;  -- 200MHz
      -- GMII Interface - EMAC0
      GMII_TXD_0           : OUT std_logic_vector(7 DOWNTO 0);
      GMII_TX_EN_0         : OUT std_logic;
      GMII_TX_ER_0         : OUT std_logic;
      GMII_TX_CLK_0        : OUT std_logic;
      GMII_RXD_0           : IN  std_logic_vector(7 DOWNTO 0);
      GMII_RX_DV_0         : IN  std_logic;
      GMII_RX_ER_0         : IN  std_logic;
      GMII_RX_CLK_0        : IN  std_logic;
      MII_TX_CLK_0         : IN  std_logic;
      GMII_COL_0           : IN  std_logic;
      GMII_CRS_0           : IN  std_logic;
      PHY_RST_n            : OUT std_logic;
      -- TCP
      TCP_CONNECTION_RESET : IN  std_logic;
      TX_TDATA             : IN  std_logic_vector(7 DOWNTO 0);
      TX_TVALID            : IN  std_logic;
      TX_TREADY            : OUT std_logic;
      RX_TDATA             : OUT std_logic_vector(7 DOWNTO 0);
      RX_TVALID            : OUT std_logic;
      RX_TREADY            : IN  std_logic;
      -- FIFO
      TCP_USE_FIFO         : IN  std_logic;
      TX_FIFO_WRCLK        : IN  std_logic;
      TX_FIFO_Q            : IN  std_logic_vector(31 DOWNTO 0);
      TX_FIFO_WREN         : IN  std_logic;
      TX_FIFO_FULL         : OUT std_logic;
      RX_FIFO_RDCLK        : IN  std_logic;
      RX_FIFO_Q            : OUT std_logic_vector(31 DOWNTO 0);
      RX_FIFO_RDEN         : IN  std_logic;
      RX_FIFO_EMPTY        : OUT std_logic
    );
  END COMPONENT;
  ---------------------------------------------> gig_eth
  ---------------------------------------------< UART/RS232
  COMPONENT uartio
    GENERIC (
      -- tick repetition frequency is (input freq) / (2**COUNTER_WIDTH / DIVISOR)
      COUNTER_WIDTH : positive;
      DIVISOR       : positive
    );
    PORT (
      CLK     : IN  std_logic;
      RESET   : IN  std_logic;
      RX_DATA : OUT std_logic_vector(7 DOWNTO 0);
      RX_RDY  : OUT std_logic;
      TX_DATA : IN  std_logic_vector(7 DOWNTO 0);
      TX_EN   : IN  std_logic;
      TX_RDY  : OUT std_logic;
      -- serial lines
      RX_PIN  : IN  std_logic;
      TX_PIN  : OUT std_logic
    );
  END COMPONENT;
  COMPONENT byte2cmd
    PORT (
      CLK            : IN  std_logic;
      RESET          : IN  std_logic;
      -- byte in
      RX_DATA        : IN  std_logic_vector(7 DOWNTO 0);
      RX_RDY         : IN  std_logic;
      -- cmd out
      CMD_FIFO_Q     : OUT std_logic_vector(35 DOWNTO 0);  -- command fifo data out port
      CMD_FIFO_EMPTY : OUT std_logic;   -- command fifo "emtpy" SIGNAL
      CMD_FIFO_RDCLK : IN  std_logic;
      CMD_FIFO_RDREQ : IN  std_logic    -- command fifo read request
    );
  END COMPONENT;
  COMPONENT control_interface
    PORT (
      RESET           : IN  std_logic;
      CLK             : IN  std_logic;    -- system clock
      -- From FPGA to PC
      FIFO_Q          : OUT std_logic_vector(35 DOWNTO 0);  -- interface fifo data output port
      FIFO_EMPTY      : OUT std_logic;    -- interface fifo "emtpy" signal
      FIFO_RDREQ      : IN  std_logic;    -- interface fifo read request
      FIFO_RDCLK      : IN  std_logic;    -- interface fifo read clock
      -- From PC to FPGA, FWFT
      CMD_FIFO_Q      : IN  std_logic_vector(35 DOWNTO 0);  -- interface command fifo data out port
      CMD_FIFO_EMPTY  : IN  std_logic;    -- interface command fifo "emtpy" signal
      CMD_FIFO_RDREQ  : OUT std_logic;    -- interface command fifo read request
      -- Digital I/O
      CONFIG_REG      : OUT std_logic_vector(511 DOWNTO 0); -- thirtytwo 16bit registers
      PULSE_REG       : OUT std_logic_vector(15 DOWNTO 0);  -- 16bit pulse register
      STATUS_REG      : IN  std_logic_vector(175 DOWNTO 0); -- eleven 16bit registers
      -- Memory interface
      MEM_WE          : OUT std_logic;    -- memory write enable
      MEM_ADDR        : OUT std_logic_vector(31 DOWNTO 0);
      MEM_DIN         : OUT std_logic_vector(31 DOWNTO 0);  -- memory data input
      MEM_DOUT        : IN  std_logic_vector(31 DOWNTO 0);  -- memory data output
      -- Data FIFO interface, FWFT
      DATA_FIFO_Q     : IN  std_logic_vector(31 DOWNTO 0);
      DATA_FIFO_EMPTY : IN  std_logic;
      DATA_FIFO_RDREQ : OUT std_logic;
      DATA_FIFO_RDCLK : OUT std_logic
    );
  END COMPONENT;
  ---------------------------------------------> UART/RS232
  ---------------------------------------------< Topmetal
  COMPONENT topmetal_simple
    GENERIC (
      TRIGGER_DELAY_WIDTH  : positive := 16
    );
    PORT(
      RST                  : IN  std_logic;
      CLK                  : IN  std_logic;
      SWG                  : IN  std_logic_vector(7 DOWNTO 0);
      BTN                  : IN  std_logic_vector(6 DOWNTO 0);
      MARKER_IN            : IN  std_logic;
      MARKER_OUT           : OUT std_logic;
      STOP_CONTROL         : IN  std_logic;
      STOP_ADDRESS         : IN  std_logic_vector(9 DOWNTO 0);
      TRIGGER_CONTROL      : IN  std_logic;
      TRIGGER_RATE_CONTROL : IN  std_logic;
      TRIGGER_RATE         : IN  std_logic_vector (3 DOWNTO 0);
      TRIGGER_DELAY        : IN  std_logic_vector (TRIGGER_DELAY_WIDTH-1 DOWNTO 0);
      TRIGGER_OUT          : OUT std_logic;
      TM_CLK               : OUT std_logic;
      TM_RST               : OUT std_logic;
      TM_START             : OUT std_logic;
      TM_SPEAK             : OUT std_logic;
      EX_RST_n             : OUT std_logic
    );
  END COMPONENT;
  COMPONENT dac_inter8568
    PORT(
      RESET : IN  std_logic;
      CLK   : IN  std_logic;
      DATAA : IN  std_logic_vector(15 DOWNTO 0);
      DATAC : IN  std_logic_vector(15 DOWNTO 0);
      DATAE : IN  std_logic_vector(15 DOWNTO 0);
      DATAG : IN  std_logic_vector(15 DOWNTO 0);
      DIN   : OUT std_logic;
      SCLK  : OUT std_logic;
      SYN   : OUT std_logic
    );
  END COMPONENT;
  COMPONENT shiftreg_drive
    GENERIC (
      WIDTH   : positive := 32;           -- parallel data width
      CLK_DIV : positive := 2             -- SCLK freq is CLK / 2**(CLK_DIV+1)
    );
    PORT(
      CLK   : IN  std_logic;
      RESET : IN  std_logic;
      DATA  : IN  std_logic_vector(31 DOWNTO 0);
      START : IN  std_logic;
      SCLK  : OUT std_logic;
      DOUT  : OUT std_logic;
      SYNCn : OUT std_logic
    );
  END COMPONENT;
  ---------------------------------------------> Topmetal
  ---------------------------------------------< Chipscope
  COMPONENT cs_icon
    PORT (
      CONTROL0 : INOUT std_logic_vector(35 DOWNTO 0);
      CONTROL1 : INOUT std_logic_vector(35 DOWNTO 0)
    );
  END COMPONENT;
  COMPONENT cs_vio
    PORT (
      CONTROL   : INOUT std_logic_vector(35 DOWNTO 0);
      CLK       : IN std_logic;
      ASYNC_IN  : IN std_logic_vector(35 DOWNTO 0);
      ASYNC_OUT : OUT std_logic_vector(17 DOWNTO 0);
      SYNC_IN   : IN std_logic_vector(35 DOWNTO 0);
      SYNC_OUT  : OUT std_logic_vector(17 DOWNTO 0)
    );
  END COMPONENT;
  COMPONENT cs_ila
    PORT (
      CONTROL : INOUT std_logic_vector(35 DOWNTO 0);
      CLK     : IN    std_logic;
      TRIG0   : IN    std_logic_vector(39 DOWNTO 0)
    );
  END COMPONENT;
  ---------------------------------------------> Chipscope
  COMPONENT pulsegen
    GENERIC (
      period : positive
    );
    PORT (
      clk : IN  std_logic;
      I   : IN  std_logic;
      O   : OUT std_logic
    );
  END COMPONENT;

  -- Signals
  SIGNAL clk_sys         : std_logic;
  SIGNAL gclk4           : std_logic;
  SIGNAL gclk5           : std_logic;
  SIGNAL clk_100MHz      : std_logic;   -- 100 MHz clock
  SIGNAL clk_50MHz       : std_logic;
  SIGNAL clk_125MHz      : std_logic;
  SIGNAL clk_200MHz      : std_logic;
  SIGNAL clk_bufg_out    : std_logic;
  SIGNAL clk_locked      : std_logic;
  SIGNAL clk_rst         : std_logic;
  SIGNAL reset           : std_logic;
  ---------------------------------------------< Chipscope signals
  SIGNAL cs_control0     : std_logic_vector (35 DOWNTO 0);
  SIGNAL cs_control1     : std_logic_vector (35 DOWNTO 0);
  SIGNAL cs_trig0        : std_logic_vector (39 DOWNTO 0);
  SIGNAL cs_vio_syncin   : std_logic_vector (35 DOWNTO 0);
  SIGNAL cs_vio_syncout  : std_logic_vector (17 DOWNTO 0);
  SIGNAL cs_vio_asyncin  : std_logic_vector (35 DOWNTO 0);
  SIGNAL cs_vio_asyncout : std_logic_vector (17 DOWNTO 0);
  ---------------------------------------------> Chipscope signals
  ---------------------------------------------< gig_eth
  SIGNAL gig_eth_tx_tdata                  : std_logic_vector(7 DOWNTO 0);
  SIGNAL gig_eth_tx_tvalid                 : std_logic;
  SIGNAL gig_eth_tx_tready                 : std_logic;  
  SIGNAL gig_eth_rx_tdata                  : std_logic_vector(7 DOWNTO 0);
  SIGNAL gig_eth_rx_tvalid                 : std_logic;
  SIGNAL gig_eth_rx_tready                 : std_logic;
  SIGNAL gig_eth_tcp_use_fifo              : std_logic;
  SIGNAL gig_eth_tx_fifo_wrclk             : std_logic;
  SIGNAL gig_eth_tx_fifo_q                 : std_logic_vector(31 DOWNTO 0);
  SIGNAL gig_eth_tx_fifo_wren              : std_logic;
  SIGNAL gig_eth_tx_fifo_full              : std_logic;
  SIGNAL gig_eth_rx_fifo_rdclk             : std_logic;
  SIGNAL gig_eth_rx_fifo_q                 : std_logic_vector(31 DOWNTO 0);
  SIGNAL gig_eth_rx_fifo_rden              : std_logic;
  SIGNAL gig_eth_rx_fifo_empty             : std_logic;
  ---------------------------------------------> gig_eth
  ---------------------------------------------< UART/RS232
  SIGNAL uart_rx_data                      : std_logic_vector(7 DOWNTO 0);
  SIGNAL uart_rx_rdy                       : std_logic;
  SIGNAL control_clk                       : std_logic;
  SIGNAL control_fifo_q                    : std_logic_vector(35 DOWNTO 0);
  SIGNAL control_fifo_rdreq                : std_logic;
  SIGNAL control_fifo_empty                : std_logic;
  SIGNAL control_fifo_rdclk                : std_logic;
  SIGNAL cmd_fifo_q                        : std_logic_vector(35 DOWNTO 0);
  SIGNAL cmd_fifo_empty                    : std_logic;
  SIGNAL cmd_fifo_rdreq                    : std_logic;
  -- thirtytwo 16bit registers  
  SIGNAL config_reg                        : std_logic_vector(511 DOWNTO 0);
  -- 16bit pulse register
  SIGNAL pulse_reg                         : std_logic_vector(15 DOWNTO 0);
  -- eleven 16bit registers
  SIGNAL status_reg                        : std_logic_vector(175 DOWNTO 0) := (OTHERS => '0');
  SIGNAL control_mem_we                    : std_logic;
  SIGNAL control_mem_addr                  : std_logic_vector(31 DOWNTO 0);
  SIGNAL control_mem_din                   : std_logic_vector(31 DOWNTO 0);
  ---------------------------------------------> UART/RS232
  ---------------------------------------------< Topmetal
  SIGNAL dac_cnt                           : unsigned(5 DOWNTO 0);
  SIGNAL led_cnt                           : unsigned(25 DOWNTO 0);
  SIGNAL tm_btn                            : std_logic_vector(6 DOWNTO 0);
  SIGNAL tm_rst                            : std_logic;
  SIGNAL adc_refclk                        : std_logic;
  SIGNAL tm_trig_out                       : std_logic;
  SIGNAL tm_ex_rst_n                       : std_logic;
  ---------------------------------------------> Topmetal
  SIGNAL usr_data_output    : std_logic_vector (7 DOWNTO 0);

BEGIN
  UART_TX_PIN <= 'Z';
  ---------------------------------------------< Clock
  gclk4_ibufgds : IBUFGDS
    PORT MAP (
      O  => gclk4,
      I  => GCLK4_P,
      IB => GCLK4_N
    );
  gclk5_ibufgds : IBUFGDS
    PORT MAP (
      O  => gclk5,
      I  => GCLK5_P,
      IB => GCLK5_N
    );

  clk_sys <= clk_xtal;
  clockg_inst : clock_generator
    PORT MAP (
      -- Clock in ports
      CLKIN_IN        => clk_sys,       -- 100MHz In
      -- Clock out ports
      CLK0_OUT        => clk_100MHz,    -- 100MHz
      CLK2X_OUT       => clk_200MHz,    -- 200MHz
      CLKDV_OUT       => clk_50MHz,     -- 50MHz
      CLKFX_OUT       => clk_125MHz,    -- 125MHz
      CLKIN_IBUFG_OUT => clk_bufg_out,
      -- Status and control signals
      RST_IN          => clk_rst,
      LOCKED_OUT      => clk_locked
    );
  ---------------------------------------------> Clock
  globalresetter_inst : GlobalResetter
    PORT MAP (
      FORCE_RST_n => rst_sys_n,
      CLK         => clk_bufg_out,
      DCM_LOCKED  => clk_locked,
      CLK_RST     => clk_rst,
      GLOBAL_RST  => reset
  );
  ---------------------------------------------< Chipscope
  IncChipScope : IF includeChipscope GENERATE
    cs_icon_inst : cs_icon
      PORT MAP (
        CONTROL0 => cs_control0,
        CONTROL1 => cs_control1
      );
    cs_vio_inst : cs_vio
      PORT MAP (
        CONTROL   => cs_control1,
        CLK       => clk_125MHz,
        ASYNC_IN  => cs_vio_asyncin,
        ASYNC_OUT => cs_vio_asyncout,
        SYNC_IN   => cs_vio_syncin,
        SYNC_OUT  => cs_vio_syncout
      );
    cs_ila_inst : cs_ila
      PORT MAP (
        CONTROL => cs_control0,
        CLK     => NOT clk_125MHz,
        TRIG0   => cs_trig0
      );
    cs_trig0(39) <= clk_100MHz;
  END GENERATE IncChipScope;
  ---------------------------------------------> Chipscope
  ---------------------------------------------< gig_eth
  gig_eth_inst : gig_eth
    PORT MAP (
      -- asynchronous reset
      RESET                => reset,
      -- clocks
      CLK125               => clk_125MHz,
      REFCLK               => clk_200MHz,
      -- GMII Interface - EMAC0
      GMII_TXD_0           => GMII_TXD_0,
      GMII_TX_EN_0         => GMII_TX_EN_0,
      GMII_TX_ER_0         => GMII_TX_ER_0,
      GMII_TX_CLK_0        => GMII_TX_CLK_0,
      GMII_RXD_0           => GMII_RXD_0,
      GMII_RX_DV_0         => GMII_RX_DV_0,
      GMII_RX_ER_0         => GMII_RX_ER_0,
      GMII_RX_CLK_0        => GMII_RX_CLK_0,
      MII_TX_CLK_0         => MII_TX_CLK_0,
      GMII_COL_0           => GMII_COL_0,
      GMII_CRS_0           => GMII_CRS_0,
      PHY_RST_n            => PHY_RST_n,
      -- TCP
      TCP_CONNECTION_RESET => '0',
      TX_TDATA             => gig_eth_tx_tdata,
      TX_TVALID            => gig_eth_tx_tvalid,
      TX_TREADY            => gig_eth_tx_tready,
      RX_TDATA             => gig_eth_rx_tdata,
      RX_TVALID            => gig_eth_rx_tvalid,
      RX_TREADY            => gig_eth_rx_tready,
      -- FIFO
      TCP_USE_FIFO         => gig_eth_tcp_use_fifo,
      TX_FIFO_WRCLK        => gig_eth_tx_fifo_wrclk,
      TX_FIFO_Q            => gig_eth_tx_fifo_q,
      TX_FIFO_WREN         => gig_eth_tx_fifo_wren,
      TX_FIFO_FULL         => gig_eth_tx_fifo_full,
      RX_FIFO_RDCLK        => gig_eth_rx_fifo_rdclk,
      RX_FIFO_Q            => gig_eth_rx_fifo_q,
      RX_FIFO_RDEN         => gig_eth_rx_fifo_rden,
      RX_FIFO_EMPTY        => gig_eth_rx_fifo_empty
    );
  -- loopback
  --gig_eth_tx_tdata  <= gig_eth_rx_tdata;
  --gig_eth_tx_tvalid <= gig_eth_rx_tvalid;
  --gig_eth_rx_tready <= gig_eth_tx_tready;

  -- receive to cmd_fifo
  gig_eth_tcp_use_fifo         <= '1';
  gig_eth_rx_fifo_rdclk        <= control_clk;
  cmd_fifo_q(31 DOWNTO 0)      <= gig_eth_rx_fifo_q;
  cmd_fifo_empty               <= gig_eth_rx_fifo_empty;
  gig_eth_rx_fifo_rden         <= cmd_fifo_rdreq;
  -- send control_fifo data through gig_eth_tx_fifo
  gig_eth_tx_fifo_wrclk        <= clk_125MHz;
  -- connect FWFT fifo interface
  control_fifo_rdclk           <= gig_eth_tx_fifo_wrclk;
  gig_eth_tx_fifo_q            <= control_fifo_q(31 DOWNTO 0);
  gig_eth_tx_fifo_wren         <= NOT control_fifo_empty;
  control_fifo_rdreq           <= NOT gig_eth_tx_fifo_full;
  ---------------------------------------------> gig_eth
  ---------------------------------------------< UART/RS232
  uartio_inst : uartio
    GENERIC MAP (
      -- tick repetition frequency is (input freq) / (2**COUNTER_WIDTH / DIVISOR)
      COUNTER_WIDTH => 16,
      DIVISOR       => 1208*2
    )
    PORT MAP (
      CLK     => clk_50MHz,
      RESET   => reset,
      RX_DATA => uart_rx_data,
      RX_RDY  => uart_rx_rdy,
      TX_DATA => DIPSw8Bit,
      TX_EN   => '1',
      TX_RDY  => cs_trig0(2),
      -- serial lines
      RX_PIN  => UART_RX_PIN,
      TX_PIN  => UART_TX_PIN
    );
  byte2cmd_inst : byte2cmd
    PORT MAP (
      CLK            => clk_50MHz,
      RESET          => reset,
      -- byte in
      RX_DATA        => uart_rx_data,
      RX_RDY         => uart_rx_rdy,
      -- cmd out
      CMD_FIFO_Q     => OPEN,-- cmd_fifo_q,
      CMD_FIFO_EMPTY => OPEN,-- cmd_fifo_empty,
      CMD_FIFO_RDCLK => clk_100MHz,
      CMD_FIFO_RDREQ => '0'  -- cmd_fifo_rdreq
    );
  control_interface_inst : control_interface
    PORT MAP (
      RESET => reset,
      CLK   => control_clk,
      -- From FPGA to PC
      FIFO_Q          => control_fifo_q,
      FIFO_EMPTY      => control_fifo_empty,
      FIFO_RDREQ      => control_fifo_rdreq,
      FIFO_RDCLK      => control_fifo_rdclk,
      -- From PC to FPGA, FWFT
      CMD_FIFO_Q      => cmd_fifo_q,
      CMD_FIFO_EMPTY  => cmd_fifo_empty,
      CMD_FIFO_RDREQ  => cmd_fifo_rdreq,
      -- Digital I/O
      CONFIG_REG      => config_reg,
      PULSE_REG       => pulse_reg,
      STATUS_REG      => status_reg,
      -- Memory interface
      MEM_WE          => OPEN,
      MEM_ADDR        => OPEN,
      MEM_DIN         => OPEN,
      MEM_DOUT        => (OTHERS => '0'),
      -- Data FIFO interface, FWFT
      DATA_FIFO_Q     => (OTHERS => '0'),
      DATA_FIFO_EMPTY => '0',
      DATA_FIFO_RDREQ => OPEN,
      DATA_FIFO_RDCLK => OPEN
    );
  control_clk           <= clk_125MHz;
  cs_trig0(18 DOWNTO 3) <= pulse_reg;
  cs_vio_syncin         <= config_reg(35 DOWNTO 0);
  cs_vio_asyncin        <= config_reg(71 DOWNTO 36);
  ---------------------------------------------> UART/RS232
  ---------------------------------------------< Topmetal
  PROCESS (clk_50MHz, reset)
  BEGIN
    IF reset = '1' then
      dac_cnt <= (OTHERS => '0');
    ELSIF rising_edge(clk_50MHz) then
      dac_cnt <= dac_cnt + 1;
    END IF;
  END PROCESS;

  -- on the 4-SMA driver board
  --dac_inter8568_inst : dac_inter8568
  --  PORT MAP(
  --    RESET => NOT config_reg(16*1),
  --    CLK   => dac_cnt(2),
  --    DATAA => config_reg(15 DOWNTO 0),
  --    DATAC => (OTHERS => '0'),
  --    DATAE => (OTHERS => '0'),
  --    DATAG => (OTHERS => '0'),
  --    DIN   => JD(5),
  --    SCLK  => JD(1),
  --    SYN   => JD(4)
  --  );

  -- on the `bottom' board
  dac8568_inst : shiftreg_drive
    GENERIC MAP (
      WIDTH   => 32,           -- parallel data width
      CLK_DIV => 6             -- SCLK freq is CLK / 2**(CLK_DIV+1)
    )
    PORT MAP (
      CLK   => control_clk,
      RESET => tm_rst,
      DATA  => config_reg(16*9-1 DOWNTO 16*7),
      START => pulse_reg(1),
      SCLK  => JD(1),
      DOUT  => JD(0),
      SYNCn => JD(2)
    );

  topmetal_simple_inst : topmetal_simple PORT MAP(
    RST                  => tm_rst,
    CLK                  => adc_refclk,
    SWG                  => config_reg(16*3-1-8 DOWNTO 16*2),
    BTN                  => tm_btn,
    MARKER_IN            => JB(2),
    MARKER_OUT           => OPEN,
    STOP_CONTROL         => config_reg(16*4-1),
    STOP_ADDRESS         => config_reg(16*4-1-6 DOWNTO 16*3),
    TRIGGER_CONTROL      => config_reg(16*5-2),
    TRIGGER_RATE_CONTROL => config_reg(16*5-1),
    TRIGGER_RATE         => config_reg(16*5-1-12 DOWNTO 16*4),
    TRIGGER_DELAY        => config_reg(16*7-1 DOWNTO 16*6),
    TRIGGER_OUT          => tm_trig_out,
    TM_CLK               => JB(5),
    TM_RST               => JC(0),
    TM_START             => JB(4),
    TM_SPEAK             => JB(0),
    EX_RST_n             => tm_ex_rst_n
  );
  tm_btn(6) <= config_reg(16*3-1-7);
  tm_btn(1) <= config_reg(16*3-1-6);
  tm_btn(0) <= config_reg(16*3-1-5);
  tm_rst    <= reset OR config_reg(16*1+8);
  JC(3)     <= (tm_trig_out AND (NOT config_reg(16*3-2))) OR pulse_reg(0) OR BTN(0);  -- trigger to digitizer
  JB(1)     <= tm_ex_rst_n OR config_reg(16*3-1);  -- ex_rst
  WITH config_reg(16*5+1 DOWNTO 16*5) SELECT
    adc_refclk <= JD(6) WHEN "01",      -- diff in, converted to single-ended
    JB(3)               WHEN "10",      -- pins on JB
    JB(7)               WHEN "11",      -- pins on JB
    clk_50MHz           WHEN OTHERS;
  
  PROCESS (adc_refclk, reset)
  BEGIN
    IF reset = '1' then
      led_cnt <= (OTHERS => '0');
    ELSIF rising_edge(adc_refclk) then
      led_cnt <= led_cnt + 1;
    END IF;
  END PROCESS;
  usr_data_output(3 DOWNTO 0) <= std_logic_vector(led_cnt(25 DOWNTO 22));
  ---------------------------------------------> Topmetal

  led_obufs : FOR i IN 0 TO 7 GENERATE
    led_obuf : OBUF
      PORT MAP (
        I => usr_data_output(i),
        O => LED8Bit(i)
      );
  END GENERATE led_obufs;

  pulsegen_inst : pulsegen
    GENERIC MAP (
      period => 1500
    )
    PORT MAP (
      clk => clk_125MHz,
      I   => BTN(0),
      O   => cs_trig0(1)
    );
  
END Behavioral;
