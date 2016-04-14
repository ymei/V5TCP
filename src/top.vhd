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
-- Tool versions: ISE 14.7
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

USE work.common_pkg.ALL;

ENTITY top IS
  GENERIC (
    includeChipscope : boolean := false
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
  ATTRIBUTE clock_dedicated_route          : string;
  ATTRIBUTE clock_dedicated_route OF JA    : SIGNAL IS "false";
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
      MAC_ADDR             : IN std_logic_vector(47 DOWNTO 0);
      IPv4_ADDR            : IN std_logic_vector(31 DOWNTO 0);
      IPv6_ADDR            : IN std_logic_vector(127 DOWNTO 0);
      SUBNET_MASK          : IN std_logic_vector(31 DOWNTO 0);
      GATEWAY_IP_ADDR      : IN std_logic_vector(31 DOWNTO 0);
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
  COMPONENT topmetal_iiminus_analog
    GENERIC (
      ROWS          : positive := 72;   -- number of ROWS in the array
      COLS          : positive := 72;   -- number of COLS in the ARRAY
      CLK_DIV_WIDTH : positive := 16;
      CONFIG_WIDTH  : positive := 16
    );
    PORT (
      CLK           : IN  std_logic;  -- clock, TM_CLK_S is derived from this one
      RESET         : IN  std_logic;    -- reset
      -- data input for writing to in-chip SRAM
      MEM_CLK       : IN  std_logic;    -- connect to control_interface
      MEM_WE        : IN  std_logic;
      MEM_ADDR      : IN  std_logic_vector(31 DOWNTO 0);
      MEM_DIN       : IN  std_logic_vector(31 DOWNTO 0);
      SRAM_WR_START : IN  std_logic;  -- 1 MEM_CLK wide pulse to initiate in-chip SRAM write
      -- configuration
      CLK_DIV       : IN  std_logic_vector(3 DOWNTO 0);  -- log2(CLK_DIV_WIDTH), CLK/(2**CLK_DIV)
      STOP_ADDR     : IN  std_logic_vector(CONFIG_WIDTH-1 DOWNTO 0);  --MSB enables
      TRIGGER_RATE  : IN  std_logic_vector(CONFIG_WIDTH-1 DOWNTO 0);  --trigger every () frames
      TRIGGER_DELAY : IN  std_logic_vector(CONFIG_WIDTH-1 DOWNTO 0);
      -- input
      MARKER_A      : IN  std_logic;
      -- output
      TRIGGER_OUT   : OUT std_logic;
      --
      SRAM_D        : OUT std_logic_vector(4 DOWNTO 0);
      SRAM_WE       : OUT std_logic;
      TM_RST        : OUT std_logic;    -- digital reset
      TM_CLK_S      : OUT std_logic;
      TM_RST_S      : OUT std_logic;
      TM_START_S    : OUT std_logic;
      TM_SPEAK_S    : OUT std_logic
    );
  END COMPONENT;
  COMPONENT topmetal_iiminus_digital
    GENERIC (
      CLK_DIV_WIDTH  : positive := 16;
      DATA_OUT_WIDTH : positive := 32
    );
    PORT (
      CLK         : IN  std_logic;      -- clock to be derived from
      RESET       : IN  std_logic;      -- reset    
      CLK_DIV     : IN  std_logic_vector(3 DOWNTO 0);  -- log2(CLK_DIV_WIDTH), CLK/(2**CLK_DIV)
      --
      CLK_D       : OUT std_logic;      -- clock sent to chip's digital part
      CLK_FB      : IN  std_logic;      -- clock fed back from the chip
      READY_P     : IN  std_logic;
      READY_N     : IN  std_logic;
      ADDR_P      : IN  std_logic_vector(6 DOWNTO 0);
      ADDR_N      : IN  std_logic_vector(6 DOWNTO 0);
      TIME_P      : IN  std_logic_vector(9 DOWNTO 0);
      TIME_N      : IN  std_logic_vector(9 DOWNTO 0);
      MARKERD_P   : IN  std_logic;
      MARKERD_N   : IN  std_logic;
      --
      DATA_VALID  : OUT std_logic;
      DATA_CLK    : OUT std_logic;
      DATA_OUT    : OUT std_logic_vector(DATA_OUT_WIDTH-1 DOWNTO 0)
    );
  END COMPONENT;
  COMPONENT fifo2shiftreg
    GENERIC (
      WIDTH   : positive := 32;         -- parallel data width
      CLK_DIV : positive := 2           -- SCLK freq is CLK / 2**(CLK_DIV)
    );
    PORT (
      CLK      : IN  std_logic;         -- clock
      RESET    : IN  std_logic;         -- reset
      -- input data interface
      WR_CLK   : IN  std_logic;         -- FIFO write clock
      DIN      : IN  std_logic_vector(15 DOWNTO 0);
      WR_EN    : IN  std_logic;
      WR_PULSE : IN  std_logic;  -- one pulse writes one word, regardless of pulse duration
      FULL     : OUT std_logic;
      -- output
      SCLK     : OUT std_logic;
      DOUT     : OUT std_logic;
      SYNCn    : OUT std_logic
    );
  END COMPONENT;
  ---------------------------------------------> Topmetal
  ---------------------------------------------< Data readback
  COMPONENT fifo36l
    PORT (
      RST    : IN  std_logic;
      WR_CLK : IN  std_logic;
      RD_CLK : IN  std_logic;
      DIN    : IN  std_logic_vector(35 DOWNTO 0);
      WR_EN  : IN  std_logic;
      RD_EN  : IN  std_logic;
      DOUT   : OUT std_logic_vector(35 DOWNTO 0);
      FULL   : OUT std_logic;
      EMPTY  : OUT std_logic
    );
  END COMPONENT;
  COMPONENT fifo_rdwidth_reducer
    GENERIC (
      RDWIDTH    : positive := 32;
      RDRATIO    : positive := 3;
      SHIFTORDER : positive := 1        -- 1: MSB first, 0: LSB first
    );
    PORT (
      RESET : IN  std_logic;
      CLK   : IN  std_logic;
      -- input data interface
      DIN   : IN  std_logic_vector(RDWIDTH*RDRATIO-1 DOWNTO 0);
      VALID : IN  std_logic;
      RDREQ : OUT std_logic;
      -- output
      DOUT  : OUT std_logic_vector(RDWIDTH-1 DOWNTO 0);
      EMPTY : OUT std_logic;
      RD_EN : IN  std_logic
    );
  END COMPONENT;
  ---------------------------------------------> Data readback
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
  SIGNAL gig_eth_mac_addr                  : std_logic_vector(47 DOWNTO 0);
  SIGNAL gig_eth_ipv4_addr                 : std_logic_vector(31 DOWNTO 0);
  SIGNAL gig_eth_subnet_mask               : std_logic_vector(31 DOWNTO 0);
  SIGNAL gig_eth_gateway_ip_addr           : std_logic_vector(31 DOWNTO 0);
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
  SIGNAL control_mem_dout                  : std_logic_vector(31 DOWNTO 0);
  --
  SIGNAL control_data_fifo_q               : std_logic_vector(31 DOWNTO 0);
  SIGNAL control_data_fifo_rdclk           : std_logic;
  SIGNAL control_data_fifo_rdreq           : std_logic;
  SIGNAL control_data_fifo_empty           : std_logic;
  ---------------------------------------------> UART/RS232
  ---------------------------------------------< Topmetal
  SIGNAL led_cnt                           : unsigned(25 DOWNTO 0);
  SIGNAL clki_cnt                          : unsigned(3 DOWNTO 0);
  SIGNAL tm_rst                            : std_logic;
  SIGNAL adc_refclk                        : std_logic;
  SIGNAL tm_trig_out                       : std_logic;
  SIGNAL tm_sram_d                         : std_logic_vector(4 DOWNTO 0);
  SIGNAL tm_sram_we                        : std_logic;
  SIGNAL tm_addr_p                         : std_logic_vector(6 DOWNTO 0);
  SIGNAL tm_addr_n                         : std_logic_vector(6 DOWNTO 0);
  SIGNAL tm_time_p                         : std_logic_vector(9 DOWNTO 0);
  SIGNAL tm_time_n                         : std_logic_vector(9 DOWNTO 0);
  SIGNAL tm_data_valid                     : std_logic;
  SIGNAL tm_data_clk                       : std_logic;
  SIGNAL tm_data_d                         : std_logic_vector(31 DOWNTO 0);
  SIGNAL tm_markera                        : std_logic;
  SIGNAL tm_rst_s                          : std_logic;
  SIGNAL tm_start_s                        : std_logic;
  SIGNAL tm_clk_s                          : std_logic;
  SIGNAL dac_sclk                          : std_logic;
  SIGNAL dac_d                             : std_logic;
  SIGNAL dac_syncn                         : std_logic;
  TYPE INT_ARRAY IS ARRAY (integer RANGE <>) OF integer;
  SIGNAL tm_ctrl_sig                       : std_logic_vector(0 TO 5);
  CONSTANT tm_ctrl_idx                     : INT_ARRAY(0 TO 5) := (0,1,2,4,5,6);
  ---------------------------------------------> Topmetal
  ---------------------------------------------< Data readback
  SIGNAL fifo36_din                        : std_logic_vector(35 DOWNTO 0);
  SIGNAL fifo36_wrclk                      : std_logic;
  SIGNAL fifo36_wren                       : std_logic;
  SIGNAL fifo36_full                       : std_logic;
  SIGNAL fifo36_dout                       : std_logic_vector(35 DOWNTO 0);
  SIGNAL fifo36_rden                       : std_logic;
  SIGNAL fifo36_empty                      : std_logic;
  SIGNAL fifo36_valid                      : std_logic;
  SIGNAL fifo36_trig                       : std_logic;
  ---------------------------------------------> Data readback
  SIGNAL usr_data_output                   : std_logic_vector (7 DOWNTO 0);

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
  gig_eth_mac_addr(gig_eth_mac_addr'length-1 DOWNTO 4)   <= x"00183e010f0";
  gig_eth_mac_addr(3 DOWNTO 0)                           <= DIPSw8Bit(3 DOWNTO 0);
  gig_eth_ipv4_addr(gig_eth_ipv4_addr'length-1 DOWNTO 4) <= x"c0a8020";
  gig_eth_ipv4_addr(3 DOWNTO 0)                          <= DIPSw8Bit(3 DOWNTO 0);
  gig_eth_subnet_mask                                    <= x"ffffff00";
  gig_eth_gateway_ip_addr                                <= x"c0a80201";
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
      MAC_ADDR             => gig_eth_mac_addr,
      IPv4_ADDR            => gig_eth_ipv4_addr,
      IPv6_ADDR            => (OTHERS => '0'),
      SUBNET_MASK          => gig_eth_subnet_mask,
      GATEWAY_IP_ADDR      => gig_eth_gateway_ip_addr,
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
      MEM_WE          => control_mem_we,
      MEM_ADDR        => control_mem_addr,
      MEM_DIN         => control_mem_din,
      MEM_DOUT        => control_mem_dout,
      -- Data FIFO interface, FWFT
      DATA_FIFO_Q     => control_data_fifo_q,
      DATA_FIFO_EMPTY => control_data_fifo_empty,
      DATA_FIFO_RDREQ => control_data_fifo_rdreq,
      DATA_FIFO_RDCLK => control_data_fifo_rdclk
    );
  control_clk           <= clk_125MHz;
  cs_trig0(18 DOWNTO 3) <= pulse_reg;
  cs_vio_syncin         <= config_reg(35 DOWNTO 0);
  cs_vio_asyncin        <= config_reg(71 DOWNTO 36);
  ---------------------------------------------> UART/RS232
  ---------------------------------------------< Topmetal
  -- on the `bottom' board
  dac8568_inst : fifo2shiftreg
    GENERIC MAP (
      WIDTH   => 32,                    -- parallel data width
      CLK_DIV => 8                      -- SCLK freq is CLK / 2**(CLK_DIV+1)
    )
    PORT MAP (
      CLK      => control_clk,          -- clock
      RESET    => tm_rst,               -- reset
      -- input data interface
      WR_CLK   => control_clk,          -- FIFO write clock
      DIN      => config_reg(16*7+15 DOWNTO 16*7),
      WR_EN    => '0',
      WR_PULSE => pulse_reg(1),  -- one pulse writes one word, regardless of pulse duration
      FULL     => OPEN,
      -- output
      SCLK     => dac_sclk, 
      DOUT     => dac_d, 
      SYNCn    => dac_syncn
    );
  JD(5) <= dac_sclk;
  JD(4) <= dac_d;
  JD(6) <= dac_syncn;
  -- topmetal internal DAC
  topmetal_dac_inst : fifo2shiftreg
    GENERIC MAP (
      WIDTH   => 32,                    -- parallel data width
      CLK_DIV => 10                     -- SCLK freq is CLK / 2**(CLK_DIV+1)
    )
    PORT MAP (
      CLK      => control_clk,          -- clock
      RESET    => tm_rst,               -- reset
      -- input data interface
      WR_CLK   => control_clk,          -- FIFO write clock
      DIN      => config_reg(16*8+15 DOWNTO 16*8),
      WR_EN    => '0',
      WR_PULSE => pulse_reg(2),  -- one pulse writes one word, regardless of pulse duration
      FULL     => OPEN,
      -- output
      SCLK     => JD(2),
      DOUT     => JD(3),
      SYNCn    => OPEN
    );
  JD(7)<='0';  --DAC_VREF_MODE, 0 means using internal bandgap reference
  --
  JC(4)<='0';  --ADDR_GRST
  JC(5)<=tm_sram_d(4); JC(1)<=tm_sram_d(3); JC(7)<=tm_sram_d(2); JC(2)<=tm_sram_d(1); JC(6)<=tm_sram_d(0);
  JC(0)<=tm_sram_we;
  topmetal_iiminus_analog_inst : topmetal_iiminus_analog
    GENERIC MAP (
      ROWS          => 72,              -- number of ROWS in the array
      COLS          => 72,              -- number of COLS in the ARRAY
      CLK_DIV_WIDTH => 16,
      CONFIG_WIDTH  => 16
    )
    PORT MAP (
      CLK           => adc_refclk,  -- clock, TM_CLK_S is derived from this one
      RESET         => tm_rst,      -- reset
      -- data input for writing to in-chip SRAM
      MEM_CLK       => control_clk,
      MEM_WE        => control_mem_we,
      MEM_ADDR      => control_mem_addr,
      MEM_DIN       => control_mem_din,
      SRAM_WR_START => pulse_reg(3),  -- 1 MEM_CLK wide pulse to initiate in-chip SRAM write
      -- configuration
      CLK_DIV       => config_reg(16*2+3 DOWNTO 16*2),  -- log2(CLK_DIV_WIDTH), CLK/(2**CLK_DIV)
      STOP_ADDR     => config_reg(16*4-1 DOWNTO 16*3),  -- MSB enables
      TRIGGER_RATE  => config_reg(16*6-1 DOWNTO 16*5),  -- trigger every () frames
      TRIGGER_DELAY => config_reg(16*5-1 DOWNTO 16*4),
      -- input
      MARKER_A      => JB(6),
      -- output
      TRIGGER_OUT   => tm_trig_out,
      --
      SRAM_D        => tm_sram_d,
      SRAM_WE       => tm_sram_we,
      TM_RST        => JC(3),           -- digital reset
      TM_CLK_S      => tm_clk_s,
      TM_RST_S      => tm_rst_s,
      TM_START_S    => tm_start_s,
      TM_SPEAK_S    => JB(4)
    );
  JB(1)  <= tm_clk_s;
  JB(5)  <= tm_rst_s;
  JB(0)  <= tm_start_s;
  tm_rst <= reset OR config_reg(16*1+8);
  JB(3)  <= (tm_trig_out AND (NOT config_reg(16*3-2))) OR pulse_reg(0) OR BTN(0);  -- trigger to digitizer
  -- JA(3)  <= (tm_trig_out AND (NOT config_reg(16*3-2))) OR pulse_reg(0) OR BTN(0);  -- replica
  JB(7)  <= config_reg(16*3-1);         -- ex_rst
  WITH config_reg(16*6+1 DOWNTO 16*6) SELECT
    adc_refclk <= JB(2) WHEN "01",      -- optocoupler isolated
    JA(7)               WHEN "10",      -- pins on JA
    clk_100MHz          WHEN "11",
    clki_cnt(3)         WHEN OTHERS;    -- 3.125MHz
  PROCESS (clk_50MHz, reset)            -- producing 50MHz/2**4 = 3.125MHz clock
  BEGIN
    IF reset = '1' then
      clki_cnt <= (OTHERS => '0');
    ELSIF rising_edge(clk_50MHz) then
      clki_cnt <= clki_cnt + 1;
    END IF;
  END PROCESS;
  -- signals over VHDCI J1
  tm_ctrl_sig <= (0=>dac_sclk, 1=>dac_d, 2=>dac_syncn, 3=>tm_rst_s, 4=>tm_start_s, 5=>tm_clk_s);
  tm_ctrl_obufds_insts : FOR i IN 0 TO 5 GENERATE
    tm_ctrl_obufds_inst : OBUFDS
      GENERIC MAP (
        IOSTANDARD => "DEFAULT"
      )
      PORT MAP (
        O  => VHDCI1P(tm_ctrl_idx(i)),  -- Diff_p output (connect directly to top-level port)
        OB => VHDCI1N(tm_ctrl_idx(i)),  -- Diff_n output (connect directly to top-level port)
        I  => tm_ctrl_sig(i)            -- Buffer input
      );
  END GENERATE;
  tm_ctrl_ibufds_inst : IBUFDS
    GENERIC MAP (
      DIFF_TERM  => true,
      IOSTANDARD => "DEFAULT"
    )
    PORT MAP (
      O  => tm_markera,
      I  => VHDCI1P(3),
      IB => VHDCI1N(3)
    );
  JA(4) <= tm_markera;

  -- digital
  topmetal_iiminus_digital_inst : topmetal_iiminus_digital
    PORT MAP (
      CLK         => clk_125MHz,        -- clock to be derived from
      RESET       => reset,             -- reset    
      CLK_DIV     => config_reg(16*8+3 DOWNTO 16*8),  -- log2(CLK_DIV_WIDTH), CLK/(2**CLK_DIV)
      --
      CLK_D       => JD(0),             -- clock sent to chip's digital part
      CLK_FB      => JA(3),             -- clock fed back from the chip
      READY_P     => VHDCI2P(1),
      READY_N     => VHDCI2N(1),
      ADDR_P      => tm_addr_p,
      ADDR_N      => tm_addr_n,
      TIME_P      => tm_time_p,
      TIME_N      => tm_time_n,
      MARKERD_P   => VHDCI2P(0),
      MARKERD_N   => VHDCI2N(0),
      --
      DATA_VALID  => tm_data_valid,
      DATA_CLK    => tm_data_clk,
      DATA_OUT    => tm_data_d
    );
  tm_addr_p <= (VHDCI2P(2), VHDCI2P(3), VHDCI2P(4), VHDCI2P(5), VHDCI2P(6), VHDCI2P(7),
                VHDCI2P(8));
  tm_addr_n <= (VHDCI2N(2), VHDCI2N(3), VHDCI2N(4), VHDCI2N(5), VHDCI2N(6), VHDCI2N(7),
                VHDCI2N(8));
  tm_time_p <= (VHDCI2P(10), VHDCI2P(11), VHDCI2P(12), VHDCI2P(13), VHDCI2P(14),
                VHDCI2P(15), VHDCI2P(16), VHDCI2P(17), VHDCI2P(18), VHDCI2P(19));
  tm_time_n <= (VHDCI2N(10), VHDCI2N(11), VHDCI2N(12), VHDCI2N(13), VHDCI2N(14),
                VHDCI2N(15), VHDCI2N(16), VHDCI2N(17), VHDCI2N(18), VHDCI2N(19));

  -- marker_d for debug
  JA(0) <= fifo36_din(18);
  JA(1) <= fifo36_wrclk;

  -- Data readback
  fifo36_inst : fifo36l
    PORT MAP (
      RST    => fifo36_trig,
      WR_CLK => fifo36_wrclk,
      RD_CLK => control_data_fifo_rdclk,
      DIN    => fifo36_din,
      WR_EN  => fifo36_wren,
      RD_EN  => fifo36_rden,
      DOUT   => fifo36_dout,
      FULL   => status_reg(0),
      EMPTY  => fifo36_empty
    );
  fifo36_wren  <= tm_data_valid;
  fifo36_din   <= "0000" & tm_data_d;
  fifo36_wrclk <= tm_data_clk;
  fifo36_trig  <= reset OR pulse_reg(5);
  fifo36_valid <= NOT fifo36_empty;
  --
  control_data_fifo_q     <= fifo36_dout(31 DOWNTO 0);
  control_data_fifo_empty <= fifo36_empty;
  fifo36_rden             <= control_data_fifo_rdreq;

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
