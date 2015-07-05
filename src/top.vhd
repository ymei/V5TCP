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
    -- SDRAM
    DDR2_DQ       : INOUT std_logic_vector(63 DOWNTO 0);
    DDR2_DQS      : INOUT std_logic_vector(7 DOWNTO 0);
    DDR2_DQS_N    : INOUT std_logic_vector(7 DOWNTO 0);
    DDR2_A        : OUT   std_logic_vector(12 DOWNTO 0);
    DDR2_BA       : OUT   std_logic_vector(1 DOWNTO 0);
    DDR2_RAS_N    : OUT   std_logic;
    DDR2_CAS_N    : OUT   std_logic;
    DDR2_WE_N     : OUT   std_logic;
    DDR2_CS_N     : OUT   std_logic_vector(0 DOWNTO 0);
    DDR2_ODT      : OUT   std_logic_vector(0 DOWNTO 0);
    DDR2_CKE      : OUT   std_logic_vector(0 DOWNTO 0);
    DDR2_DM       : OUT   std_logic_vector(7 DOWNTO 0);
    DDR2_CK       : OUT   std_logic_vector(1 DOWNTO 0);
    DDR2_CK_N     : OUT   std_logic_vector(1 DOWNTO 0);
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
  ---------------------------------------------< SDRAM
  COMPONENT sdram_ddr2
    GENERIC (
      INDATA_WIDTH   : positive := 256;
      OUTDATA_WIDTH  : positive := 32;
      APP_ADDR_WIDTH : positive := 31;
      APP_DATA_WIDTH : positive := 128;
      APP_MASK_WIDTH : positive := 16;
      APP_ADDR_BURST : positive := 8
    );
    PORT (
      CLK           : IN    std_logic;  -- system clock, directly out of ibufg
      CLK200        : IN    std_logic;  -- 200MHz reference clock for iodelay
      RESET         : IN    std_logic;
      --
      DDR2_DQ       : INOUT std_logic_vector(63 DOWNTO 0);
      DDR2_DQS      : INOUT std_logic_vector(7 DOWNTO 0);
      DDR2_DQS_N    : INOUT std_logic_vector(7 DOWNTO 0);
      DDR2_A        : OUT   std_logic_vector(12 DOWNTO 0);
      DDR2_BA       : OUT   std_logic_vector(1 DOWNTO 0);
      DDR2_RAS_N    : OUT   std_logic;
      DDR2_CAS_N    : OUT   std_logic;
      DDR2_WE_N     : OUT   std_logic;
      DDR2_CS_N     : OUT   std_logic_vector(0 DOWNTO 0);
      DDR2_ODT      : OUT   std_logic_vector(0 DOWNTO 0);
      DDR2_CKE      : OUT   std_logic_vector(0 DOWNTO 0);
      DDR2_DM       : OUT   std_logic_vector(7 DOWNTO 0);
      DDR2_CK       : OUT   std_logic_vector(1 DOWNTO 0);
      DDR2_CK_N     : OUT   std_logic_vector(1 DOWNTO 0);
      -- Status Outputs
      PHY_INIT_DONE : OUT   std_logic
    );
  END COMPONENT;
  ---------------------------------------------> SDRAM
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
  ---------------------------------------------< ADC
  COMPONENT ads5282_interface
    GENERIC (
      ADC_NCH : positive := 8
    );
    PORT (
      RESET   : IN  std_logic;
      CLK     : IN  std_logic;
      --
      CONFIG  : IN  std_logic_vector(31 DOWNTO 0);
      CONFPS  : IN  std_logic;
      CONFULL : OUT std_logic;
      --
      ADCLKp  : IN  std_logic;          -- LVDS frame clock (1X)
      ADCLKn  : IN  std_logic;
      LCLKp   : IN  std_logic;          -- LVDS bit clock (6X)
      LCLKn   : IN  std_logic;
      DATAp   : IN  std_logic_vector(ADC_NCH-1 DOWNTO 0);
      DATAn   : IN  std_logic_vector(ADC_NCH-1 DOWNTO 0);
      --
      ADCLK   : OUT std_logic;
      DATA    : OUT ADS5282DATA(ADC_NCH-1 DOWNTO 0);
      --
      SCLK    : OUT std_logic;
      SDATA   : OUT std_logic;
      CSn     : OUT std_logic
    );
  END COMPONENT;
  COMPONENT fifo96
    PORT (
      RST    : IN  std_logic;
      WR_CLK : IN  std_logic;
      RD_CLK : IN  std_logic;
      DIN    : IN  std_logic_vector(95 DOWNTO 0);
      WR_EN  : IN  std_logic;
      RD_EN  : IN  std_logic;
      DOUT   : OUT std_logic_vector(95 DOWNTO 0);
      FULL   : OUT std_logic;
      EMPTY  : OUT std_logic
    );
  END COMPONENT;
  COMPONENT fifo_rdwidth_reducer
    GENERIC (
      RDWIDTH : positive := 32;
      RDRATIO : positive := 3
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
  ---------------------------------------------> ADC
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
  ---------------------------------------------< SDRAM
  SIGNAL sdram_phy_init_done               : std_logic;
  ---------------------------------------------> SDRAM
  ---------------------------------------------< Topmetal
  SIGNAL dac_sclk                          : std_logic;
  SIGNAL dac_dout                          : std_logic;
  SIGNAL dac_sync_n                        : std_logic;
  SIGNAL led_cnt                           : unsigned(25 DOWNTO 0);
  SIGNAL tm_btn                            : std_logic_vector(6 DOWNTO 0);
  SIGNAL tm_rst                            : std_logic;
  SIGNAL adc_refclk                        : std_logic;
  SIGNAL tm_trig_out                       : std_logic;
  SIGNAL tm_ex_rst_n                       : std_logic;
  ---------------------------------------------> Topmetal
  ---------------------------------------------< ADC
  SIGNAL ads5282_0_data_p    : std_logic_vector(7 DOWNTO 0);
  SIGNAL ads5282_0_data_n    : std_logic_vector(7 DOWNTO 0);
  SIGNAL ads5282_0_adclk     : std_logic;
  SIGNAL ads5282_0_data      : ADS5282DATA(7 DOWNTO 0);
  SIGNAL ads5282_0_config    : std_logic_vector(31 DOWNTO 0);
  SIGNAL ads5282_0_confps    : std_logic;
  SIGNAL ads5282_1_data_p    : std_logic_vector(7 DOWNTO 0);
  SIGNAL ads5282_1_data_n    : std_logic_vector(7 DOWNTO 0);
  SIGNAL ads5282_1_adclk     : std_logic;
  SIGNAL ads5282_1_data      : ADS5282DATA(7 DOWNTO 0);
  SIGNAL ads5282_1_config    : std_logic_vector(31 DOWNTO 0);
  SIGNAL ads5282_1_confps    : std_logic;
  SIGNAL ads5282_2_data_p    : std_logic_vector(3 DOWNTO 0);
  SIGNAL ads5282_2_data_n    : std_logic_vector(3 DOWNTO 0);
  SIGNAL ads5282_2_adclk     : std_logic;
  SIGNAL ads5282_2_data      : ADS5282DATA(3 DOWNTO 0);
  SIGNAL ads5282_2_config    : std_logic_vector(31 DOWNTO 0);
  SIGNAL ads5282_2_confps    : std_logic;
  SIGNAL adc_data_fifo_q     : std_logic_vector(31 DOWNTO 0);
  SIGNAL adc_data_fifo_rdclk : std_logic;
  SIGNAL adc_data_fifo_rdreq : std_logic;
  SIGNAL adc_data_fifo_empty : std_logic;
  SIGNAL fifo96_din          : std_logic_vector(95 DOWNTO 0);
  SIGNAL fifo96_wrclk        : std_logic;
  SIGNAL fifo96_wren         : std_logic;
  SIGNAL fifo96_full         : std_logic;  
  SIGNAL fifo96_dout         : std_logic_vector(95 DOWNTO 0);
  SIGNAL fifo96_rden         : std_logic;
  SIGNAL fifo96_empty        : std_logic;
  SIGNAL fifo96_valid        : std_logic;
  SIGNAL fifo96_trig         : std_logic;
  ---------------------------------------------> ADC
  SIGNAL usr_data_output  : std_logic_vector (7 DOWNTO 0);

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
      DATA_FIFO_Q     => adc_data_fifo_q,
      DATA_FIFO_EMPTY => adc_data_fifo_empty,
      DATA_FIFO_RDREQ => adc_data_fifo_rdreq,
      DATA_FIFO_RDCLK => adc_data_fifo_rdclk
    );
  control_clk           <= clk_125MHz;
  cs_trig0(18 DOWNTO 3) <= pulse_reg;
  cs_vio_syncin         <= config_reg(35 DOWNTO 0);
  cs_vio_asyncin        <= config_reg(71 DOWNTO 36);
  ---------------------------------------------> UART/RS232
  ---------------------------------------------< SDRAM
  sdram_ddr2_inst : sdram_ddr2
    PORT MAP (
      CLK           => control_clk,
      CLK200        => clk_200MHz,
      RESET         => reset,
      --
      DDR2_DQ       => DDR2_DQ,
      DDR2_DQS      => DDR2_DQS,
      DDR2_DQS_N    => DDR2_DQS_N,
      DDR2_A        => DDR2_A,
      DDR2_BA       => DDR2_BA,
      DDR2_RAS_N    => DDR2_RAS_N,
      DDR2_CAS_N    => DDR2_CAS_N,
      DDR2_WE_N     => DDR2_WE_N,
      DDR2_CS_N     => DDR2_CS_N,
      DDR2_ODT      => DDR2_ODT,
      DDR2_CKE      => DDR2_CKE,
      DDR2_DM       => DDR2_DM,
      DDR2_CK       => DDR2_CK,
      DDR2_CK_N     => DDR2_CK_N,
      -- Status Outputs
      PHY_INIT_DONE => sdram_phy_init_done
    );
  ---------------------------------------------> SDRAM
  ---------------------------------------------< Topmetal
  dac8568_inst : fifo2shiftreg
    GENERIC MAP (
      WIDTH   => 32,                    -- parallel data width
      CLK_DIV => 2                      -- SCLK freq is CLK / 2**(CLK_DIV+1)
    )
    PORT MAP (
      CLK      => control_clk,          -- clock
      RESET    => tm_rst,               -- reset
      -- input data interface
      WR_CLK   => control_clk,          -- FIFO write clock
      DIN      => config_reg(15 DOWNTO 0),
      WR_EN    => '0',
      WR_PULSE => pulse_reg(1),  -- one pulse writes one word, regardless of pulse duration
      FULL     => OPEN,
      -- output
      SCLK     => dac_sclk,
      DOUT     => dac_dout,
      SYNCn    => dac_sync_n
    );
  dac_sclk_obufds_inst : OBUFDS
    GENERIC MAP (
      IOSTANDARD => "DEFAULT"
    )
    PORT MAP (
      O  => VHDCI1P(7),
      OB => VHDCI1N(7),
      I  => dac_sclk
    );
  dac_dout_obufds_inst : OBUFDS
    GENERIC MAP (
      IOSTANDARD => "DEFAULT"
    )
    PORT MAP (
      O  => VHDCI1P(6),
      OB => VHDCI1N(6),
      I  => dac_dout
    );
  dac_sync_n_obufds_inst : OBUFDS
    GENERIC MAP (
      IOSTANDARD => "DEFAULT"
    )
    PORT MAP (
      O  => VHDCI1P(8),
      OB => VHDCI1N(8),
      I  => dac_sync_n
    );
  topmetal_simple_inst : topmetal_simple PORT MAP (
    RST                  => tm_rst,
    CLK                  => adc_refclk,
    SWG                  => config_reg(16*3-1-8 DOWNTO 16*2),
    BTN                  => tm_btn,
    MARKER_IN            => JC(2),
    MARKER_OUT           => OPEN,
    STOP_CONTROL         => config_reg(16*4-1),
    STOP_ADDRESS         => config_reg(16*4-1-6 DOWNTO 16*3),
    TRIGGER_CONTROL      => config_reg(16*5-2),
    TRIGGER_RATE_CONTROL => config_reg(16*5-1),
    TRIGGER_RATE         => config_reg(16*5-1-12 DOWNTO 16*4),
    TRIGGER_DELAY        => config_reg(16*7-1 DOWNTO 16*6),
    TRIGGER_OUT          => tm_trig_out,
    TM_CLK               => JC(5),
    TM_RST               => JD(0),
    TM_START             => JC(4),
    TM_SPEAK             => JC(0),
    EX_RST_n             => tm_ex_rst_n
  );
  tm_btn(6) <= config_reg(16*3-1-7);
  tm_btn(1) <= config_reg(16*3-1-6);
  tm_btn(0) <= config_reg(16*3-1-5);
  tm_rst    <= reset OR config_reg(16*1+8);
  JD(3)     <= (tm_trig_out AND (NOT config_reg(16*3-2))) OR pulse_reg(0) OR BTN(0);  -- trigger to digitizer
  JC(1)     <= tm_ex_rst_n OR config_reg(16*3-1);  -- ex_rst
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
  usr_data_output(7)          <= sdram_phy_init_done;
  led_obufs : FOR i IN 0 TO 7 GENERATE
    led_obuf : OBUF
      PORT MAP (
        I => usr_data_output(i),
        O => LED8Bit(i)
      );
  END GENERATE led_obufs;
  ---------------------------------------------> Topmetal
  ---------------------------------------------< ADC
  ads5282_idelayctrl0_inst : IDELAYCTRL
    PORT MAP (
      RDY    => OPEN,        -- 1-bit output indicates validity of the REFCLK
      REFCLK => clk_200MHz,             -- 1-bit reference clock input
      RST    => reset                   -- 1-bit reset input
    );
  ads5282_idelayctrl1_inst : IDELAYCTRL
    PORT MAP (
      RDY    => OPEN,        -- 1-bit output indicates validity of the REFCLK
      REFCLK => clk_200MHz,             -- 1-bit reference clock input
      RST    => reset                   -- 1-bit reset input
    );
  ads5282_0interface_inst : ads5282_interface
    GENERIC MAP (
      ADC_NCH => 8
    )
    PORT MAP (
      RESET   => reset,
      CLK     => control_clk,
      --
      CONFIG  => ads5282_0_config,
      CONFPS  => ads5282_0_confps,
      CONFULL => OPEN,
      --
      ADCLKp  => VHDCI1P(9),            -- LVDS frame clock (1X)
      ADCLKn  => VHDCI1N(9),
      LCLKp   => VHDCI1P(10),           -- LVDS bit clock (6X)
      LCLKn   => VHDCI1N(10),
      DATAp   => ads5282_0_data_p,
      DATAn   => ads5282_0_data_n,
      --
      ADCLK   => ads5282_0_adclk,
      DATA    => ads5282_0_data,
      --
      SCLK    => VHDCI1N(0),
      SDATA   => VHDCI1P(0),
      CSn     => VHDCI1P(1)
    );
  ads5282_0_data_p <= (VHDCI1P(19), VHDCI1P(18), VHDCI1P(17), VHDCI1P(16),
                       VHDCI1P(15), VHDCI1P(14), VHDCI1P(13), VHDCI1P(12));
  ads5282_0_data_n <= (VHDCI1N(19), VHDCI1N(18), VHDCI1N(17), VHDCI1N(16),
                       VHDCI1N(15), VHDCI1N(14), VHDCI1N(13), VHDCI1N(12));
  ads5282_0_config <= config_reg(16*10-1 DOWNTO 16*8);
  ads5282_0_confps <= pulse_reg(2);
  ads5282_1interface_inst : ads5282_interface
    GENERIC MAP (
      ADC_NCH => 8
    )
    PORT MAP (
      RESET   => reset,
      CLK     => control_clk,
      --
      CONFIG  => ads5282_1_config,
      CONFPS  => ads5282_1_confps,
      CONFULL => OPEN,
      --
      ADCLKp  => VHDCI2P(12),           -- LVDS frame clock (1X)
      ADCLKn  => VHDCI2N(12),
      LCLKp   => VHDCI2P(11),           -- LVDS bit clock (6X)
      LCLKn   => VHDCI2N(11),
      DATAp   => ads5282_1_data_p,
      DATAn   => ads5282_1_data_n,
      --
      ADCLK   => ads5282_1_adclk,
      DATA    => ads5282_1_data,
      --
      SCLK    => VHDCI2N(0),
      SDATA   => VHDCI2N(1),
      CSn     => VHDCI2N(2)
    );
  ads5282_1_data_p <= (VHDCI2P(19), VHDCI2P(18), VHDCI2P(17), VHDCI2P(16),
                       VHDCI2P(15), VHDCI2P(14), VHDCI2P(13), VHDCI2P(10));
  ads5282_1_data_n <= (VHDCI2N(19), VHDCI2N(18), VHDCI2N(17), VHDCI2N(16),
                       VHDCI2N(15), VHDCI2N(14), VHDCI2N(13), VHDCI2N(10));
  ads5282_1_config <= config_reg(16*10-1 DOWNTO 16*8);
  ads5282_1_confps <= pulse_reg(3);
  ads5282_2interface_inst : ads5282_interface
    GENERIC MAP (
      ADC_NCH => 4
    )
    PORT MAP (
      RESET   => reset,
      CLK     => control_clk,
      --
      CONFIG  => ads5282_2_config,
      CONFPS  => ads5282_2_confps,
      CONFULL => OPEN,
      --
      ADCLKp  => VHDCI2P(8),            -- LVDS frame clock (1X)
      ADCLKn  => VHDCI2N(8),
      LCLKp   => VHDCI2P(9),            -- LVDS bit clock (6X)
      LCLKn   => VHDCI2N(9),
      DATAp   => ads5282_2_data_p,
      DATAn   => ads5282_2_data_n,
      --
      ADCLK   => ads5282_2_adclk,
      DATA    => ads5282_2_data,
      --
      SCLK    => VHDCI2P(0),
      SDATA   => VHDCI2P(1),
      CSn     => VHDCI2P(2)
    );
  ads5282_2_data_p <= (VHDCI2P(6), VHDCI2P(5), VHDCI2P(4), VHDCI2P(3));
  ads5282_2_data_n <= (VHDCI2N(6), VHDCI2N(5), VHDCI2N(4), VHDCI2N(3));
  ads5282_2_config <= config_reg(16*10-1 DOWNTO 16*8);
  ads5282_2_confps <= pulse_reg(4);

  fifo96_inst : fifo96
    PORT MAP (
      RST    => fifo96_trig,
      WR_CLK => fifo96_wrclk,
      RD_CLK => adc_data_fifo_rdclk,
      DIN    => fifo96_din,
      WR_EN  => fifo96_wren,
      RD_EN  => fifo96_rden,
      DOUT   => fifo96_dout,
      FULL   => OPEN,
      EMPTY  => fifo96_empty
    );
  fifo96_wren  <= '1';
  fifo96_trig  <= pulse_reg(5);
  fifo96_valid <= NOT fifo96_empty;
  fifo_rdwidth_reducer_inst : fifo_rdwidth_reducer
    GENERIC MAP (
      RDWIDTH => 32,
      RDRATIO => 3
    )
    PORT MAP (
      RESET => fifo96_trig,
      CLK   => adc_data_fifo_rdclk,
      -- input data interface
      DIN   => fifo96_dout,
      VALID => fifo96_valid,
      RDREQ => fifo96_rden,
      -- output
      DOUT  => adc_data_fifo_q,
      EMPTY => adc_data_fifo_empty,
      RD_EN => adc_data_fifo_rdreq
    );
  WITH config_reg(16*10+1 DOWNTO 16*10) SELECT
    fifo96_wrclk <= ads5282_0_adclk WHEN "00",
    ads5282_1_adclk                 WHEN "01",
    ads5282_2_adclk                 WHEN "10",
    control_clk                     WHEN OTHERS;
  
  WITH config_reg(16*10+1 DOWNTO 16*10) SELECT
    fifo96_din <= ads5282_0_data(7) & ads5282_0_data(6) & ads5282_0_data(5) & ads5282_0_data(4) &
                  ads5282_0_data(3) & ads5282_0_data(2) & ads5282_0_data(1) & ads5282_0_data(0)
    WHEN "00",
    ads5282_1_data(7) & ads5282_1_data(6) & ads5282_1_data(5) & ads5282_1_data(4) &
    ads5282_1_data(3) & ads5282_1_data(2) & ads5282_1_data(1) & ads5282_1_data(0)
    WHEN "01",
    ads5282_2_data(3) & ads5282_2_data(2) & ads5282_2_data(1) & ads5282_2_data(0) &
    ads5282_2_data(3) & ads5282_2_data(2) & ads5282_2_data(1) & ads5282_2_data(0)
    WHEN "10",
    x"000000000000000000000000" WHEN OTHERS;

cs_trig0(30 DOWNTO 19) <= ads5282_0_data(0) XOR ads5282_0_data(1) XOR ads5282_0_data(2) XOR ads5282_0_data(3) XOR ads5282_0_data(4) XOR ads5282_0_data(5) XOR ads5282_0_data(6) XOR ads5282_0_data(7)
                            XOR
ads5282_1_data(0) XOR ads5282_1_data(1) XOR ads5282_1_data(2) XOR ads5282_1_data(3) XOR ads5282_1_data(4) XOR ads5282_1_data(5) XOR ads5282_1_data(6) XOR ads5282_1_data(7)
                            XOR
ads5282_2_data(0) XOR ads5282_2_data(1) XOR ads5282_2_data(2) XOR ads5282_2_data(3);
  cs_trig0(31)           <= ads5282_0_adclk;
  ---------------------------------------------> ADC

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
