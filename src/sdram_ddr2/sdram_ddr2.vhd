----------------------------------------------------------------------------------
-- Company:  LBNL
-- Engineer: Yuan Mei
-- 
-- Create Date: 7/4/2015 07:22:25 PM
-- Design Name: 
-- Module Name: sdram_ddr2 - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
--
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
USE ieee.numeric_std.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
LIBRARY UNISIM;
USE UNISIM.VComponents.ALL;

ENTITY sdram_ddr2 IS
  GENERIC (
    INDATA_WIDTH   : positive := 256;
    OUTDATA_WIDTH  : positive := 32;
    APP_ADDR_WIDTH : positive := 31;
    APP_DATA_WIDTH : positive := 128;
    APP_MASK_WIDTH : positive := 16;
    APP_ADDR_BURST : positive := 8
  );
  PORT (
    CLK        : IN    std_logic;
    CLK200     : IN    std_logic;       -- 200MHz reference clock for iodelay
    RESET      : IN    std_logic;
    --
    DDR2_DQ    : INOUT std_logic_vector(63 DOWNTO 0);
    DDR2_DQS   : INOUT std_logic_vector(7 DOWNTO 0);
    DDR2_DQS_N : INOUT std_logic_vector(7 DOWNTO 0);
    DDR2_A     : OUT   std_logic_vector(12 DOWNTO 0);
    DDR2_BA    : OUT   std_logic_vector(1 DOWNTO 0);
    DDR2_RAS_N : OUT   std_logic;
    DDR2_CAS_N : OUT   std_logic;
    DDR2_WE_N  : OUT   std_logic;
    DDR2_CS_N  : OUT   std_logic_vector(0 DOWNTO 0);
    DDR2_ODT   : OUT   std_logic_vector(0 DOWNTO 0);
    DDR2_CKE   : OUT   std_logic_vector(0 DOWNTO 0);
    DDR2_DM    : OUT   std_logic_vector(7 DOWNTO 0);
    DDR2_CK    : OUT   std_logic_vector(1 DOWNTO 0);
    DDR2_CK_N  : OUT   std_logic_vector(1 DOWNTO 0);
    -- Status Outputs
    PHY_INIT_DONE : OUT std_logic
  );
END sdram_ddr2;

ARCHITECTURE Behavioral OF sdram_ddr2 IS

  COMPONENT mig IS
    GENERIC (
      BANK_WIDTH            : integer := 2;
      -- # of memory bank addr bits.
      CKE_WIDTH             : integer := 1;
      -- # of memory clock enable outputs.
      CLK_WIDTH             : integer := 2;
      -- # of clock outputs.
      COL_WIDTH             : integer := 10;
      -- # of memory column bits.
      CS_NUM                : integer := 1;
      -- # of separate memory chip selects.
      CS_WIDTH              : integer := 1;
      -- # of total memory chip selects.
      CS_BITS               : integer := 0;
      -- set to log2(CS_NUM) (rounded up).
      DM_WIDTH              : integer := 8;
      -- # of data mask bits.
      DQ_WIDTH              : integer := 64;
      -- # of data width.
      DQ_PER_DQS            : integer := 8;
      -- # of DQ data bits per strobe.
      DQS_WIDTH             : integer := 8;
      -- # of DQS strobes.
      DQ_BITS               : integer := 6;
      -- set to log2(DQS_WIDTH*DQ_PER_DQS).
      DQS_BITS              : integer := 3;
      -- set to log2(DQS_WIDTH).
      ODT_WIDTH             : integer := 1;
      -- # of memory on-die term enables.
      ROW_WIDTH             : integer := 13;
      -- # of memory row and # of addr bits.
      ADDITIVE_LAT          : integer := 0;
      -- additive write latency.
      BURST_LEN             : integer := 8;
      -- burst length (in double words).
      BURST_TYPE            : integer := 0;
      -- burst type (=0 seq; =1 interleaved).
      CAS_LAT               : integer := 4;
      -- CAS latency.
      ECC_ENABLE            : integer := 0;
      -- enable ECC (=1 enable).
      APPDATA_WIDTH         : integer := 128;
      -- # of usr read/write data bus bits.
      MULTI_BANK_EN         : integer := 1;
      -- Keeps multiple banks open. (= 1 enable).
      TWO_T_TIME_EN         : integer := 1;
      -- 2t timing for unbuffered dimms.
      ODT_TYPE              : integer := 1;
      -- ODT (=0(none),=1(75),=2(150),=3(50)).
      REDUCE_DRV            : integer := 0;
      -- reduced strength mem I/O (=1 yes).
      REG_ENABLE            : integer := 0;
      -- registered addr/ctrl (=1 yes).
      TREFI_NS              : integer := 7800;
      -- auto refresh interval (ns).
      TRAS                  : integer := 40000;
      -- active->precharge delay.
      TRCD                  : integer := 15000;
      -- active->read/write delay.
      TRFC                  : integer := 105000;
      -- refresh->refresh, refresh->active delay.
      TRP                   : integer := 15000;
      -- precharge->command delay.
      TRTP                  : integer := 7500;
      -- read->precharge delay.
      TWR                   : integer := 15000;
      -- used to determine write->precharge.
      TWTR                  : integer := 7500;
      -- write->read delay.
      HIGH_PERFORMANCE_MODE : boolean := true;
      -- # = TRUE, the IODELAY performance mode is set
      -- to high.
      -- # = FALSE, the IODELAY performance mode is set
      -- to low.
      SIM_ONLY              : integer := 0;
      -- = 1 to skip SDRAM power up delay.
      DEBUG_EN              : integer := 0;
      -- Enable debug signals/controls.
      -- When this parameter is changed from 0 to 1,
      -- make sure to uncomment the coregen commands
      -- in ise_flow.bat or create_ise.bat files in
      -- par folder.
      CLK_PERIOD            : integer := 3750;
      -- Core/Memory clock period (in ps).
      DLL_FREQ_MODE         : string  := "HIGH";
      -- DCM Frequency range.
      CLK_TYPE              : string  := "SINGLE_ENDED";
      -- # = "DIFFERENTIAL " ->; Differential input clocks ,
      -- # = "SINGLE_ENDED" -> Single ended input clocks.
      NOCLK200              : boolean := false;
      -- clk200 enable and disable
      RST_ACT_LOW           : integer := 1
      -- =1 for active low reset, =0 for active high.
    );
    PORT (
      DDR2_DQ           : INOUT std_logic_vector((DQ_WIDTH-1) DOWNTO 0);
      DDR2_A            : OUT   std_logic_vector((ROW_WIDTH-1) DOWNTO 0);
      DDR2_BA           : OUT   std_logic_vector((BANK_WIDTH-1) DOWNTO 0);
      DDR2_RAS_N        : OUT   std_logic;
      DDR2_CAS_N        : OUT   std_logic;
      DDR2_WE_N         : OUT   std_logic;
      DDR2_CS_N         : OUT   std_logic_vector((CS_WIDTH-1) DOWNTO 0);
      DDR2_ODT          : OUT   std_logic_vector((ODT_WIDTH-1) DOWNTO 0);
      DDR2_CKE          : OUT   std_logic_vector((CKE_WIDTH-1) DOWNTO 0);
      DDR2_DM           : OUT   std_logic_vector((DM_WIDTH-1) DOWNTO 0);
      SYS_CLK           : IN    std_logic;
      IDLY_CLK_200      : IN    std_logic;
      SYS_RST_N         : IN    std_logic;
      PHY_INIT_DONE     : OUT   std_logic;
      RST0_TB           : OUT   std_logic;
      CLK0_TB           : OUT   std_logic;
      APP_WDF_AFULL     : OUT   std_logic;
      APP_AF_AFULL      : OUT   std_logic;
      RD_DATA_VALID     : OUT   std_logic;
      APP_WDF_WREN      : IN    std_logic;
      APP_AF_WREN       : IN    std_logic;
      APP_AF_ADDR       : IN    std_logic_vector(30 DOWNTO 0);
      APP_AF_CMD        : IN    std_logic_vector(2 DOWNTO 0);
      RD_DATA_FIFO_OUT  : OUT   std_logic_vector((APPDATA_WIDTH-1) DOWNTO 0);
      APP_WDF_DATA      : IN    std_logic_vector((APPDATA_WIDTH-1) DOWNTO 0);
      APP_WDF_MASK_DATA : IN    std_logic_vector((APPDATA_WIDTH/8-1) DOWNTO 0);
      DDR2_DQS          : INOUT std_logic_vector((DQS_WIDTH-1) DOWNTO 0);
      DDR2_DQS_N        : INOUT std_logic_vector((DQS_WIDTH-1) DOWNTO 0);
      DDR2_CK           : OUT   std_logic_vector((CLK_WIDTH-1) DOWNTO 0);
      DDR2_CK_N         : OUT   std_logic_vector((CLK_WIDTH-1) DOWNTO 0)
    );
  END COMPONENT;

  SIGNAL reset_n           : std_logic;
  SIGNAL rst0_tb           : std_logic;
  SIGNAL clk0_tb           : std_logic;
  SIGNAL app_wdf_afull     : std_logic;
  SIGNAL app_af_afull      : std_logic;
  SIGNAL rd_data_valid     : std_logic;
  SIGNAL app_wdf_wren      : std_logic;
  SIGNAL app_af_wren       : std_logic;
  SIGNAL app_af_addr       : std_logic_vector(APP_ADDR_WIDTH-1 DOWNTO 0);
  SIGNAL app_af_cmd        : std_logic_vector(2 DOWNTO 0);
  SIGNAL rd_data_fifo_out  : std_logic_vector((APP_DATA_WIDTH-1) DOWNTO 0);
  SIGNAL app_wdf_data      : std_logic_vector((APP_DATA_WIDTH-1) DOWNTO 0);
  SIGNAL app_wdf_mask_data : std_logic_vector((APP_DATA_WIDTH/8-1) DOWNTO 0);

BEGIN 

  reset_n <= NOT RESET;
  mig_inst : mig
    PORT MAP (
      DDR2_DQ           => DDR2_DQ,
      DDR2_A            => DDR2_A,
      DDR2_BA           => DDR2_BA,
      DDR2_RAS_N        => DDR2_RAS_N,
      DDR2_CAS_N        => DDR2_CAS_N,
      DDR2_WE_N         => DDR2_WE_N,
      DDR2_CS_N         => DDR2_CS_N,
      DDR2_ODT          => DDR2_ODT,
      DDR2_CKE          => DDR2_CKE,
      DDR2_DM           => DDR2_DM,
      SYS_CLK           => clk200,
      IDLY_CLK_200      => clk200,
      SYS_RST_N         => reset_n,
      PHY_INIT_DONE     => PHY_INIT_DONE,
      RST0_TB           => rst0_tb,     -- reset_n sync-ed to clk200 domain
      CLK0_TB           => clk0_tb,     -- this is exactly the same as clk200
      APP_WDF_AFULL     => app_wdf_afull,
      APP_AF_AFULL      => app_af_afull,
      RD_DATA_VALID     => rd_data_valid,
      APP_WDF_WREN      => app_wdf_wren,
      APP_AF_WREN       => app_af_wren,
      APP_AF_ADDR       => app_af_addr,
      APP_AF_CMD        => app_af_cmd,
      RD_DATA_FIFO_OUT  => rd_data_fifo_out,
      APP_WDF_DATA      => app_wdf_data,
      APP_WDF_MASK_DATA => app_wdf_mask_data,
      DDR2_DQS          => DDR2_DQS,
      DDR2_DQS_N        => DDR2_DQS_N,
      DDR2_CK           => DDR2_CK,
      DDR2_CK_N         => DDR2_CK_N
    );

  app_wdf_wren <= '1';
  app_af_wren  <= '1';
  app_af_addr  <= (OTHERS => '0');
  app_af_cmd   <= (OTHERS => '0');

END Behavioral;
