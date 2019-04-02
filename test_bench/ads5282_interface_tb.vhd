--------------------------------------------------------------------------------
-- Company:
-- Engineer:
--
-- Create Date:   03:59:37 06/26/2015
-- Design Name:
-- Module Name:   ads5282_interface_tb.vhd
-- Project Name:  V5TCP
-- Target Device:
-- Tool versions:
-- Description:
--
-- VHDL Test Bench Created by ISE for module: ads5282_interface
--
-- Dependencies:
--
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
-- Notes:
-- This testbench has been automatically generated using types std_logic and
-- std_logic_vector for the ports of the unit under test.  Xilinx recommends
-- that these types always be used for the top-level I/O of a design in order
-- to guarantee that the testbench will bind correctly to the post-implementation
-- simulation model.
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
USE ieee.numeric_std.ALL;

ENTITY ads5282_interface_tb IS
END ads5282_interface_tb;

USE work.common_pkg.ALL;

ARCHITECTURE behavior OF ads5282_interface_tb IS

  -- Component Declaration for the Unit Under Test (UUT)

  COMPONENT ads5282_interface
    PORT(
      RESET   : IN  std_logic;
      CLK     : IN  std_logic;
      CONFIG  : IN  std_logic_vector(31 DOWNTO 0);
      CONFPS  : IN  std_logic;
      CONFULL : OUT std_logic;
      ADCLKp  : IN  std_logic;
      ADCLKn  : IN  std_logic;
      LCLKp   : IN  std_logic;
      LCLKn   : IN  std_logic;
      DATAp   : IN  std_logic_vector(7 DOWNTO 0);
      DATAn   : IN  std_logic_vector(7 DOWNTO 0);
      ADCLK   : OUT std_logic;
      DATA    : OUT ADS5282DATA(7 DOWNTO 0);
      SCLK    : OUT std_logic;
      SDATA   : OUT std_logic;
      CSn     : OUT std_logic
    );
  END COMPONENT;

  --Inputs
  SIGNAL RESET  : std_logic                     := '0';
  SIGNAL CLK    : std_logic                     := '0';
  SIGNAL CONFIG : std_logic_vector(31 DOWNTO 0) := (OTHERS => '0');
  SIGNAL CONFPS : std_logic                     := '0';
  SIGNAL ADCLKp : std_logic                     := '0';
  SIGNAL ADCLKn : std_logic                     := '0';
  SIGNAL LCLKp  : std_logic                     := '0';
  SIGNAL LCLKn  : std_logic                     := '0';
  SIGNAL DATAp  : std_logic_vector(7 DOWNTO 0)  := (OTHERS => '0');
  SIGNAL DATAn  : std_logic_vector(7 DOWNTO 0)  := (OTHERS => '0');

  --Outputs
  SIGNAL CONFULL : std_logic;
  SIGNAL ADCLK   : std_logic;
  SIGNAL DATA    : ADS5282DATA(7 DOWNTO 0);
  SIGNAL SCLK    : std_logic;
  SIGNAL SDATA   : std_logic;
  SIGNAL CSn     : std_logic;

  -- Clock period definitions
  CONSTANT CLK_period   : time := 10 ns;
  CONSTANT ADCLK_period : time := 15.6 ns;
  CONSTANT LCLK_period  : time := 2.6 ns;

BEGIN

  -- Instantiate the Unit Under Test (UUT)
  uut : ads5282_interface PORT MAP (
    RESET   => RESET,
    CLK     => CLK,
    CONFIG  => CONFIG,
    CONFPS  => CONFPS,
    CONFULL => CONFULL,
    ADCLKp  => ADCLKp,
    ADCLKn  => ADCLKn,
    LCLKp   => LCLKp,
    LCLKn   => LCLKn,
    DATAp   => DATAp,
    DATAn   => DATAn,
    ADCLK   => ADCLK,
    DATA    => DATA,
    SCLK    => SCLK,
    SDATA   => SDATA,
    CSn     => CSn
  );

  -- Clock process definitions
  CLK_process : PROCESS
  BEGIN
    CLK <= '0';
    WAIT FOR CLK_period/2;
    CLK <= '1';
    WAIT FOR CLK_period/2;
  END PROCESS;

  ADCLK_process : PROCESS
  BEGIN
    WAIT FOR ADCLK_period/16.0;
    ADCLKp <= '0';
    WAIT FOR ADCLK_period/2;
    ADCLKp <= '1';
    WAIT FOR ADCLK_period*7/16;
  END PROCESS;
  ADCLKn <= NOT ADCLKp;

  LCLK_process : PROCESS
  BEGIN
    LCLKp <= '0';
    WAIT FOR LCLK_period/2;
    LCLKp <= '1';
    WAIT FOR LCLK_period/2;
  END PROCESS;
  LCLKn <= NOT LCLKp;

  DATA_PROCESS : PROCESS
  BEGIN
    WAIT FOR LCLK_period/4;
    DATAp <= (OTHERS => '1');
    DATAn <= (OTHERS => '0');
    WAIT FOR LCLK_period/2;
    DATAp <= (OTHERS => '1');
    DATAn <= (OTHERS => '0');
    WAIT FOR LCLK_period/2;
    DATAp <= (OTHERS => '1');
    DATAn <= (OTHERS => '0');
    WAIT FOR LCLK_period/2;
    DATAp <= (OTHERS => '0');
    DATAn <= (OTHERS => '1');
    WAIT FOR LCLK_period/4;
  END PROCESS;

  -- Stimulus process
  stim_proc : PROCESS
  BEGIN
    -- hold reset state for 100 ns.
    RESET <= '0';
    WAIT FOR 100 ns;
    RESET <= '1';
    WAIT FOR CLK_period*10;
    RESET <= '0';
    -- insert stimulus here
    WAIT FOR CLK_period*10;
    WAIT FOR CLK_period/1.99;
    CONFIG <= x"7fffffff";              -- bufrCLR
    CONFPS <= '1';
    WAIT FOR CLK_period*1;
    CONFPS <= '0';
    WAIT FOR CLK_period*5;
    CONFIG <= x"7fffff8f";              -- lclkIodelay
    CONFPS <= '1';
    WAIT FOR CLK_period*1;
    CONFPS <= '0';
    WAIT FOR CLK_period*20;
    CONFIG <= x"00000601";              -- bitSlipP(0)
    CONFPS <= '1';
    WAIT FOR CLK_period*1;
    CONFPS <= '0';
    WAIT FOR CLK_period*1;
    CONFIG <= x"00000642";              -- bitSlipP(0)
    CONFPS <= '1';
    WAIT FOR CLK_period*1;
    CONFPS <= '0';
    WAIT FOR CLK_period*1;
    CONFIG <= x"00000683";              -- bitSlipP(0)
    CONFPS <= '1';
    WAIT FOR CLK_period*1;
    CONFPS <= '0';
    WAIT FOR CLK_period*1;
    CONFIG <= x"000006c4";              -- bitSlipP(0)
    CONFPS <= '1';
    WAIT FOR CLK_period*1;
    CONFPS <= '0';
    WAIT FOR CLK_period*1;
    CONFIG <= x"00000705";              -- bitSlipP(0)
    CONFPS <= '1';
    WAIT FOR CLK_period*1;
    CONFPS <= '0';
    WAIT FOR CLK_period*1;
    CONFIG <= x"00000746";              -- bitSlipP(0)
    CONFPS <= '1';
    WAIT FOR CLK_period*1;
    CONFPS <= '0';
    WAIT FOR CLK_period*1;
    CONFIG <= x"00000787";              -- bitSlipP(0)
    CONFPS <= '1';
    WAIT FOR CLK_period*1;
    CONFPS <= '0';
    WAIT FOR CLK_period*1;
    CONFIG <= x"000007c8";              -- bitSlipP(0)
    CONFPS <= '1';
    WAIT FOR CLK_period*1;

    CONFPS <= '0';
    WAIT FOR CLK_period*1;
    CONFIG <= x"00000602";              -- bitSlipN(0)
    CONFPS <= '1';
    WAIT FOR CLK_period*1;
    CONFPS <= '0';
    WAIT FOR CLK_period*1;
    CONFIG <= x"00000051";              -- dataPIodelay(1)
    CONFPS <= '1';
    WAIT FOR CLK_period*1;
    CONFPS <= '0';
    WAIT FOR CLK_period*1;
    CONFIG <= x"80000051";              -- serial
    CONFPS <= '1';
    WAIT FOR CLK_period*1;
    CONFPS <= '0';
    WAIT;
  END PROCESS;

END;
