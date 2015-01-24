--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   18:18:25 01/22/2015
-- Design Name:   
-- Module Name:   test_bench/topmetal_iiminus_analog_tb.vhd
-- Project Name:  
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: topmetal_iiminus_analog
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
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
USE IEEE.NUMERIC_STD.ALL;
 
ENTITY topmetal_iiminus_analog_tb IS
END topmetal_iiminus_analog_tb;
 
ARCHITECTURE behavior OF topmetal_iiminus_analog_tb IS

  -- Component Declaration for the Unit Under Test (UUT)
  
  COMPONENT topmetal_iiminus_analog
    PORT(
      CLK           : IN  std_logic;
      RESET         : IN  std_logic;
      MEM_CLK       : IN  std_logic;
      MEM_WE        : IN  std_logic;
      MEM_ADDR      : IN  std_logic_vector(31 DOWNTO 0);
      MEM_DIN       : IN  std_logic_vector(31 DOWNTO 0);
      SRAM_WR_START : IN  std_logic;
      CLK_DIV       : IN  std_logic_vector(3 DOWNTO 0);
      STOP_ADDR     : IN  std_logic_vector(15 DOWNTO 0);
      TRIGGER_RATE  : IN  std_logic_vector(15 DOWNTO 0);
      TRIGGER_DELAY : IN  std_logic_vector(15 DOWNTO 0);
      MARKER_A      : IN  std_logic;
      TRIGGER_OUT   : OUT std_logic;
      SRAM_D        : OUT std_logic_vector(4 DOWNTO 0);
      SRAM_WE       : OUT std_logic;
      TM_RST        : OUT std_logic;
      TM_CLK_S      : OUT std_logic;
      TM_RST_S      : OUT std_logic;
      TM_START_S    : OUT std_logic;
      TM_SPEAK_S    : OUT std_logic
      );
  END COMPONENT;


  --Inputs
  SIGNAL CLK           : std_logic                     := '0';
  SIGNAL RESET         : std_logic                     := '0';
  SIGNAL MEM_CLK       : std_logic                     := '0';
  SIGNAL MEM_WE        : std_logic                     := '0';
  SIGNAL MEM_ADDR      : std_logic_vector(31 DOWNTO 0) := (OTHERS => '0');
  SIGNAL MEM_DIN       : std_logic_vector(31 DOWNTO 0) := (OTHERS => '0');
  SIGNAL SRAM_WR_START : std_logic                     := '0';
  SIGNAL CLK_DIV       : std_logic_vector(3 DOWNTO 0)  := x"1";
  SIGNAL STOP_ADDR     : std_logic_vector(15 DOWNTO 0) := x"0001";
  SIGNAL TRIGGER_RATE  : std_logic_vector(15 DOWNTO 0) := x"0004";
  SIGNAL TRIGGER_DELAY : std_logic_vector(15 DOWNTO 0) := x"0001";
  SIGNAL MARKER_A      : std_logic                     := '0';

  --Outputs
  SIGNAL TRIGGER_OUT : std_logic;
  SIGNAL SRAM_D      : std_logic_vector(4 DOWNTO 0);
  SIGNAL SRAM_WE     : std_logic;
  SIGNAL TM_RST      : std_logic;
  SIGNAL TM_CLK_S    : std_logic;
  SIGNAL TM_RST_S    : std_logic;
  SIGNAL TM_START_S  : std_logic;
  SIGNAL TM_SPEAK_S  : std_logic;

  -- Clock period definitions
  CONSTANT CLK_period     : time := 40 ns;
  CONSTANT MEM_CLK_period : time := 8  ns;
  
BEGIN

  -- Instantiate the Unit Under Test (UUT)
  uut : topmetal_iiminus_analog PORT MAP (
    CLK           => CLK,
    RESET         => RESET,
    MEM_CLK       => MEM_CLK,
    MEM_WE        => MEM_WE,
    MEM_ADDR      => MEM_ADDR,
    MEM_DIN       => MEM_DIN,
    SRAM_WR_START => SRAM_WR_START,
    CLK_DIV       => CLK_DIV,
    STOP_ADDR     => STOP_ADDR,
    TRIGGER_RATE  => TRIGGER_RATE,
    TRIGGER_DELAY => TRIGGER_DELAY,
    MARKER_A      => MARKER_A,
    TRIGGER_OUT   => TRIGGER_OUT,
    SRAM_D        => SRAM_D,
    SRAM_WE       => SRAM_WE,
    TM_RST        => TM_RST,
    TM_CLK_S      => TM_CLK_S,
    TM_RST_S      => TM_RST_S,
    TM_START_S    => TM_START_S,
    TM_SPEAK_S    => TM_SPEAK_S
  );

  -- Clock process definitions
  CLK_process : PROCESS
  BEGIN
    CLK <= '0';
    WAIT FOR CLK_period/2;
    CLK <= '1';
    WAIT FOR CLK_period/2;
  END PROCESS;

  MEM_CLK_process : PROCESS
  BEGIN
    MEM_CLK <= '0';
    WAIT FOR MEM_CLK_period/2;
    MEM_CLK <= '1';
    WAIT FOR MEM_CLK_period/2;
  END PROCESS;

  -- Stimulus process
  stim_proc : PROCESS
  BEGIN
    -- hold reset state for 100 ns.
    WAIT FOR 100 ns;
    RESET <= '1';
    WAIT FOR CLK_period*10;
    RESET <= '0';
    -- insert stimulus here 
    MEM_DIN  <= x"12345678";
    MEM_ADDR <= x"00000000";
    WAIT FOR MEM_CLK_period;
    MEM_WE   <= '1';
    WAIT FOR MEM_CLK_period;
    MEM_DIN  <= x"9abcdef0";
    MEM_ADDR <= x"00000001";

    WAIT FOR 425ns;
    SRAM_WR_START <= '1';
    WAIT FOR MEM_CLK_period;
    SRAM_WR_START <= '0';
    WAIT;
  END PROCESS;

END;
