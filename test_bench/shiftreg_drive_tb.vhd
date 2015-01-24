--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   14:39:52 01/07/2015
-- Design Name:   shiftreg_drive_tb
-- Module Name:   
-- Project Name:  
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: shiftreg_drive
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
 
ENTITY shiftreg_drive_tb IS
END shiftreg_drive_tb;
 
ARCHITECTURE behavior OF shiftreg_drive_tb IS

  -- Component Declaration for the Unit Under Test (UUT)
  
  COMPONENT shiftreg_drive
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


  --Inputs
  SIGNAL CLK   : std_logic                     := '0';
  SIGNAL RESET : std_logic                     := '0';
  SIGNAL DATA  : std_logic_vector(31 DOWNTO 0) := (OTHERS => '0');
  SIGNAL START : std_logic                     := '0';

  --Outputs
  SIGNAL SCLK  : std_logic;
  SIGNAL DOUT  : std_logic;
  SIGNAL SYNCn : std_logic;

  -- Clock period definitions
  CONSTANT CLK_period  : time := 10 ns;
  CONSTANT SCLK_period : time := 10 ns;
  
BEGIN

  -- Instantiate the Unit Under Test (UUT)
  uut : shiftreg_drive PORT MAP (
    CLK   => CLK,
    RESET => RESET,
    DATA  => DATA,
    START => START,
    SCLK  => SCLK,
    DOUT  => DOUT,
    SYNCn => SYNCn
  );

  -- Clock process definitions
  CLK_process : PROCESS
  BEGIN
    CLK <= '0';
    WAIT FOR CLK_period/2;
    CLK <= '1';
    WAIT FOR CLK_period/2;
  END PROCESS;

  -- Stimulus process
  stim_proc : PROCESS
  BEGIN 
    WAIT FOR 20ns;
    -- hold reset state for 80 ns.
    RESET <= '1';
    WAIT FOR 80 ns;
    RESET <= '0';
    WAIT FOR CLK_period*12;

    -- insert stimulus here 
    START <= '1';
    DATA  <= x"a5a5ffa5";
    WAIT FOR CLK_period;
    START <= '0';
    WAIT;
  END PROCESS;

END;
