--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   01:25:13 06/26/2015
-- Design Name:   
-- Module Name:   width_pulse_sync_tb.vhd
-- Project Name:  V5TCP
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: width_pulse_sync
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
 
ENTITY width_pulse_sync_tb IS
END width_pulse_sync_tb;
 
ARCHITECTURE behavior OF width_pulse_sync_tb IS

  -- Component Declaration for the Unit Under Test (UUT)

  COMPONENT width_pulse_sync
    GENERIC (
      DATA_WIDTH : positive := 6;
      MODE       : natural  := 0
    );
    PORT(
      RESET : IN  std_logic;
      CLK   : IN  std_logic;
      PW    : IN  std_logic_vector(DATA_WIDTH-1 DOWNTO 0);
      START : IN  std_logic;
      BUSY  : OUT std_logic;
      CLKO  : IN  std_logic;
      RSTO  : OUT std_logic;
      PO    : OUT std_logic
    );
  END COMPONENT;

  --Inputs
  SIGNAL RESET : std_logic                    := '0';
  SIGNAL CLK   : std_logic                    := '0';
  SIGNAL PW    : std_logic_vector(5 DOWNTO 0) := (OTHERS => '0');
  SIGNAL START : std_logic                    := '0';
  SIGNAL CLKO  : std_logic                    := '0';

  --Outputs
  SIGNAL BUSY : std_logic;
  SIGNAL RSTO : std_logic;
  SIGNAL PO   : std_logic;

  -- Clock period definitions
  CONSTANT CLK_period  : time := 10 ns;
  CONSTANT CLKO_period : time := 33 ns;

BEGIN

  -- Instantiate the Unit Under Test (UUT)
  uut : width_pulse_sync
    PORT MAP (
      RESET => RESET,
      CLK   => CLK,
      PW    => PW,
      START => START,
      BUSY  => BUSY,
      CLKO  => CLKO,
      RSTO  => RSTO,
      PO    => PO
    );

  -- Clock process definitions
  CLK_process : PROCESS
  BEGIN
    CLK <= '0';
    WAIT FOR CLK_period/2;
    CLK <= '1';
    WAIT FOR CLK_period/2;
  END PROCESS;

  CLKO_process : PROCESS
  BEGIN
    CLKO <= '0';
    WAIT FOR CLKO_period/2;
    CLKO <= '1';
    WAIT FOR CLKO_period/2;
  END PROCESS;

  -- Stimulus process
  stim_proc : PROCESS
  BEGIN
    -- hold reset state for 100 ns.
    RESET <= '0';
    WAIT FOR 5.0 ns;
    RESET <= '1';
    WAIT FOR 100 ns;
    RESET <= '0';
    WAIT FOR CLK_period*10;

    -- insert stimulus here 
    PW    <= std_logic_vector(to_unsigned(5,PW'length));
    START <= '1';
    WAIT FOR CLK_period*2;
    START <= '0';
    WAIT;
  END PROCESS;

END;
