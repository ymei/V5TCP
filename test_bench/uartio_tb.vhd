--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   01:05:29 10/27/2013
-- Design Name:   
-- Module Name:   /home/ymei/Work/UltrafastImaging/FPGAUDP/test_bench/uartio_tb.vhd
-- Project Name:  top
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: uartio
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

ENTITY uartio_tb IS
END uartio_tb;

ARCHITECTURE behavior OF uartio_tb IS

  -- Component Declaration for the Unit Under Test (UUT)

  COMPONENT uartio
    GENERIC(
      ClkFreq         : positive;
      BaudRate        : positive;
      BaudGenAccWidth : positive;
      BaudGenInc      : positive
    );
    PORT(
      CLK     : IN  std_logic;
      RESET   : IN  std_logic;
      RX_DATA : OUT std_logic_vector(7 DOWNTO 0);
      RX_RDY  : OUT std_logic;
      TX_DATA : IN  std_logic_vector(7 DOWNTO 0);
      TX_EN   : IN  std_logic;
      TX_BUSY : OUT std_logic;
      RX_PIN  : IN  std_logic;
      TX_PIN  : OUT std_logic
    );
  END COMPONENT;


  --Inputs
  SIGNAL CLK     : std_logic                    := '0';
  SIGNAL RESET   : std_logic                    := '0';
  SIGNAL TX_DATA : std_logic_vector(7 DOWNTO 0) := (OTHERS => '0');
  SIGNAL TX_EN   : std_logic                    := '0';
  SIGNAL RX_PIN  : std_logic;

  --Outputs
  SIGNAL RX_DATA : std_logic_vector(7 DOWNTO 0);
  SIGNAL RX_RDY  : std_logic;
  SIGNAL TX_PIN  : std_logic;
  SIGNAL TX_BUSY : std_logic;

  -- Clock period definitions
  CONSTANT CLK_period : time := 10 ns;

BEGIN

  -- Instantiate the Unit Under Test (UUT)
  uut : uartio
    GENERIC MAP(
      ClkFreq         => 1000000,
      BaudRate        => 9600,
      BaudGenAccWidth => 8,
      BaudGenInc      => 2
    )
    PORT MAP (
      CLK     => CLK,
      RESET   => RESET,
      RX_DATA => RX_DATA,
      RX_RDY  => RX_RDY,
      TX_DATA => TX_DATA,
      TX_EN   => TX_EN,
      TX_BUSY => TX_BUSY,
      RX_PIN  => RX_PIN,
      TX_PIN  => TX_PIN
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
    -- hold reset state for 100 ns.
    RESET <= '1';
    WAIT FOR 75 ns;
    RESET <= '0';
    WAIT FOR CLK_period*3;

    -- insert stimulus here 
    TX_DATA <= x"a5";
    TX_EN <= '1';
    WAIT FOR CLK_period;
    --TX_EN <= '0';
    WAIT;
  END PROCESS;

END;
