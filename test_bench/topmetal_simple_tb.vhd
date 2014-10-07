--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   15:16:42 10/06/2014
-- Design Name:   
-- Module Name:   D:/VMShared/V5TCP/test_bench/topmetal_simple_tb.vhd
-- Project Name:  V5TCP
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: topmetal_simple
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
--USE ieee.numeric_std.ALL;
 
ENTITY topmetal_simple_tb IS
END topmetal_simple_tb;
 
ARCHITECTURE behavior OF topmetal_simple_tb IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT topmetal_simple
    GENERIC (
      TRIGGER_DELAY_WIDTH : positive := 16
    );
    PORT(
         rst : IN  std_logic;
         clk : IN  std_logic;
         SWG : IN  std_logic_vector(7 downto 0);
         BTN : IN  std_logic_vector(6 downto 0);
         MARKER_IN : IN  std_logic;
         MARKER_OUT : OUT  std_logic;
         stop_control : IN  std_logic;
         stop_address : IN  std_logic_vector(9 downto 0);
         trigger_control : IN  std_logic;
         trigger_rate_control : IN  std_logic;
         trigger_rate : IN  std_logic_vector(3 downto 0);
         trigger_delay : IN  std_logic_vector (TRIGGER_DELAY_WIDTH-1 DOWNTO 0);
         trigger_out   : OUT std_logic;
         TM_CLK : OUT  std_logic;
         TM_RST : OUT  std_logic;
         TM_START : OUT  std_logic;
         TM_SPEAK : OUT  std_logic;
         ex_rst : OUT  std_logic;
         ex_rst_veto : IN std_logic
        );
    END COMPONENT;
    

   --Inputs
   signal rst : std_logic := '0';
   signal clk : std_logic := '0';
   signal SWG : std_logic_vector(7 downto 0) := (others => '0');
   signal BTN : std_logic_vector(6 downto 0) := (others => '0');
   signal MARKER_IN : std_logic := '0';
   signal stop_control : std_logic := '0';
   signal stop_address : std_logic_vector(9 downto 0) := (others => '0');
   signal trigger_control : std_logic := '0';
   signal trigger_rate_control : std_logic := '0';
   signal trigger_rate : std_logic_vector(3 downto 0) := (others => '0');
   signal trigger_delay : std_logic_vector (15 DOWNTO 0) := (others => '0');
   signal ex_rst_veto   : std_logic := '0';

   --Outputs
   signal trigger_out : std_logic;
   signal MARKER_OUT : std_logic;
   signal TM_CLK : std_logic;
   signal TM_RST : std_logic;
   signal TM_START : std_logic;
   signal TM_SPEAK : std_logic;
   signal ex_rst : std_logic;

   -- Clock period definitions
   constant clk_period : time := 10 ns;
 
BEGIN
 
   -- Instantiate the Unit Under Test (UUT)
   uut: topmetal_simple PORT MAP (
          rst => rst,
          clk => clk,
          SWG => SWG,
          BTN => BTN,
          MARKER_IN => MARKER_IN,
          MARKER_OUT => MARKER_OUT,
          stop_control => stop_control,
          stop_address => stop_address,
          trigger_control => trigger_control,
          trigger_rate_control => trigger_rate_control,
          trigger_rate => trigger_rate,
          trigger_delay => trigger_delay,
          trigger_out   => trigger_out,
          TM_CLK => TM_CLK,
          TM_RST => TM_RST,
          TM_START => TM_START,
          TM_SPEAK => TM_SPEAK,
          ex_rst => ex_rst,
          ex_rst_veto => ex_rst_veto
        );

   -- Clock process definitions
   clk_process :process
   begin
     clk <= '0';
     wait for clk_period/2;
     clk <= '1';
     wait for clk_period/2;
   end process;
 
   -- Stimulus process
   stim_proc: process
   begin		
      -- hold reset state for 100 ns.
     rst <= '1';
     SWG <= x"00";
     BTN <= "0000000";
     stop_control <= '0';
     stop_address <= (OTHERS => '0');
     trigger_control <= '0';
     trigger_rate_control <= '1';
     trigger_rate <= x"0";
     trigger_delay <= x"0001";
     wait for 100 ns;	
     rst <= '0';
     wait for clk_period*10;
     BTN(0) <= '1';
     wait for clk_period*10;
     BTN(0) <= '0';
     -- insert stimulus here 

     wait;
   end process;

END;
