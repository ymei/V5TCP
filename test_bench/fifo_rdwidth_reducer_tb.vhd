--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   01:40:13 06/29/2015
-- Design Name:   
-- Module Name:   test_bench/fifo_rdwidth_reducer_tb.vhd
-- Project Name:  V5TCP
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: fifo_rdwidth_reducer
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
 
ENTITY fifo_rdwidth_reducer_tb IS
END fifo_rdwidth_reducer_tb;
 
ARCHITECTURE behavior OF fifo_rdwidth_reducer_tb IS

  -- Component Declaration for the Unit Under Test (UUT)

  COMPONENT fifo_rdwidth_reducer
    GENERIC (
      RDWIDTH : positive := 32;
      RDRATIO : positive := 3
    );
    PORT(
      RESET : IN  std_logic;
      CLK   : IN  std_logic;
      DIN   : IN  std_logic_vector(95 DOWNTO 0);
      VALID : IN  std_logic;
      RDREQ : OUT std_logic;
      DOUT  : OUT std_logic_vector(31 DOWNTO 0);
      EMPTY : OUT std_logic;
      RD_EN : IN  std_logic
    );
  END COMPONENT;

  COMPONENT fifo16to32
    PORT (
      RST    : IN  std_logic;
      WR_CLK : IN  std_logic;
      RD_CLK : IN  std_logic;
      DIN    : IN  std_logic_vector(15 DOWNTO 0);
      WR_EN  : IN  std_logic;
      RD_EN  : IN  std_logic;
      DOUT   : OUT std_logic_vector(31 DOWNTO 0);
      FULL   : OUT std_logic;
      EMPTY  : OUT std_logic
    );
  END COMPONENT;

  --Inputs
  SIGNAL RESET : std_logic                     := '0';
  SIGNAL CLK   : std_logic                     := '0';
  SIGNAL DIN   : std_logic_vector(95 DOWNTO 0) := (OTHERS => '0');
  SIGNAL VALID : std_logic                     := '0';
  SIGNAL RD_EN : std_logic                     := '0';

  --Outputs
  SIGNAL RDREQ : std_logic;
  SIGNAL DOUT  : std_logic_vector(31 DOWNTO 0);
  SIGNAL EMPTY : std_logic;

  -- internals
  SIGNAL fifo_din   : std_logic_vector(15 DOWNTO 0) := (OTHERS => '0');
  SIGNAL fifo_dout  : std_logic_vector(31 DOWNTO 0) := (OTHERS => '0');
  SIGNAL fifo_wr_en : std_logic                     := '0';
  SIGNAL fifo_rd_en : std_logic;
  SIGNAL fifo_full  : std_logic;
  SIGNAL fifo_empty : std_logic;

  -- Clock period definitions
  CONSTANT CLK_period : time := 10 ns;

BEGIN

  -- Instantiate the Unit Under Test (UUT)
  uut : fifo_rdwidth_reducer
    GENERIC MAP (
      RDWIDTH => 32,
      RDRATIO => 3
    )
    PORT MAP (
      RESET => RESET,
      CLK   => CLK,
      DIN   => DIN,
      VALID => VALID,
      RDREQ => RDREQ,
      DOUT  => DOUT,
      EMPTY => EMPTY,
      RD_EN => RD_EN
    );

  fifo : fifo16to32
    PORT MAP (
      RST    => RESET,
      WR_CLK => CLK,
      RD_CLK => CLK,
      DIN    => fifo_din,
      WR_EN  => fifo_wr_en,
      RD_EN  => fifo_rd_en,
      DOUT   => fifo_dout,
      FULL   => fifo_full,
      EMPTY  => fifo_empty
    );
  VALID <= NOT fifo_empty;
  fifo_rd_en <= RDREQ;
  DIN   <= fifo_dout & (NOT fifo_dout) & fifo_dout;

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
    WAIT FOR 25ns;
    -- hold reset state for 80 ns.
    RESET <= '1';
    WAIT FOR 80 ns;
    RESET <= '0';
    WAIT FOR CLK_period*12;

    -- insert stimulus here
    WAIT FOR 10ps;
    fifo_din <= x"505a";
    fifo_wr_en <= '1';
    WAIT FOR CLK_period*1;
    fifo_din <= x"a505";
    WAIT FOR CLK_period;
    fifo_din <= x"5a50";
    WAIT FOR CLK_period*1;
    fifo_din <= x"05a5";
    WAIT FOR CLK_period*1;
    fifo_wr_en <= '0';
    WAIT FOR CLK_period*10;
    RD_EN <= '1';
    WAIT FOR CLK_period*2;
    RD_EN <= '0';
    WAIT FOR CLK_period*2;
    RD_EN <= '1';
    WAIT FOR CLK_period*3;
    RD_EN <= '0';
    WAIT FOR CLK_period*4;
    RD_EN <= '1';
    --
    WAIT FOR CLK_period*2;
    fifo_din <= x"505a";
    fifo_wr_en <= '1';
    WAIT FOR CLK_period*1;
    fifo_din <= x"a505";
    WAIT FOR CLK_period;
    fifo_din <= x"5a50";
    WAIT FOR CLK_period*1;
    fifo_din <= x"05a5";
    WAIT FOR CLK_period*1;
    fifo_wr_en <= '0';

    WAIT;
  END PROCESS;

END;
