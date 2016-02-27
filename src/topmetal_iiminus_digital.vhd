----------------------------------------------------------------------------------
-- Company: 
-- Engineer: Yuan Mei
-- 
-- Create Date:    23:56:58 2/22/2016
-- Design Name:    Topmetal II- digital readout
-- Module Name:    topmetal_iiminus_digital - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
USE IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
LIBRARY UNISIM;
USE UNISIM.VComponents.ALL;

ENTITY topmetal_iiminus_digital IS
  GENERIC (
    CLK_DIV_WIDTH  : positive := 16;
    DATA_OUT_WIDTH : positive := 32
  );
  PORT (
    CLK         : IN  std_logic;        -- clock to be derived from
    RESET       : IN  std_logic;        -- reset    
    CLK_DIV     : IN  std_logic_vector(3 DOWNTO 0);  -- log2(CLK_DIV_WIDTH), CLK/(2**CLK_DIV)
    --
    CLK_D       : OUT std_logic;        -- clock sent to chip's digital part
    CLK_FB      : IN  std_logic;        -- clock fed back from the chip
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
END topmetal_iiminus_digital;

ARCHITECTURE Behavioral OF topmetal_iiminus_digital IS

  SIGNAL clk_cnt   : unsigned(CLK_DIV_WIDTH-1 DOWNTO 0);
  SIGNAL col_cnt   : unsigned(12 DOWNTO 0);
  SIGNAL clk_buf   : std_logic;
  SIGNAL clk_d_buf : std_logic;
  SIGNAL addr_d    : std_logic_vector(6 DOWNTO 0);
  SIGNAL time_d    : std_logic_vector(9 DOWNTO 0);
  SIGNAL ready_d   : std_logic;
  SIGNAL marker_d  : std_logic;
  SIGNAL data_buf  : std_logic_vector(DATA_OUT_WIDTH-1 DOWNTO 0);
  
BEGIN 

  clk_proc: PROCESS (CLK, RESET)
  BEGIN
    IF RESET = '1' THEN
      clk_cnt <= to_unsigned(0, clk_cnt'length);
    ELSIF rising_edge(CLK) THEN
      clk_cnt <= clk_cnt + 1;
    END IF;
  END PROCESS clk_proc;
  clk_buf <= CLK WHEN to_integer(unsigned(CLK_DIV)) = 0 ELSE
             clk_cnt(to_integer(unsigned(CLK_DIV))-1);

  clk_fwd_inst : ODDR
    GENERIC MAP(
      DDR_CLK_EDGE => "OPPOSITE_EDGE",  -- "OPPOSITE_EDGE" or "SAME_EDGE" 
      INIT         => '0',  -- Initial value for Q port ('1' or '0')
      SRTYPE       => "ASYNC"           -- Reset Type ("ASYNC" or "SYNC")
    )
    PORT MAP (
      Q  => CLK_D,                      -- 1-bit DDR output
      C  => clk_buf,                    -- 1-bit clock input
      CE => '1',                        -- 1-bit clock enable input
      D1 => '1',                        -- 1-bit data input (positive edge)
      D2 => '0',  -- 1-bit data input (negative edge)
      R  => RESET,                      -- 1-bit reset input
      S  => '0'                         -- 1-bit set input
    );

  IBUF_inst : IBUF
    GENERIC MAP (
      IOSTANDARD => "DEFAULT")
    PORT MAP (
      O => clk_d_buf,                   -- Buffer output
      I => CLK_FB      -- Buffer input (connect directly to top-level port)
    );
  ---------------------------------------------<
  ADDR_ibufds_insts : FOR i IN 0 TO 6 GENERATE
    ADDR_ibufds_inst : IBUFDS
      GENERIC MAP (
        DIFF_TERM  => true,
        IOSTANDARD => "DEFAULT"
      )
      PORT MAP (
        O  => addr_d(i),
        I  => ADDR_P(i),
        IB => ADDR_N(i)
      );
  END GENERATE;
  TIME_ibufds_insts : FOR i IN 0 TO 9 GENERATE
    TIME_ibufds_inst : IBUFDS
      GENERIC MAP (
        DIFF_TERM  => true,
        IOSTANDARD => "DEFAULT"
      )
      PORT MAP (
        O  => time_d(i),
        I  => TIME_P(i),
        IB => TIME_N(i)
      );
  END GENERATE;
  ready_ibufds_inst : IBUFDS
    GENERIC MAP (
      DIFF_TERM  => true,
      IOSTANDARD => "DEFAULT"
    )
    PORT MAP (
      O  => ready_d,
      I  => READY_P,
      IB => READY_N
    );
  markerd_ibufds_inst : IBUFDS
    GENERIC MAP (
      DIFF_TERM  => true,
      IOSTANDARD => "DEFAULT"
    )
    PORT MAP (
      O  => marker_d,
      I  => MARKERD_P,
      IB => MARKERD_N
    );
  --------------------------------------------->
  col_proc: PROCESS (clk_d_buf, RESET)
  BEGIN
    IF RESET = '1' THEN
      col_cnt <= to_unsigned(0, col_cnt'length);
    ELSIF rising_edge(clk_d_buf) THEN
      col_cnt               <= col_cnt + 1;
      data_buf(18 DOWNTO 0) <= marker_d & ready_d & time_d & addr_d;
    END IF;
  END PROCESS col_proc;

  DATA_VALID                            <= data_buf(18) OR data_buf(17);
  DATA_CLK                              <= clk_d_buf;
  DATA_OUT(18 DOWNTO 0)                 <= data_buf(18 DOWNTO 0);
  DATA_OUT(DATA_OUT_WIDTH-1 DOWNTO 19)  <= std_logic_vector(col_cnt);

END Behavioral;
