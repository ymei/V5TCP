----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    12:56:58 09/29/2013 
-- Design Name: 
-- Module Name:    topmetal_simple - Behavioral 
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
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
LIBRARY UNISIM;
USE UNISIM.VComponents.ALL;

ENTITY topmetal_simple IS
  GENERIC (
    TRIGGER_DELAY_WIDTH  : positive := 16
  );
  PORT (
    rst                  : IN  std_logic;
    clk                  : IN  std_logic;
    SWG                  : IN  std_logic_vector (7 DOWNTO 0);
    BTN                  : IN  std_logic_vector (6 DOWNTO 0);
    MARKER_IN            : IN  std_logic;  -- process MARKER from TopMetal
    MARKER_OUT           : OUT std_logic;
    stop_control         : IN  std_logic;
    stop_address         : IN  std_logic_vector (9 DOWNTO 0);
    trigger_control      : IN  std_logic;
    trigger_rate_control : IN  std_logic;
    trigger_rate         : IN  std_logic_vector (3 DOWNTO 0);
    trigger_delay        : IN  std_logic_vector (TRIGGER_DELAY_WIDTH-1 DOWNTO 0);
    trigger_out          : OUT std_logic;
    TM_CLK               : OUT std_logic;
    TM_RST               : OUT std_logic;
    TM_START             : OUT std_logic;
    TM_SPEAK             : OUT std_logic;
    ex_rst               : OUT std_logic;
    ex_rst_veto          : IN  std_logic
  );
END topmetal_simple;

ARCHITECTURE Behavioral OF topmetal_simple IS

  SIGNAL clk_cnt             : std_logic_vector(15 DOWNTO 0);
  SIGNAL TM_CLK_cnt          : std_logic_vector(15 DOWNTO 0);
  SIGNAL frame_cnt           : std_logic_vector(15 DOWNTO 0);
  SIGNAL ext_rst_cnt         : std_logic_vector(15 DOWNTO 0);
  SIGNAL BTN_buf             : std_logic_vector(6 DOWNTO 0);
  SIGNAL TM_CLK_buf          : std_logic;
  SIGNAL frame_trg_buf       : std_logic;
  SIGNAL frame_trg           : std_logic;
  SIGNAL ex_rst_buf          : std_logic;
  SIGNAL started             : std_logic;
  SIGNAL marker_cnt          : std_logic_vector(15 DOWNTO 0);
  SIGNAL marker_cnt_buf      : std_logic_vector(15 DOWNTO 0);
  SIGNAL MARKER_IN_buf       : std_logic;
  SIGNAL trigger_control_buf : std_logic;
  SIGNAL trigger_set         : std_logic;
  SIGNAL marker_buf_ext      : std_logic;
  SIGNAL marker_buf_ext_half : std_logic;
  SIGNAL trigger_delay_cnt   : std_logic_vector(TRIGGER_DELAY_WIDTH-1 DOWNTO 0);

BEGIN

  ex_rst <= (NOT ex_rst_buf) OR ex_rst_veto;

  PROCESS(rst, clk)
  BEGIN
    IF rst = '1' then
      clk_cnt <= (OTHERS => '0');
    ELSIF clk'event AND clk = '1' then
      clk_cnt <= clk_cnt+1;
    END IF;
  END PROCESS;

--10MHz / 2^(SWG(3 downto 0))
  TM_CLK_buf <= clk_cnt(conv_integer(SWG(3 DOWNTO 0))) WHEN stop_control = '0' or stop_address /= TM_CLK_cnt(11 downto 2);
--SWG low 4 bit for 2*clks per TM_CLK
  TM_CLK     <= TM_CLK_buf;

  PROCESS(rst, TM_CLK_buf)
  BEGIN
    IF rst = '1' then

      TM_CLK_cnt <= (OTHERS => '0');
      frame_cnt  <= (OTHERS => '0');
      TM_START   <= '0';
      TM_SPEAK   <= '0';
      TM_RST     <= '0';
      started    <= '0';

      BTN_buf <= (OTHERS => '0');

    ELSIF TM_CLK_buf'event AND TM_CLK_buf = '1' then
      TM_CLK_cnt <= TM_CLK_cnt+1;
      IF conv_integer(TM_CLK_cnt) = 64*64-1 THEN
        TM_CLK_cnt <= (OTHERS => '0');
      END IF;
      IF conv_integer(TM_CLK_cnt) = 63-2-2 THEN  -- push frame_cnt hence ex_rst earlier
        frame_cnt <= frame_cnt+1;
      END IF;
      BTN_buf <= BTN;
      IF BTN_buf(0) = '0' and BTN(0) = '1' THEN  --issue start at rising edge of BTN(0)
        started    <= '1';
        TM_START   <= '0';
        TM_SPEAK   <= '0';
        TM_RST     <= '0';
        TM_CLK_cnt <= (OTHERS => '0');
        frame_cnt  <= (OTHERS => '0');
      ELSIF conv_integer(TM_CLK_cnt) = 16 AND started = '1' then
        TM_RST <= '1';
      ELSIF conv_integer(TM_CLK_cnt) = 32 AND started = '1' then
        TM_RST <= '0';
      ELSIF conv_integer(TM_CLK_cnt) = 48 AND started = '1' then
        TM_START <= '1';
      ELSIF conv_integer(TM_CLK_cnt) = 64 AND started = '1' then
        TM_START <= '0';
        TM_SPEAK <= '1';
        started  <= '0';
      END IF;

      IF BTN_buf(1) = '0' and BTN(1) = '1' THEN  -- set speak to 0 at falling edge of BTN(1)
        TM_SPEAK <= '0';
      END IF;

    END IF;
  END PROCESS;

--ex_rst
  
  PROCESS(rst, TM_CLK_buf)
  BEGIN
    IF rst = '1' then
      frame_trg_buf <= '0';
      frame_trg     <= '0';
      ext_rst_cnt   <= (OTHERS => '0');
      ex_rst_buf    <= '0';
    ELSIF TM_CLK_buf'event AND TM_CLK_buf = '1' THEN 
      frame_trg_buf <= frame_cnt(conv_integer(SWG(7 DOWNTO 4)));
      IF frame_trg_buf = '0' and frame_cnt(conv_integer(SWG(7 downto 4))) = '1' THEN  --SWG high 4 bit for frames per ext_rst
        IF BTN(6) = '0' then  -- disable ext_reset by set BTN(6)=1
          frame_trg <= '1';
        END IF;
        ext_rst_cnt <= (OTHERS => '0');
        ex_rst_buf  <= '0';
      ELSIF frame_trg = '1' then
        ext_rst_cnt <= ext_rst_cnt+1;
        ex_rst_buf  <= '1';
        IF conv_integer(ext_rst_cnt) >= 64*64+4 THEN  -- ext_reset 4096 * TM_CLK cycle
          frame_trg <= '0';
        END IF;
      ELSE
        ex_rst_buf <= '0';
      END IF;
    END IF;
  END PROCESS;

--trigger 

  MARKER_OUT <= marker_buf_ext;

  PROCESS(rst, TM_CLK_buf)
    VARIABLE triggered : std_logic := '0';
  BEGIN
    IF rst = '1' then
      marker_buf_ext    <= '0';
      marker_cnt        <= (OTHERS => '0');
      MARKER_IN_buf     <= '0';
      trigger_set       <= '0';
      trigger_delay_cnt <= (OTHERS => '0');
      trigger_out       <= '0';
      triggered         := '0';
    ELSIF TM_CLK_buf'event AND TM_CLK_buf = '1' then
      MARKER_IN_buf       <= ex_rst_buf;
      trigger_control_buf <= trigger_control;
      marker_cnt_buf      <= marker_cnt;
      IF trigger_control_buf = '0' and trigger_control = '1' THEN
        trigger_set <= '1';
      END IF;
      IF trigger_rate_control = '1' and marker_cnt_buf(conv_integer(trigger_rate)) = '0' AND marker_cnt(conv_integer(trigger_rate)) = '1' then
        trigger_set <= '1';
      END IF;
      IF MARKER_IN_buf = '0' and ex_rst_buf = '1' THEN
        marker_cnt <= marker_cnt+1;
        IF trigger_set = '1' THEN
          marker_buf_ext    <= '1';
          trigger_set       <= '0';
          triggered         := '1';
          trigger_delay_cnt <= (OTHERS => '0');
        ELSE
          marker_buf_ext <= '0';
        END IF;
      END IF;
      IF triggered = '1' THEN
        trigger_delay_cnt <= trigger_delay_cnt+1;
        IF trigger_delay_cnt = trigger_delay THEN
          trigger_out <= '1';
          triggered   := '0';
        ELSE
          trigger_out <= '0';
        END IF;
      ELSE
        trigger_out       <= '0';
        trigger_delay_cnt <= (OTHERS => '0');
      END IF;
    END IF;
  END PROCESS;




END Behavioral;
