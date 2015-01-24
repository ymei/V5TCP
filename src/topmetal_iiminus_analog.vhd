----------------------------------------------------------------------------------
-- Company: 
-- Engineer: Yuan Mei
-- 
-- Create Date:    23:56:58 1/19/2015
-- Design Name:    Topmetal II- analog scan driver
-- Module Name:    topmetal_iiminux_analog - Behavioral
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

ENTITY topmetal_iiminus_analog IS
  GENERIC (
    ROWS          : positive := 72;     -- number of ROWS in the array
    COLS          : positive := 72;     -- number of COLS in the ARRAY
    CLK_DIV_WIDTH : positive := 16;
    CONFIG_WIDTH  : positive := 16
  );
  PORT (
    CLK           : IN  std_logic;      -- clock, TM_CLK_S is derived from this one
    RESET         : IN  std_logic;      -- reset
    -- data input for writing to in-chip SRAM
    MEM_CLK       : IN  std_logic;      -- connect to control_interface
    MEM_WE        : IN  std_logic;
    MEM_ADDR      : IN  std_logic_vector(31 DOWNTO 0);
    MEM_DIN       : IN  std_logic_vector(31 DOWNTO 0);
    SRAM_WR_START : IN  std_logic;  -- 1 MEM_CLK wide pulse to initiate in-chip SRAM write
    -- configuration
    CLK_DIV       : IN  std_logic_vector(3 DOWNTO 0);  -- log2(CLK_DIV_WIDTH), CLK/(2**CLK_DIV)
    STOP_ADDR     : IN  std_logic_vector(CONFIG_WIDTH-1 DOWNTO 0);  --MSB enables
    TRIGGER_RATE  : IN  std_logic_vector(CONFIG_WIDTH-1 DOWNTO 0);  --trigger every () frames
    TRIGGER_DELAY : IN  std_logic_vector(CONFIG_WIDTH-1 DOWNTO 0);
    -- input
    MARKER_A      : IN  std_logic;
    -- output
    TRIGGER_OUT   : OUT std_logic;
    --
    SRAM_D        : OUT std_logic_vector(4 DOWNTO 0);
    SRAM_WE       : OUT std_logic;
    TM_RST        : OUT std_logic;      -- digital reset
    TM_CLK_S      : OUT std_logic;
    TM_RST_S      : OUT std_logic;
    TM_START_S    : OUT std_logic;
    TM_SPEAK_S    : OUT std_logic
 );
END topmetal_iiminus_analog;

ARCHITECTURE Behavioral OF topmetal_iiminus_analog IS

  COMPONENT bram_sdp_w32r8
    PORT (
      CLKA  : IN  std_logic;
      WEA   : IN  std_logic_vector(0 DOWNTO 0);
      ADDRA : IN  std_logic_vector(10 DOWNTO 0);
      DINA  : IN  std_logic_vector(31 DOWNTO 0);
      CLKB  : IN  std_logic;
      ADDRB : IN  std_logic_vector(12 DOWNTO 0);
      DOUTB : OUT std_logic_vector(7 DOWNTO 0)
    );
  END COMPONENT;
  COMPONENT pulse2pulse
    PORT (
      IN_CLK   : IN  std_logic;
      OUT_CLK  : IN  std_logic;
      RST      : IN  std_logic;
      PULSEIN  : IN  std_logic;
      INBUSY   : OUT std_logic;
      PULSEOUT : OUT std_logic
    );
  END COMPONENT;

  SIGNAL clk_cnt         : unsigned(CLK_DIV_WIDTH-1 DOWNTO 0);
  SIGNAL TM_CLK_buf      : std_logic;
  SIGNAL TM_RST_S_buf    : std_logic;
  SIGNAL TM_RST_S_buf1   : std_logic;
  SIGNAL TM_START_S_buf  : std_logic;
  SIGNAL TM_START_S_buf1 : std_logic;
  SIGNAL TM_SPEAK_S_buf  : std_logic;
  SIGNAL TM_SPEAK_S_buf1 : std_logic;
  SIGNAL sram_wr_startp  : std_logic;
  SIGNAL sram_wr_clk     : std_logic;
  SIGNAL sram_writing    : std_logic;
  --
  SIGNAL bram_wea        : std_logic_vector(0 DOWNTO 0);
  SIGNAL bram_outb       : std_logic_vector(7 DOWNTO 0);
  SIGNAL bram_addrb      : std_logic_vector(12 DOWNTO 0);
  SIGNAL bram_addrb_cnt  : unsigned(bram_addrb'length-1 DOWNTO 0);
  --
  SIGNAL pxladdr         : unsigned(bram_addrb'length-1 DOWNTO 0);
  SIGNAL trigger_cnt     : unsigned(TRIGGER_RATE'length-1 DOWNTO 0);
  SIGNAL trigger_buf     : std_logic;
  --
  TYPE driveState_t IS (S0, S1, S2, S3, S4, S5, S6, S7);
  SIGNAL driveState      : driveState_t;
  SIGNAL driveState1     : driveState_t;

BEGIN 

  bram_inst : bram_sdp_w32r8
    PORT MAP (
      CLKA  => MEM_CLK,
      WEA   => bram_wea,
      ADDRA => MEM_ADDR(10 DOWNTO 0),
      DINA  => MEM_DIN,
      CLKB  => CLK,
      ADDRB => bram_addrb(12 DOWNTO 0),
      DOUTB => bram_outb
    );
  bram_wea <= (OTHERS => MEM_WE);

  pulse2pulse_inst : pulse2pulse
    PORT MAP (
      IN_CLK   => MEM_CLK,
      OUT_CLK  => CLK,
      RST      => RESET,
      PULSEIN  => SRAM_WR_START,
      INBUSY   => OPEN,
      PULSEOUT => sram_wr_startp
    );

  clk_proc: PROCESS (CLK, RESET)
  BEGIN
    IF RESET = '1' THEN
      clk_cnt <= to_unsigned(0, clk_cnt'length);
    ELSIF rising_edge(CLK) THEN
      clk_cnt <= clk_cnt + 1;
    END IF;
  END PROCESS clk_proc;
  TM_CLK_buf <= CLK WHEN to_integer(unsigned(CLK_DIV)) = 0 ELSE
                clk_cnt(to_integer(unsigned(CLK_DIV))-1);

  tm_proc : PROCESS (TM_CLK_buf, RESET)
    VARIABLE stopped : std_logic;
  BEGIN
    IF RESET = '1' OR sram_writing = '1' THEN 
      TM_RST_S_buf   <= '0';
      TM_START_S_buf <= '0';
      TM_SPEAK_S_buf <= '0';
      trigger_buf    <= '0';
      pxladdr        <= (OTHERS => '0');
      trigger_cnt    <= (OTHERS => '0');
      stopped        := '0';
      driveState     <= S0;
    ELSIF falling_edge(TM_CLK_buf) THEN
      TM_RST_S_buf   <= '0';
      TM_START_S_buf <= '0';
      TM_SPEAK_S_buf <= '0';
      trigger_buf    <= '0';      
      CASE driveState IS
        WHEN S0 =>                      -- wait a clk cycle
          driveState <= S1;
        WHEN S1 =>
          TM_RST_S_buf <= '1';
          driveState   <= S2;
        WHEN S2 =>                      -- wait a clk cycle then start
          driveState <= S3;
        WHEN S3 =>
          TM_START_S_buf <= '1';
          pxladdr        <= (OTHERS => '1');
          driveState     <= S4;
        WHEN S4 =>
          IF STOP_ADDR(STOP_ADDR'length-1) = '0' THEN 
            TM_SPEAK_S_buf <= '1';
          ELSIF stopped = '1' THEN
            TM_SPEAK_S_buf <= '0';
          ELSE
            TM_SPEAK_S_buf <= '1';
            IF pxladdr = unsigned(STOP_ADDR(pxladdr'length-1 DOWNTO 0)) THEN
              TM_SPEAK_S_buf <= '0';
              stopped        := '1';
            END IF;
          END IF;
          IF trigger_cnt = unsigned(TRIGGER_RATE) AND 
                (pxladdr = unsigned(TRIGGER_DELAY(pxladdr'length-1 DOWNTO 0))-1 OR 
                  (pxladdr = to_unsigned(ROWS*COLS-1, pxladdr'length) AND 
                   unsigned(TRIGGER_DELAY(pxladdr'length-1 DOWNTO 0)) = 0)) THEN
            trigger_buf <= '1';
          END IF;
          IF pxladdr >= to_unsigned(ROWS*COLS-1, pxladdr'length) THEN
            pxladdr <= (OTHERS => '0');
            IF trigger_cnt >= unsigned(TRIGGER_RATE) THEN
              trigger_cnt <= (OTHERS => '0');
            ELSE
              trigger_cnt <= trigger_cnt + 1;
            END IF;
          ELSE
            pxladdr <= pxladdr + 1;
          END IF;
          driveState <= S4;
        WHEN OTHERS =>
          driveState <= S0;
      END CASE;
    END IF;
  END PROCESS;

  sram_write_proc : PROCESS (CLK, RESET)
  BEGIN
    IF RESET = '1' THEN
      bram_addrb_cnt  <= to_unsigned(0, bram_addrb_cnt'length);
      sram_writing    <= '0';
      TM_RST_S_buf1   <= '0';
      TM_START_S_buf1 <= '0';
      TM_SPEAK_S_buf1 <= '0';
      SRAM_WE         <= '0';
      driveState1     <= S0;
    ELSIF rising_edge(CLK) THEN
      TM_RST_S_buf1   <= '0';
      TM_START_S_buf1 <= '0';
      TM_SPEAK_S_buf1 <= '0';
      SRAM_WE         <= '0';
      CASE driveState1 IS
        WHEN S0 =>
          IF sram_wr_startp = '1' THEN
            sram_writing <= '1';
          END IF;
          IF sram_writing = '1' AND clk_cnt(1 DOWNTO 0) = "11" THEN  -- falling edge
            TM_RST_S_buf1 <= '1';
            driveState1   <= S1;
          END IF;
        WHEN S1 =>
          TM_RST_S_buf1 <= '1';
          IF clk_cnt(1 DOWNTO 0) = "11" THEN
            TM_RST_S_buf1 <= '0';
            driveState1   <= S2;
          END IF;
        WHEN S2 =>                      -- wait a clk cycle then start
          IF clk_cnt(1 DOWNTO 0) = "11" THEN
            TM_START_S_buf1 <= '1';
            driveState1     <= S3;
          END IF;
        WHEN S3 =>
          TM_START_S_buf1 <= '1';
          IF clk_cnt(1 DOWNTO 0) = "11" THEN
            TM_START_S_buf1 <= '0';
            bram_addrb_cnt  <= (OTHERS => '1');
            driveState1     <= S4;
          END IF;
        WHEN S4 =>                      -- wait a clk cycle then speak
          IF clk_cnt(1 DOWNTO 0) = "11" THEN
            TM_SPEAK_S_buf1 <= '1';
            driveState1     <= S5;
          END IF;
        WHEN S5 =>
          TM_SPEAK_S_buf1 <= '1';
          IF clk_cnt(1 DOWNTO 0) = "11" THEN  -- tune this value to change the phase of WE
            SRAM_WE <= '1';
          ELSIF clk_cnt(1 DOWNTO 0) = "00" THEN -- tune this value to change the phase of WE
            SRAM_WE <= '1';
            IF bram_addrb_cnt = to_unsigned(ROWS*COLS-1, bram_addrb_cnt'length) THEN
              driveState1 <= S6;
            END IF;
          END IF;
          IF clk_cnt(1 DOWNTO 0) = "01" THEN -- tune this value to change the phase of bram_d
            bram_addrb_cnt <= bram_addrb_cnt + 1;              
          END IF;
        WHEN S6 =>
          TM_SPEAK_S_buf1 <= '1';
          driveState1     <= S7;
        WHEN OTHERS =>
          TM_SPEAK_S_buf1 <= '0';
          sram_writing    <= '0';
          driveState1     <= S0;
      END CASE;
    END IF;
  END PROCESS;
  sram_wr_clk <= clk_cnt(1);
  bram_addrb  <= std_logic_vector(bram_addrb_cnt);

  -- output
  TM_RST      <= RESET;
  TM_CLK_S    <= sram_wr_clk     WHEN sram_writing = '1' ELSE TM_CLK_buf;
  TM_RST_S    <= TM_RST_S_buf1   WHEN sram_writing = '1' ELSE TM_RST_S_buf;
  TM_START_S  <= TM_START_S_buf1 WHEN sram_writing = '1' ELSE TM_START_S_buf;
  TM_SPEAK_S  <= TM_SPEAK_S_buf1 WHEN sram_writing = '1' ELSE TM_SPEAK_S_buf;
  TRIGGER_OUT <= trigger_buf;
  --
  SRAM_D      <= bram_outb(SRAM_D'length-1 DOWNTO 0);

END Behavioral;
