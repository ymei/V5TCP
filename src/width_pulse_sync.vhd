--------------------------------------------------------------------------------
--! @file width_pulse_sync.vhd
--! @brief Produce a pulse of specified width in a different clock domain.
--! @author Yuan Mei
--!
--! Produce a pulse of specified width in a different clock domain
--! following a 1-CLKO wide reset pulse (when RSTO_EN='1').
--! Ideally suited to change iodelay taps and iserdes bitslip
--!               MODE := 0   output one pulse of duration PW
--!                    := 1   a train of 1-clk wide pulses (of number PW)
--! DELAY_AFTER_RST sets the time after RSTO AND before PO.  1 is the minimum.
--! PW = 0 will still generate RSTO but no PO.
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
USE ieee.numeric_std.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
LIBRARY UNISIM;
USE UNISIM.VComponents.ALL;

ENTITY width_pulse_sync IS
  GENERIC (
    DATA_WIDTH : positive := 6;
    MODE       : natural  := 0;         -- 0: output one pulse of duration PW
                                        -- 1: a train of 1-clk wide pulses (of number PW)
    RSTO_EN    : std_logic := '1';      -- RSTO output enable
    DELAY_AFTER_RST : natural := 6      -- pulse starts () CLK after RSTO
  );
  PORT (
    RESET : IN  std_logic;
    CLK   : IN  std_logic;
    PW    : IN  std_logic_vector(DATA_WIDTH-1 DOWNTO 0);
    START : IN  std_logic;              -- should be synchronous to CLK, of any width
    BUSY  : OUT std_logic;
    --
    CLKO  : IN  std_logic;
    RSTO  : OUT std_logic;
    PO    : OUT std_logic
  );
END width_pulse_sync;

ARCHITECTURE Behavioral OF width_pulse_sync IS

  SIGNAL prev       : std_logic;
  SIGNAL prevb      : std_logic;
  SIGNAL prevb1     : std_logic;
  SIGNAL prevo      : std_logic;
  SIGNAL prevo1     : std_logic;
  SIGNAL busy_buf   : std_logic;
  SIGNAL busy_bufo  : std_logic;
  SIGNAL pw_buf     : std_logic_vector(DATA_WIDTH-1 DOWNTO 0);
  SIGNAL po_buf     : std_logic;
  SIGNAL rsto_buf   : std_logic;

BEGIN

  PROCESS (CLK, RESET) IS
  BEGIN
    IF RESET = '1' THEN
      prev     <= '0';
      busy_buf <= '0';
      pw_buf   <= (OTHERS => '0');
      prevb    <= '0';
      prevb1   <= '0';
    ELSIF rising_edge(CLK) THEN
      prev <= START;
      -- Capture the rising edge of START, which is synchronous to CLK, of any width
      IF (prev = '0' AND START = '1' AND prevb1 = '0') THEN
        busy_buf <= '1';
        pw_buf   <= PW;
      END IF;
      prevb  <= busy_bufo;
      prevb1 <= prevb;
      -- Capture the falling edge of busy_bufo
      IF (prevb = '0' AND prevb1 = '1') THEN
        busy_buf <= '0';
      END IF;
    END IF;
  END PROCESS;
  BUSY <= busy_buf;

  -- output clock domain
  PROCESS (CLKO, RESET) IS
    VARIABLE counter, rcnt : unsigned(DATA_WIDTH DOWNTO 0);
  BEGIN
    IF RESET = '1' THEN
      counter   := (OTHERS => '1');
      rcnt      := (0=>'1', OTHERS => '0');
      prevo     <= '0';
      prevo1    <= '0';
      busy_bufo <= '0';
      po_buf    <= '0';
      rsto_buf  <= '0';
    ELSIF rising_edge(CLKO) THEN
      prevo     <= busy_buf;
      prevo1    <= prevo;
      busy_bufo <= '0';
      po_buf    <= '0';
      rsto_buf  <= '0';
      -- Capture the rising edge of busy_buf
      IF (prevo1 = '0' AND prevo = '1') THEN
        busy_bufo <= '1';
        counter   := (OTHERS => '0');
        rcnt      := (0=>'1', OTHERS => '0');
        rsto_buf  <= '1';
      ELSIF counter = 0 THEN
        busy_bufo <= '1';
        IF rcnt < to_unsigned(DELAY_AFTER_RST, rcnt'length) THEN
          rcnt := rcnt + 1;
        ELSE
          counter := counter + 1;
        END IF;
      ELSIF counter <= unsigned(pw_buf) THEN
        busy_bufo <= '1';
        IF MODE = 0 THEN
          counter := counter + 1;
          po_buf  <= '1';
        ELSIF MODE = 1 THEN
          IF po_buf = '0' THEN
            counter := counter + 1;
          END IF;
          po_buf <= NOT po_buf;
        END IF;
      ELSE
        counter := (OTHERS => '1');
      END IF;
    END IF;
  END PROCESS;
  PO   <= po_buf;
  RSTO <= RESET OR (rsto_buf AND RSTO_EN);
END Behavioral;
