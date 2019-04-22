--------------------------------------------------------------------------------
--! @file channel_decim.vhd
--! @brief Average and decimate data in a group of channels.
--! @author Yuan Mei
--!
--! This version assumes the input is unsigned.
--! offset counts 2 CLK after TRIG is seen.
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

ENTITY channel_decim IS
  GENERIC (
    NCH            : positive := 8;
    OUTCH_WIDTH    : positive := 16;
    INTERNAL_WIDTH : positive := 32;
    INDATA_WIDTH   : positive := 128;
    OUTDATA_WIDTH  : positive := 128
  );
  PORT (
    RESET           : IN  std_logic;
    CLK             : IN  std_logic;
    -- high 4-bit is stride, middle 4-bit is offset,
    -- low 4-bit is number of samples to average.
    CONFIG          : IN  std_logic_vector(11 DOWNTO 0);
    -- trig zeros the offset calculation
    TRIG            : IN  std_logic;
    INDATA_Q        : IN  std_logic_vector(INDATA_WIDTH-1 DOWNTO 0);
    OUTVALID        : OUT std_logic;
    OUTDATA_Q       : OUT std_logic_vector(OUTDATA_WIDTH-1 DOWNTO 0)
  );
END channel_decim;

ARCHITECTURE Behavioral OF channel_decim IS

  SIGNAL trig_prev   : std_logic;
  SIGNAL trig_prev1  : std_logic;
  SIGNAL trig_prev2  : std_logic;
  SIGNAL trig_synced : std_logic;
  --
  SIGNAL stride_n    : natural;
  SIGNAL offset_n    : natural;
  SIGNAL avg_n       : natural;
  --
  TYPE INTERNALVAL IS ARRAY(NCH-1 DOWNTO 0) OF unsigned(INTERNAL_WIDTH-1 DOWNTO 0);
  SIGNAL inch_val     : INTERNALVAL;
  SIGNAL internal_val : INTERNALVAL;

BEGIN

  stride_n <= to_integer(unsigned(CONFIG(11 DOWNTO 8)));
  offset_n <= to_integer(unsigned(CONFIG(7 DOWNTO 4)));
  avg_n    <= to_integer(unsigned(CONFIG(3 DOWNTO 0)));
  PROCESS (CLK) IS
    VARIABLE i : integer;
  BEGIN
    IF falling_edge(CLK) THEN  -- register half-cycle earlier
      FOR i IN 0 TO NCH-1 LOOP
        inch_val(i) <= resize(unsigned(INDATA_Q(16*(i+1)-1 DOWNTO 16*i)), INTERNAL_WIDTH);
      END LOOP;
    END IF;
  END PROCESS;

  -- capture the rising edge of trigger
  PROCESS (CLK, RESET) IS
  BEGIN
    IF RESET = '1' THEN
      trig_prev   <= '0';
      trig_prev1  <= '0';
      trig_prev2  <= '0';
    ELSIF rising_edge(CLK) THEN
      trig_prev   <= TRIG;
      trig_prev1  <= trig_prev;
      trig_prev2  <= trig_prev1;
    END IF;
  END PROCESS;
  trig_synced <= '1' WHEN trig_prev2 = '0' AND trig_prev1 = '1' ELSE '0';

  PROCESS (CLK, RESET) IS
    VARIABLE i : integer;
    VARIABLE j : unsigned(INTERNAL_WIDTH-1 DOWNTO 0);
  BEGIN
    IF RESET = '1' THEN
      FOR i IN 0 TO NCH-1 LOOP
        internal_val(i) <= (OTHERS => '0');
      END LOOP;
      OUTVALID <= '0';
      j        := (OTHERS => '0');
    ELSIF rising_edge(CLK) THEN
      OUTVALID <= '0';
      IF trig_synced = '1' THEN
        j := to_unsigned(0, j'length);
        OUTVALID <= '0';
      END IF;
      IF j = to_unsigned(offset_n, j'length) THEN
        FOR i IN 0 TO NCH-1 LOOP
          internal_val(i) <= inch_val(i);
        END LOOP;
      END IF;
      IF j>to_unsigned(offset_n, j'length) AND j<to_unsigned(offset_n+avg_n, j'length) THEN
        FOR i IN 0 TO NCH-1 LOOP
          internal_val(i) <= internal_val(i) + inch_val(i);
        END LOOP;
      END IF;
      j := j + 1;
      IF stride_n = 0 OR j = stride_n THEN
        j        := to_unsigned(0, j'length);
        OUTVALID <= '1';
      END IF;
    END IF;
  END PROCESS;

  outdata_q_inst : FOR i IN 0 TO NCH-1 GENERATE
    OUTDATA_Q(OUTCH_WIDTH*(i+1)-1 DOWNTO OUTCH_WIDTH*i) <=
        std_logic_vector(internal_val(i)(OUTCH_WIDTH-1 DOWNTO 0));
  END GENERATE;

END Behavioral;
