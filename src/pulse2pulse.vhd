--------------------------------------------------------------------------------
--! @file pulse2pulse.vhd
--! @brief Transport a pulse from one clock domain to another
--
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;

ENTITY pulse2pulse IS
  PORT (
    in_clk   : IN  std_logic;
    out_clk  : IN  std_logic;
    rst      : IN  std_logic;
    pulsein  : IN  std_logic;
    inbusy   : OUT std_logic;
    pulseout : OUT std_logic
  );
END pulse2pulse;

ARCHITECTURE syn OF pulse2pulse IS
-----------------------------------------------------------------------------------
--constant declarations
-----------------------------------------------------------------------------------

-----------------------------------------------------------------------------------
--constant declarations
-----------------------------------------------------------------------------------

-----------------------------------------------------------------------------------
--signal declarations
-----------------------------------------------------------------------------------

  SIGNAL out_set        : std_logic;
  SIGNAL out_set_prev   : std_logic;
  SIGNAL out_set_prev2  : std_logic;
  SIGNAL in_set         : std_logic;
  SIGNAL outreset       : std_logic;
  SIGNAL in_reset       : std_logic;
  SIGNAL in_reset_prev  : std_logic;
  SIGNAL in_reset_prev2 : std_logic;

-----------------------------------------------------------------------------------
--component declarations
-----------------------------------------------------------------------------------

--*********************************************************************************
BEGIN
--*********************************************************************************

-----------------------------------------------------------------------------------
--component instantiations
-----------------------------------------------------------------------------------
-----------------------------------------------------------------------------------
--synchronous processes
-----------------------------------------------------------------------------------
  in_proc : PROCESS(in_clk, rst)
  BEGIN
    IF(rst = '1') THEN
      in_reset       <= '0';
      in_reset_prev  <= '0';
      in_reset_prev2 <= '0';
      in_set         <= '0';

    ELSIF(in_clk'event AND in_clk = '1') THEN
      --regitser a pulse on the pulse in port
      --reset the signal when the ouput has registerred the pulse
      IF (in_reset_prev = '1' AND in_reset_prev2 = '1') THEN
        in_set <= '0';
      ELSIF (pulsein = '1') THEN
        in_set <= '1';
      END IF;

      --register the reset signal from the other clock domain
      --three times. double stage synchronising circuit
      --reduces the MTB
      in_reset       <= outreset;
      in_reset_prev  <= in_reset;
      in_reset_prev2 <= in_reset_prev;

    END IF;
  END PROCESS in_proc;

  out_proc : PROCESS(out_clk, rst)
  BEGIN
    IF(rst = '1') THEN
      out_set       <= '0';
      out_set_prev  <= '0';
      out_set_prev2 <= '0';
      outreset      <= '0';
      pulseout      <= '0';
    ELSIF(out_clk'event AND out_clk = '1') THEN
      --generate a pulse on the outpput when the
      --set signal has travelled through the synchronising fip flops
      IF (out_set_prev = '1' AND out_set_prev2 = '0') THEN
        pulseout <= '1';
      ELSE
        pulseout <= '0';
      END IF;

      --feedback the corret reception of the set signal to reset the set pulse
      IF (out_set_prev = '1' AND out_set_prev2 = '1') THEN
        outreset <= '1';
      ELSIF (out_set_prev = '0' AND out_set_prev2 = '0') THEN
        outreset <= '0';
      END IF;

      --register the reset signal from the other clock domain
      --three times. double stage synchronising circuit
      --reduces the MTB
      out_set       <= in_set;
      out_set_prev  <= out_set;
      out_set_prev2 <= out_set_prev;

    END IF;
  END PROCESS out_proc;
-----------------------------------------------------------------------------------
--asynchronous processes
-----------------------------------------------------------------------------------

-----------------------------------------------------------------------------------
--asynchronous mapping
-----------------------------------------------------------------------------------
  inbusy <= in_set OR in_reset_prev;

-------------------
-------------------
END syn;
