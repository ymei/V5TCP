----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    19:32:46 08/25/2013 
-- Design Name: 
-- Module Name:    pulsegen - Behavioral 
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

ENTITY pulsegen IS
  GENERIC (
    period        : positive := 125000000;
    counter_width : positive := 27
  );
  PORT (
    clk : IN  std_logic;
    I   : IN  std_logic;
    O   : OUT std_logic
  );
END pulsegen;

ARCHITECTURE Behavioral OF pulsegen IS
  SIGNAL counter : unsigned(counter_width - 1 DOWNTO 0) := (OTHERS => '0');
BEGIN

  PROCESS (clk, I) IS
  BEGIN
    IF I = '1' THEN
      O <= '1';
    ELSIF rising_edge(clk) THEN
      counter <= counter + 1;
      O       <= '0';
      IF counter = period-1 THEN
        O <= '1';
      ELSIF counter >= period THEN
        O       <= '0';
        counter <= (OTHERS => '0');
      END IF;
    END IF;
  END PROCESS;

END Behavioral;
