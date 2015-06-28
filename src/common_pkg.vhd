----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    19:32:46 06/23/2015
-- Design Name: 
-- Module Name:    Common package - Behavioral 
-- Project Name: 
-- Target Devices:
-- Tool versions: 
-- Description:    Defines commonly used types and functions
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
USE ieee.numeric_std.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
LIBRARY UNISIM;
USE UNISIM.VComponents.ALL;

PACKAGE common_pkg IS
  CONSTANT ADS5282BITS : positive := 12;
  TYPE ADS5282DATA IS ARRAY (integer RANGE <>) OF std_logic_vector(ADS5282BITS-1 DOWNTO 0);
END common_pkg;

PACKAGE BODY common_pkg IS
-- Future use
END common_pkg;
