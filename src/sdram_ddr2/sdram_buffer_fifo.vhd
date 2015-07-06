----------------------------------------------------------------------------------
-- Company:  LBNL
-- Engineer: Yuan Mei
-- 
-- Create Date: 07/05/2015 07:22:25 PM
-- Design Name: 
-- Module Name: sdram_buffer_fifo - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
--
-- Interface to MIG DDR2 UI to use external sdram as a buffer for
-- stream data input and output with fifo interface
-- Currently read and write are not allowed to happen simultaneously.
--
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
-- We use 26 or 25 bits in address to access 512MB or 256MB memory.  64 bit (8
-- bytes per) address.  MIG gives 31-bit wide address bus though.
-- Set APP_ADDR_WIDTH to the actual width + 1 so that wrap-around can work and
-- RD_ADDR_END can have highest bit 1 to indicate we want to read the whole memory
--
-- At the 1-clk wide WR_START pulse.  Afterwards, as writes advances, WR_POINTER
-- increments accordingly.  When WR_POINTER wraps around and hits the original RD_POINTER
-- asserts.  Writes will continue (overwritting previous data) until WR_STOP (1-clk)
-- asserts.  WR_STOP can be considered as a stop trigger.
--
-- AT RD_START (1-clk), RD_ADDR is loaded
----------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
USE ieee.numeric_std.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
LIBRARY UNISIM;
USE UNISIM.VComponents.ALL;

ENTITY sdram_buffer_fifo IS
  GENERIC (
    INDATA_WIDTH   : positive := 256;
    OUTDATA_WIDTH  : positive := 32;
    APP_ADDR_WIDTH : positive := 27;
    APP_DATA_WIDTH : positive := 128;
    APP_MASK_WIDTH : positive := 16;
    APP_ADDR_BURST : positive := 8      -- 4 or 8
  );
  PORT (
    CLK                : IN  std_logic;  -- MIG UI_CLK
    RESET              : IN  std_logic;
    --
    APP_ADDR           : OUT std_logic_vector(APP_ADDR_WIDTH-1 DOWNTO 0);
    APP_CMD            : OUT std_logic_vector(2 DOWNTO 0);
    APP_EN             : OUT std_logic;
    APP_RDY            : IN  std_logic;
    APP_WDF_DATA       : OUT std_logic_vector(APP_DATA_WIDTH-1 DOWNTO 0);
    APP_WDF_END        : OUT std_logic;
    APP_WDF_MASK       : OUT std_logic_vector(APP_MASK_WIDTH-1 DOWNTO 0);
    APP_WDF_WREN       : OUT std_logic;
    APP_WDF_RDY        : IN  std_logic;
    APP_RD_DATA        : IN  std_logic_vector(APP_DATA_WIDTH-1 DOWNTO 0);
    APP_RD_DATA_END    : IN  std_logic;
    APP_RD_DATA_VALID  : IN  std_logic;
    --
    CTRL_RESET         : IN  std_logic;
    WR_START           : IN  std_logic;
    WR_ADDR_BEGIN      : IN  std_logic_vector(APP_ADDR_WIDTH-1 DOWNTO 0);    
    WR_STOP            : IN  std_logic;
    WR_WRAP_AROUND     : IN  std_logic;
    POST_TRIGGER       : IN  std_logic_vector(APP_ADDR_WIDTH-1 DOWNTO 0);
    WR_BUSY            : OUT std_logic;    
    WR_POINTER         : OUT std_logic_vector(APP_ADDR_WIDTH-1 DOWNTO 0);    
    TRIGGER_POINTER    : OUT std_logic_vector(APP_ADDR_WIDTH-1 DOWNTO 0);    
    WR_WRAPPED         : OUT std_logic;
    RD_START           : IN  std_logic;
    RD_ADDR_BEGIN      : IN  std_logic_vector(APP_ADDR_WIDTH-1 DOWNTO 0);
    RD_ADDR_END        : IN  std_logic_vector(APP_ADDR_WIDTH-1 DOWNTO 0);    
    RD_BUSY            : OUT std_logic;
    --
    DATA_FIFO_RESET    : IN  std_logic;
    INDATA_FIFO_WRCLK  : IN  std_logic;
    INDATA_FIFO_Q      : IN  std_logic_vector(INDATA_WIDTH-1 DOWNTO 0);
    INDATA_FIFO_FULL   : OUT std_logic;
    INDATA_FIFO_WREN   : IN  std_logic;
    --
    OUTDATA_FIFO_RDCLK : IN  std_logic;
    OUTDATA_FIFO_Q     : OUT std_logic_vector(OUTDATA_WIDTH-1 DOWNTO 0);
    OUTDATA_FIFO_EMPTY : OUT std_logic;
    OUTDATA_FIFO_RDEN  : IN  std_logic
  );
END sdram_buffer_fifo;

ARCHITECTURE Behavioral OF sdram_buffer_fifo IS

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

  COMPONENT fifo256to128                -- FWFT
    PORT (
      RST        : IN  std_logic;
      WR_CLK     : IN  std_logic;
      RD_CLK     : IN  std_logic;
      DIN        : IN  std_logic_vector(255 DOWNTO 0);
      WR_EN      : IN  std_logic;
      RD_EN      : IN  std_logic;
      DOUT       : OUT std_logic_vector(127 DOWNTO 0);
      FULL       : OUT std_logic;
      EMPTY      : OUT std_logic;
      PROG_EMPTY : OUT std_logic        -- at least 4 reads left in the fifo for burst length = 8.
                                        -- set threshold to 5 in coregen
    );
  END COMPONENT;

  COMPONENT fifo128to32                 -- FWFT
    PORT (
      RST        : IN  std_logic;
      WR_CLK     : IN  std_logic;
      RD_CLK     : IN  std_logic;
      DIN        : IN  std_logic_vector(127 DOWNTO 0);
      WR_EN      : IN  std_logic;
      RD_EN      : IN  std_logic;
      DOUT       : OUT std_logic_vector(31 DOWNTO 0);
      FULL       : OUT std_logic;
      EMPTY      : OUT std_logic;
      PROG_FULL  : OUT std_logic        -- at least 4 slots left to write into, set threshold
                                        -- to total write depth (e.g. 16) - 4
    );
  END COMPONENT;

  CONSTANT DDR_CMD_WRITE     : std_logic_vector(2 DOWNTO 0) := "000";
  CONSTANT DDR_CMD_READ      : std_logic_vector(2 DOWNTO 0) := "001";
  --
  SIGNAL indata_fifo_rdclk   : std_logic;
  SIGNAL indata_fifo_rden    : std_logic;
  SIGNAL indata_fifo_dout    : std_logic_vector(APP_DATA_WIDTH-1 DOWNTO 0);
  SIGNAL indata_fifo_empty   : std_logic;
  --
  SIGNAL outdata_fifo_wren   : std_logic;
  SIGNAL outdata_fifo_full   : std_logic;
  SIGNAL outdata_fifo_din    : std_logic_vector(127 DOWNTO 0);
  --
  SIGNAL fifo_rst            : std_logic;
  --
  TYPE read_state_type IS (R0, R1, R2, R3, R4);
  SIGNAL read_state          : read_state_type              := R0;
  TYPE write_state_type IS (W0, W1, W2, W3, W4);
  SIGNAL write_state         : write_state_type             := W0;
  --
  SIGNAL rd_start_pulse      : std_logic                    := '0';
  SIGNAL rd_addr_begin_reg   : unsigned(APP_ADDR'length-1 DOWNTO 0);
  SIGNAL rd_addr_end_reg     : unsigned(APP_ADDR'length-1 DOWNTO 0);
  SIGNAL rd_addr_i           : unsigned(APP_ADDR'length-1 DOWNTO 0);
  SIGNAL rd_reading          : std_logic                    := '0';
  SIGNAL rd_app_en           : std_logic                    := '0';
  SIGNAL rd_app_cmd          : std_logic_vector(2 DOWNTO 0);
  SIGNAL rd_readable         : std_logic;
  --
  SIGNAL wr_addr_begin_reg   : unsigned(APP_ADDR'length-1 DOWNTO 0);
  SIGNAL wr_addr_i           : unsigned(APP_ADDR'length-1 DOWNTO 0);
  SIGNAL trigger_pointer_reg : unsigned(TRIGGER_POINTER'length-1 DOWNTO 0);
  SIGNAL post_trigger_reg    : unsigned(POST_TRIGGER'length-1 DOWNTO 0);
  SIGNAL wr_wrap_around_reg  : std_logic;
  SIGNAL wr_wrapped_i        : std_logic;
  SIGNAL wr_stopping         : std_logic;
  SIGNAL wr_en               : std_logic;
  SIGNAL wr_app_en           : std_logic                    := '0';
  SIGNAL wr_app_cmd          : std_logic_vector(2 DOWNTO 0);
  SIGNAL wr_start_pulse      : std_logic                    := '0';
  SIGNAL wr_stop_pulse       : std_logic                    := '0';
  SIGNAL wr_writing          : std_logic                    := '0';
  SIGNAL wr_wdf_end          : std_logic                    := '0';
  SIGNAL wr_wdf_wren         : std_logic                    := '0';
  
BEGIN

  fifo_rst <= RESET OR DATA_FIFO_RESET;

  indata_fifo : fifo256to128            -- FWFT
  PORT MAP (
    RST        => fifo_rst,
    WR_CLK     => INDATA_FIFO_WRCLK,
    RD_CLK     => indata_fifo_rdclk,
    DIN        => INDATA_FIFO_Q,
    WR_EN      => INDATA_FIFO_WREN,
    RD_EN      => indata_fifo_rden,
    DOUT       => indata_fifo_dout,
    FULL       => INDATA_FIFO_FULL,
    EMPTY      => OPEN,
    PROG_EMPTY => indata_fifo_empty
  );
  indata_fifo_rdclk <= CLK;
  APP_WDF_DATA      <= indata_fifo_dout;
  APP_WDF_MASK      <= (OTHERS => '0');

  outdata_fifo : fifo128to32           -- FWFT
  PORT MAP (
    RST        => fifo_rst,
    WR_CLK     => CLK,
    RD_CLK     => OUTDATA_FIFO_RDCLK,
    DIN        => outdata_fifo_din,
    WR_EN      => outdata_fifo_wren,
    RD_EN      => OUTDATA_FIFO_RDEN,
    DOUT       => OUTDATA_FIFO_Q,
    FULL       => OPEN,
    EMPTY      => OUTDATA_FIFO_EMPTY,
    PROG_FULL  => outdata_fifo_full
  );
  outdata_fifo_din <= APP_RD_DATA;

  ------------------------------------------------------------------------------  
  -- make sure _pulse's are 1-clk wide, since the inputs are from another clock
  -- domain
  pulse2pulse_rd_start : pulse2pulse
    PORT MAP (IN_CLK => CLK, OUT_CLK => CLK, RST => RESET, PULSEIN => RD_START,
              INBUSY => OPEN, PULSEOUT => rd_start_pulse);
  pulse2pulse_wr_start : pulse2pulse
    PORT MAP (IN_CLK => CLK, OUT_CLK => CLK, RST => RESET, PULSEIN => WR_START,
              INBUSY => OPEN, PULSEOUT => wr_start_pulse);
  pulse2pulse_wr_stop : pulse2pulse
    PORT MAP (IN_CLK => CLK, OUT_CLK => CLK, RST => RESET, PULSEIN => WR_STOP,
              INBUSY => OPEN, PULSEOUT => wr_stop_pulse);

  ------------------------------------------------------------------------------  
  -- register addresses and status
  PROCESS (CLK, RESET, CTRL_RESET)
    VARIABLE addr_tmp : unsigned(trigger_pointer_reg'length-1 DOWNTO 0) := (OTHERS => '0');
  BEGIN
    IF RESET = '1' OR CTRL_RESET = '1' THEN
      wr_addr_begin_reg   <= (OTHERS => '0');
      wr_wrap_around_reg  <= '0';
      post_trigger_reg    <= (OTHERS => '0');
      wr_wrapped_i        <= '0';
      wr_stopping         <= '0';
      wr_writing          <= '0';
      rd_addr_begin_reg   <= (OTHERS => '0');
      rd_addr_end_reg     <= ('1', OTHERS => '0');
      rd_reading          <= '0';
    ELSIF rising_edge(CLK) THEN
      -- start
      IF wr_start_pulse = '1' THEN
        wr_addr_begin_reg   <= unsigned(WR_ADDR_BEGIN);
        wr_wrap_around_reg  <= WR_WRAP_AROUND;
        wr_writing          <= '1';
        wr_stopping         <= '0';
        wr_wrapped_i        <= '0';
        rd_reading          <= '0';     -- abort reading
      -- wrap around
      ELSIF wr_addr_i >= ('1' & wr_addr_begin_reg(wr_addr_begin_reg'length-2 DOWNTO 0)) THEN
        -- when no wrap-around, automatically stop upon address collision
        IF wr_wrap_around_reg = '0' THEN
          wr_writing  <= '0';
          wr_stopping <= '0';
        END IF;
        -- update when a wrap around occures
        wr_wrapped_i  <= '1';
      END IF;
      -- stop
      IF wr_stop_pulse = '1' THEN
        post_trigger_reg <= unsigned(POST_TRIGGER);
        IF wr_writing = '1' THEN  -- IF we are reading etc, wr_stop won't trigger
          trigger_pointer_reg <= wr_addr_i;
          wr_stopping         <= '1';
        END IF;
      END IF;
      -- stopping condition
      IF wr_stopping = '1' THEN
        addr_tmp := trigger_pointer_reg + post_trigger_reg;
        IF addr_tmp = wr_addr_i THEN
          wr_writing  <= '0';
          wr_stopping <= '0';
        END IF;
      END IF;
      -- reading
      IF rd_start_pulse = '1' THEN
        wr_writing        <= '0';       -- abort any writing
        wr_stopping       <= '0';
        rd_reading        <= '1';
        rd_addr_begin_reg <= unsigned(RD_ADDR_BEGIN);
        rd_addr_end_reg   <= unsigned(RD_ADDR_END);
      ELSIF rd_addr_i >= rd_addr_end_reg THEN
        rd_reading <= '0';
      END IF;
    END IF;
  END PROCESS;

  -- write command and data
  PROCESS (CLK, RESET, CTRL_RESET)
    VARIABLE write_cnt : integer RANGE 0 TO APP_ADDR_BURST;
  BEGIN
    IF RESET = '1' OR CTRL_RESET = '1' THEN
      wr_addr_i   <= (OTHERS => '0');
      write_state <= W0;
    ELSIF rising_edge(CLK) THEN
      write_state      <= W0;
      indata_fifo_rden <= '0';
      wr_wdf_wren      <= '0';
      wr_wdf_end       <= '0';
      wr_app_en        <= '0';
      CASE write_state IS
        WHEN W0 =>
          IF APP_RDY = '1' AND APP_WDF_RDY = '1' AND
            indata_fifo_empty = '0' AND wr_writing = '1' THEN
            indata_fifo_rden <= '1';    -- read next data from fifo
            wr_app_en        <= '1';
            wr_wdf_wren      <= '1';
            write_cnt        := 0;
            write_state      <= W1;
          END IF;
        WHEN W1 =>
          IF wr_writing = '0' THEN
            write_state <= W0;
          ELSIF APP_RDY = '0' OR APP_WDF_RDY = '0' THEN
            write_state <= W1;
          ELSE
            wr_wdf_wren      <= '1';
            indata_fifo_rden <= '1';
            write_state      <= W1;
            IF (write_cnt = 0 AND APP_ADDR_BURST = 4) OR (write_cnt = 2 AND APP_ADDR_BURST = 8)
            THEN
              wr_addr_i   <= wr_addr_i + APP_ADDR_BURST;
              write_state <= W0;
            END IF;
            write_cnt := write_cnt + 1;            
          END IF;

        WHEN OTHERS =>
          write_state <= W0;
      END CASE;
      IF wr_start_pulse = '1' THEN  -- wr_writing must be true from this point on
        wr_addr_i   <= unsigned(WR_ADDR_BEGIN);
        write_state <= W0;
      END IF;
    END IF;
  END PROCESS;
  wr_app_cmd <= DDR_CMD_WRITE;
  wr_en      <= wr_app_en OR wr_wdf_wren;

  -- read command and data
  PROCESS (CLK, RESET, CTRL_RESET)
  BEGIN
    IF RESET = '1' OR CTRL_RESET = '1' THEN
      rd_addr_i         <= (OTHERS => '0');
      rd_app_en         <= '0';
      read_state        <= R0;
    ELSIF rising_edge(CLK) THEN
      rd_app_en         <= '0';
      read_state        <= R0;
      CASE read_state IS
        WHEN R0 =>
        -- ALL back to defaults
        WHEN R1 =>
          read_state <= R1;
          IF rd_readable = '1' THEN
            rd_app_en  <= '1';
            read_state <= R2;
          END IF;
        WHEN R2 =>
          read_state <= R2;
          IF APP_RD_DATA_VALID = '1' THEN
            rd_addr_i  <= rd_addr_i + APP_ADDR_BURST;
            read_state <= R1;
          END IF;
        WHEN OTHERS =>
          read_state <= R0;
      END CASE;
      -- higher priority conditions
      IF rd_reading = '0' THEN
        rd_addr_i         <= (OTHERS => '0');
        rd_app_en         <= '0';
        read_state        <= R0;
      END IF;
      IF rd_start_pulse = '1' THEN  -- rd_reading must be true from this point on
        rd_addr_i  <= unsigned(RD_ADDR_BEGIN);
        read_state <= R1;
      END IF;
    END IF;
  END PROCESS;
  rd_app_cmd        <= DDR_CMD_READ;
  rd_readable       <= APP_RDY AND (NOT outdata_fifo_full) AND (NOT APP_RD_DATA_VALID);
  outdata_fifo_wren <= APP_RD_DATA_VALID AND rd_reading;

  -- connect signals
  APP_ADDR <= '0' & std_logic_vector(wr_addr_i(wr_addr_i'length-2 DOWNTO 0))
              WHEN wr_writing = '1' ELSE
              '0' & std_logic_vector(rd_addr_i(rd_addr_i'length-2 DOWNTO 0));
  APP_CMD         <= wr_app_cmd WHEN wr_writing = '1' ELSE rd_app_cmd;
  APP_EN          <= wr_app_en OR rd_app_en;
  APP_WDF_END     <= wr_wdf_end;
  APP_WDF_WREN    <= wr_wdf_wren;
  --
  WR_BUSY         <= wr_writing;
  WR_POINTER      <= std_logic_vector(wr_addr_i);
  WR_WRAPPED      <= wr_wrapped_i;
  TRIGGER_POINTER <= std_logic_vector(trigger_pointer_reg);
  --
  RD_BUSY         <= rd_reading;

END Behavioral;
