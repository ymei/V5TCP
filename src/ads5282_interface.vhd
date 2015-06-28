----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    19:32:46 06/22/2015
-- Design Name: 
-- Module Name:    ads5282_interface - Behavioral 
-- Project Name: 
-- Target Devices: Virtex5
-- Tool versions: 
-- Description:    Interface to ADC5282: 8ch 12bit ADC.  Samples DDR data on
--                 each edge separately with ISERDES then combine.
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

USE work.common_pkg.ALL;

ENTITY ads5282_interface IS
  GENERIC (
    ADC_NCH : positive := 8
  );
  PORT (
    RESET   : IN  std_logic;
    CLK     : IN  std_logic;
    --
    CONFIG  : IN  std_logic_vector(31 DOWNTO 0);
    CONFPS  : IN  std_logic;
    CONFULL : OUT std_logic;
    --
    ADCLKp  : IN  std_logic;            -- LVDS frame clock (1X)
    ADCLKn  : IN  std_logic;
    LCLKp   : IN  std_logic;            -- LVDS bit clock (6X)
    LCLKn   : IN  std_logic;
    DATAp   : IN  std_logic_vector(ADC_NCH-1 DOWNTO 0);
    DATAn   : IN  std_logic_vector(ADC_NCH-1 DOWNTO 0);
    --
    ADCLK   : OUT std_logic;
    DATA    : OUT ADS5282DATA(ADC_NCH-1 DOWNTO 0);
    --
    SCLK    : OUT std_logic;
    SDATA   : OUT std_logic;
    CSn     : OUT std_logic
  );
END ads5282_interface;

ARCHITECTURE Behavioral OF ads5282_interface IS

  CONSTANT CTRL_DATA_WIDTH : positive := 6;
  TYPE ctrl_data_array IS ARRAY (integer RANGE <>) OF std_logic_vector(CTRL_DATA_WIDTH-1 DOWNTO 0);

  COMPONENT shiftreg_drive
    GENERIC (
      WIDTH   : positive := 24;           -- parallel data width
      CLK_DIV : positive := 2             -- SCLK freq is CLK / 2**(CLK_DIV+1)
    );    
    PORT(
      CLK   : IN  std_logic;
      RESET : IN  std_logic;
      DATA  : IN  std_logic_vector(WIDTH-1 DOWNTO 0);
      START : IN  std_logic;
      BUSY  : OUT std_logic;
      SCLK  : OUT std_logic;
      DOUT  : OUT std_logic;
      SYNCn : OUT std_logic
    );
  END COMPONENT;
  --
  COMPONENT edge_sync
    GENERIC (
      EDGE : std_logic := '1'  -- '1'  :  rising edge,  '0' falling edge
    );
    PORT (
      RESET : IN  std_logic;
      CLK   : IN  std_logic;
      EI    : IN  std_logic;
      SO    : OUT std_logic
    );
  END COMPONENT;
  --
  COMPONENT width_pulse_sync
    GENERIC (
      DATA_WIDTH : positive := CTRL_DATA_WIDTH;
      MODE       : natural
    );
    PORT (
      RESET : IN  std_logic;
      CLK   : IN  std_logic;
      PW    : IN  std_logic_vector(DATA_WIDTH-1 DOWNTO 0);
      START : IN  std_logic;  -- should be synchronous to CLK, of any width
      BUSY  : OUT std_logic;
      --
      CLKO  : IN  std_logic;
      RSTO  : OUT std_logic;
      PO    : OUT std_logic
    );
  END COMPONENT;
  --
  COMPONENT fifo32comclk
    PORT (
      CLK   : IN  std_logic;
      RST   : IN  std_logic;
      DIN   : IN  std_logic_vector(31 DOWNTO 0);
      WR_EN : IN  std_logic;
      RD_EN : IN  std_logic;
      DOUT  : OUT std_logic_vector(31 DOWNTO 0);
      FULL  : OUT std_logic;
      EMPTY : OUT std_logic
    );
  END COMPONENT;
  
  SIGNAL adcLCLK               : std_logic;  -- adjusted LCLK inside of FPGA fabric
  SIGNAL adcLCLKb              : std_logic;
  SIGNAL adcLCLK_int           : std_logic;
  SIGNAL adcLCLK_int1          : std_logic;
  SIGNAL adclk_ext             : std_logic;  -- ADCLK directly from the ADC
  SIGNAL adclk_rec             : std_logic;  -- reconstructed adclk from LCLK
  SIGNAL lclkIodelayCE         : std_logic;
  SIGNAL lclkIodelayINC        : std_logic;
  SIGNAL lclkIodelayRST        : std_logic;
  SIGNAL lclkIodelayCtrlPW     : std_logic_vector(CTRL_DATA_WIDTH-1 DOWNTO 0);
  SIGNAL lclkIodelayCtrlStart  : std_logic;
  SIGNAL lclkIodelayCtrlBusy   : std_logic;
  SIGNAL bufrRST               : std_logic;
  SIGNAL bufrCLR               : std_logic;
  --
  SIGNAL adcInDataP            : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL adcDelayedDataP       : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL dataPIodelayCE        : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL dataPIodelayINC       : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL dataPIodelayRST       : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL dataPIodelayCtrlPW    : ctrl_data_array(ADC_NCH-1 DOWNTO 0);
  SIGNAL dataPIodelayCtrlStart : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL dataPIodelayCtrlBusy  : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL bitSlipPRST           : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL bitSlipP              : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL bitSlipPCtrlPW        : ctrl_data_array(ADC_NCH-1 DOWNTO 0);
  SIGNAL bitSlipPCtrlStart     : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL bitSlipPCtrlBusy      : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL adcInDataN            : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL adcDelayedDataN       : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL dataNIodelayCE        : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL dataNIodelayINC       : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL dataNIodelayRST       : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL dataNIodelayCtrlPW    : ctrl_data_array(ADC_NCH-1 DOWNTO 0);
  SIGNAL dataNIodelayCtrlStart : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL dataNIodelayCtrlBusy  : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL bitSlipNRST           : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL bitSlipN              : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL bitSlipNCtrlPW        : ctrl_data_array(ADC_NCH-1 DOWNTO 0);
  SIGNAL bitSlipNCtrlStart     : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL bitSlipNCtrlBusy      : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  --
  SIGNAL Q1p                   : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL Q2p                   : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL Q3p                   : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL Q4p                   : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL Q5p                   : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL Q6p                   : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL Q1n                   : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL Q2n                   : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL Q3n                   : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL Q4n                   : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL Q5n                   : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  SIGNAL Q6n                   : std_logic_vector(ADC_NCH-1 DOWNTO 0);
  --
  SIGNAL serialStart           : std_logic;
  SIGNAL serialStartO          : std_logic;
  SIGNAL serialBusy            : std_logic;
  SIGNAL sclk_buf              : std_logic;
  --
  SIGNAL configAddr            : std_logic_vector(CONFIG'length-CTRL_DATA_WIDTH-1 DOWNTO 0);
  CONSTANT bufrRSTAddr         : std_logic_vector(CONFIG'length-CTRL_DATA_WIDTH-1 DOWNTO 0) := (CONFIG'length-CTRL_DATA_WIDTH-1 => '0', OTHERS => '1');
  CONSTANT lclkIodelayCtrlAddr : std_logic_vector(CONFIG'length-CTRL_DATA_WIDTH-1 DOWNTO 0) := (CONFIG'length-CTRL_DATA_WIDTH-1 => '0', 0 => '0', OTHERS => '1');
  SIGNAL configFIFOout         : std_logic_vector(CONFIG'length-1 DOWNTO 0);
  SIGNAL configFIFOempty       : std_logic;
  SIGNAL configFIFOvalid       : std_logic;
  SIGNAL configFIFOwrEn        : std_logic;
  SIGNAL configFIFOrdEn        : std_logic;
  SIGNAL configBusy            : std_logic;

BEGIN

  -- config word FIFO
  config_fifo_inst : fifo32comclk
    PORT MAP (
      CLK   => CLK,
      RST   => RESET,
      DIN   => CONFIG,
      WR_EN => configFIFOwrEn,
      RD_EN => configFIFOrdEn,
      dout  => configFIFOout,
      FULL  => CONFULL,
      EMPTY => configFIFOempty
    );
  -- capture the rising edge of CONFPS to write to FIFO
  confps_rise_es : PROCESS (CLK, CONFPS) IS
    VARIABLE prev : std_logic;
  BEGIN
    IF rising_edge(CLK) THEN
      prev := CONFPS;
    END IF;
    IF prev = '0' AND CONFPS = '1' THEN
      configFIFOwrEn <= '1';
    ELSE
      configFIFOwrEn <= '0';
    END IF;
  END PROCESS;
  configFIFOvalid <= NOT configFIFOempty;
  -- congregate busy signals
  configBusy_gen : PROCESS (lclkIodelayCtrlBusy, dataPIodelayCtrlBusy, bitSlipPCtrlBusy,
                            bufrCLR, serialBusy, dataNIodelayCtrlBusy, bitSlipNCtrlBusy) IS
    VARIABLE p : std_logic;
  BEGIN
    p := '0';
    FOR iCh IN 0 TO ADC_NCH-1 LOOP
      p := p OR dataPIodelayCtrlBusy(iCh) OR bitSlipPCtrlBusy(iCh)
             OR dataNIodelayCtrlBusy(iCh) OR bitSlipNCtrlBusy(iCh);
    END LOOP;  -- iCh
    configBusy <= p OR lclkIodelayCtrlBusy OR bufrCLR OR serialBusy;
  END PROCESS;
  -- capture the falling edge of configBusy to advance a FIFO read
  configBusy_fall_es : PROCESS (CLK, configBusy) IS
    VARIABLE prev : std_logic;
  BEGIN
    IF rising_edge(CLK) THEN
      prev := configBusy;
    END IF;
    IF prev = '1' AND configBusy = '0' THEN
      configFIFOrdEn <= '1';
    ELSE
      configFIFOrdEn <= '0';
    END IF;
  END PROCESS;
  -- mux configFIFOvalid to start pulse signals
  configAddr           <= configFIFOout(CONFIG'length-1 DOWNTO CTRL_DATA_WIDTH);
  --
  serialStart          <= configFIFOvalid WHEN configFIFOout(CONFIG'length-1) = '1' ELSE '0';
  bufrRST              <= configFIFOvalid WHEN configAddr = bufrRSTAddr             ELSE '0';
  lclkIodelayCtrlStart <= configFIFOvalid WHEN configAddr = lclkIodelayCtrlAddr     ELSE '0';
  lclkIodelayCtrlPW    <= configFIFOout(CTRL_DATA_WIDTH-1 DOWNTO 0);
  
  config_mux : FOR iCh IN 0 TO ADC_NCH-1 GENERATE
    dataPIodelayCtrlStart(iCh) <= configFIFOvalid WHEN configAddr = std_logic_vector(to_unsigned(iCh,    CONFIG'length-CTRL_DATA_WIDTH)) ELSE '0';
    dataNIodelayCtrlStart(iCh) <= configFIFOvalid WHEN configAddr = std_logic_vector(to_unsigned(iCh+8,  CONFIG'length-CTRL_DATA_WIDTH)) ELSE '0';
    bitSlipPCtrlStart(iCh)     <= configFIFOvalid WHEN configAddr = std_logic_vector(to_unsigned(iCh+16, CONFIG'length-CTRL_DATA_WIDTH)) ELSE '0';
    bitSlipNCtrlStart(iCh)     <= configFIFOvalid WHEN configAddr = std_logic_vector(to_unsigned(iCh+24, CONFIG'length-CTRL_DATA_WIDTH)) ELSE '0';
    --
    dataPIodelayCtrlPW(iCh) <= configFIFOout(CTRL_DATA_WIDTH-1 DOWNTO 0);
    dataNIodelayCtrlPW(iCh) <= configFIFOout(CTRL_DATA_WIDTH-1 DOWNTO 0);
    bitSlipPCtrlPW(iCh)     <= configFIFOout(CTRL_DATA_WIDTH-1 DOWNTO 0);
    bitSlipNCtrlPW(iCh)     <= configFIFOout(CTRL_DATA_WIDTH-1 DOWNTO 0);
  END GENERATE;
  
  -- serial interface
  serialStart_rise_es : PROCESS (CLK, serialStart) IS
    VARIABLE prev : std_logic;
  BEGIN
    IF rising_edge(CLK) THEN
      prev := serialStart;
    END IF;
    IF prev = '0' AND serialStart = '1' THEN
      serialStartO <= '1';
    ELSE
      serialStartO <= '0';
    END IF;
  END PROCESS;
  --
  sd : shiftreg_drive
    GENERIC MAP (
      WIDTH   => 24,
      CLK_DIV => 2
    )
    PORT MAP (
      CLK   => CLK,
      RESET => RESET,
      DATA  => configFIFOout(23 DOWNTO 0),
      START => serialStartO,
      BUSY  => serialBusy,
      SCLK  => sclk_buf,
      DOUT  => SDATA,
      SYNCn => CSn
    );
  SCLK <= NOT sclk_buf;  -- ADS5282 requires the rising edge of SCLK to be in the middle OF SDATA.

  -- ADCLK directly from the ADC
  adclk_ext_ibufds_inst : IBUFGDS
    GENERIC MAP (
      DIFF_TERM  => TRUE,               -- Differential Termination 
      IOSTANDARD => "DEFAULT"
    )
    PORT MAP (
      O  => adclk_ext,                  -- Buffer output
      I  => ADCLKp,  -- Diff_p buffer input (connect directly to top-level port)
      IB => ADCLKn   -- Diff_n buffer input (connect directly to top-level port)
    );

  -- idelay the bit clock
  lclk_ibufds_inst : IBUFGDS
    GENERIC MAP (
      DIFF_TERM  => TRUE,               -- Differential Termination 
      IOSTANDARD => "DEFAULT"
    )
    PORT MAP (
      O  => adcLCLK_int,                -- Buffer output
      I  => LCLKp,  -- Diff_p buffer input (connect directly to top-level port)
      IB => LCLKn   -- Diff_n buffer input (connect directly to top-level port)
    );
  lclk_iodelay_inst : IODELAY
      GENERIC MAP (
        DELAY_SRC             => "I",   -- Specify which input port to be used
        -- "I"=IDATAIN, "O"=ODATAIN, "DATAIN"=DATAIN, "IO"=Bi-directional
        HIGH_PERFORMANCE_MODE => TRUE, -- TRUE specifies lower jitter
        -- at expense of more power
        IDELAY_TYPE           => "VARIABLE",  -- "FIXED" or "VARIABLE" 
        IDELAY_VALUE          => 0,     -- 0 to 63 tap values
        ODELAY_VALUE          => 0,     -- 0 to 63 tap values
        REFCLK_FREQUENCY      => 200.0,   -- Frequency used for IDELAYCTRL
        -- 175.0 to 225.0
        SIGNAL_PATTERN        => "CLOCK"  -- Input signal type, "CLOCK" or "DATA"
      )
      PORT MAP (
        DATAOUT => adcLCLK_int1,        -- 1-bit delayed data output
        C       => adclk_rec,           -- 1-bit clock input
        CE      => lclkIodelayCE,       -- 1-bit clock enable input
        DATAIN  => '0',                 -- 1-bit internal data input
        IDATAIN => adcLCLK_int,     -- 1-bit input data input (connect to port)
        INC     => lclkIodelayINC,  -- 1-bit increment(1)/decrement(0) input
        ODATAIN => '0',                 -- 1-bit output data input
        RST     => lclkIodelayRST,      -- 1-bit active high, synch reset input
        T       => '1'              -- 1-bit 3-state control input, '1' = input
      );
  lclk_bufio_inst : BUFIO
    PORT MAP (
      O => adcLCLK,                     -- Clock buffer output
      I => adcLCLK_int1                 -- Clock buffer input
    );
  -- reconstruct the frame clock from bit clock
  lclk_bufr_inst : BUFR
    GENERIC MAP (
      BUFR_DIVIDE => "6",  -- "BYPASS", "1", "2", "3", "4", "5", "6", "7", "8" 
      SIM_DEVICE  => "VIRTEX5"          -- Specify target device
    )
    PORT MAP (
      O   => adclk_rec,                 -- Clock buffer output
      CE  => '1',                       -- Clock enable input
      CLR => bufrCLR,                   -- Clock buffer reset input
      I   => adcLCLK_int1               -- Clock buffer input
    );
  -- synchronize the phase of reconstructed frame clock to ADC's original frame clock
  adclk_ext_bufr_es_inst : edge_sync
    GENERIC MAP (
      EDGE => '1'  -- '1'  :  rising edge,  '0' falling edge
    )
    PORT MAP (
      RESET => '0',
      CLK   => adclk_ext,
      EI    => bufrRST,
      SO    => bufrCLR
    );
  --
  ADCLK    <= adclk_rec;
  adcLCLKb <= NOT adcLCLK;
  --
  lclk_iodelay_ctrl_inst : width_pulse_sync
    GENERIC MAP (
      DATA_WIDTH => CTRL_DATA_WIDTH,
      MODE       => 0
    )
    PORT MAP (
      RESET => RESET,
      CLK   => CLK,
      PW    => lclkIodelayCtrlPW,
      START => lclkIodelayCtrlStart,
      BUSY  => lclkIodelayCtrlBusy,
      --
      CLKO  => adclk_rec,
      RSTO  => lclkIodelayRST,
      PO    => lclkIodelayINC
    );
  lclkIodelayCE <= lclkIodelayINC;

  -- data stream
  data_iserdes_iodelay_insts : FOR iCh IN 0 TO ADC_NCH-1 GENERATE
    data_ibufds_diff_out_inst : IBUFDS_DIFF_OUT
    GENERIC MAP (
      DIFF_TERM  => TRUE,               -- Differential Termination 
      IOSTANDARD => "DEFAULT"
    )
    PORT MAP (
      O  => adcInDataP(iCh),            -- Buffer output
      OB => adcInDataN(iCh),
      I  => DATAp(iCh),  -- Diff_p buffer input (connect directly to top-level port)
      IB => DATAn(iCh)   -- Diff_n buffer input (connect directly to top-level port)
    );
    -- sample on positive and negative edges separately, then combine
    datap_iodelay_inst : IODELAY
      GENERIC MAP (
        DELAY_SRC             => "I",   -- Specify which input port to be used
        -- "I"=IDATAIN, "O"=ODATAIN, "DATAIN"=DATAIN, "IO"=Bi-directional
        HIGH_PERFORMANCE_MODE => TRUE,  -- TRUE specifies lower jitter
        -- at expense of more power
        IDELAY_TYPE           => "VARIABLE",  -- "FIXED" or "VARIABLE" 
        IDELAY_VALUE          => 0,     -- 0 to 63 tap values
        ODELAY_VALUE          => 0,     -- 0 to 63 tap values
        REFCLK_FREQUENCY      => 200.0,  -- Frequency used for IDELAYCTRL
        -- 175.0 to 225.0
        SIGNAL_PATTERN        => "DATA"  -- Input signal type, "CLOCK" or "DATA"
      )
      PORT MAP (
        DATAOUT => adcDelayedDataP(iCh),  -- 1-bit delayed data output
        C       => adclk_rec,           -- 1-bit clock input
        CE      => dataPIodelayCE(iCh),   -- 1-bit clock enable input
        DATAIN  => '0',                 -- 1-bit internal data input
        IDATAIN => adcInDataP(iCh),  -- 1-bit input data input (connect to port)
        INC     => dataPIodelayINC(iCh),  -- 1-bit increment(1)/decrement(0) input
        ODATAIN => '0',                 -- 1-bit output data input
        RST     => dataPIodelayRST(iCh),  -- 1-bit active high, synch reset input
        T       => '1'  -- 1-bit 3-state control input, '1' = input
      );
    datap_iodelay_ctrl_inst : width_pulse_sync
      GENERIC MAP (
        DATA_WIDTH => CTRL_DATA_WIDTH,
        MODE       => 0
      )
      PORT MAP (
        RESET => RESET,
        CLK   => CLK,
        PW    => dataPIodelayCtrlPW(iCh),
        START => dataPIodelayCtrlStart(iCh),
        BUSY  => dataPIodelayCtrlBusy(iCh),
        --
        CLKO  => adclk_rec,
        RSTO  => dataPIodelayRST(iCh),
        PO    => dataPIodelayINC(iCh)
    );
    dataPIodelayCE(iCh) <= dataPIodelayINC(iCh);

    datan_iodelay_inst : IODELAY
      GENERIC MAP (
        DELAY_SRC             => "I",   -- Specify which input port to be used
        -- "I"=IDATAIN, "O"=ODATAIN, "DATAIN"=DATAIN, "IO"=Bi-directional
        HIGH_PERFORMANCE_MODE => TRUE,  -- TRUE specifies lower jitter
        -- at expense of more power
        IDELAY_TYPE           => "VARIABLE",  -- "FIXED" or "VARIABLE" 
        IDELAY_VALUE          => 0,     -- 0 to 63 tap values
        ODELAY_VALUE          => 0,     -- 0 to 63 tap values
        REFCLK_FREQUENCY      => 200.0,  -- Frequency used for IDELAYCTRL
        -- 175.0 to 225.0
        SIGNAL_PATTERN        => "DATA"  -- Input signal type, "CLOCK" or "DATA"
      )
      PORT MAP (
        DATAOUT => adcDelayedDataN(iCh),  -- 1-bit delayed data output
        C       => adclk_rec,           -- 1-bit clock input
        CE      => dataNIodelayCE(iCh),   -- 1-bit clock enable input
        DATAIN  => '0',                 -- 1-bit internal data input
        IDATAIN => adcInDataN(iCh),  -- 1-bit input data input (connect to port)
        INC     => dataNIodelayINC(iCh),  -- 1-bit increment(1)/decrement(0) input
        ODATAIN => '0',                 -- 1-bit output data input
        RST     => dataNIodelayRST(iCh),  -- 1-bit active high, synch reset input
        T       => '1'  -- 1-bit 3-state control input, '1' = input
      );
    datan_iodelay_ctrl_inst : width_pulse_sync
      GENERIC MAP (
        DATA_WIDTH => CTRL_DATA_WIDTH,
        MODE       => 0
      )
      PORT MAP (
        RESET => RESET,
        CLK   => CLK,
        PW    => dataNIodelayCtrlPW(iCh),
        START => dataNIodelayCtrlStart(iCh),
        BUSY  => dataNIodelayCtrlBusy(iCh),
        --
        CLKO  => adclk_rec,
        RSTO  => dataNIodelayRST(iCh),
        PO    => dataNIodelayINC(iCh)
    );
    dataNIodelayCE(iCh) <= dataNIodelayINC(iCh);
    --
    datap_iserdes_nodelay_inst : ISERDES_NODELAY
      GENERIC MAP (
        BITSLIP_ENABLE => TRUE,         -- TRUE/FALSE to enable bitslip controller
        --   Must be "FALSE" when interface type is "MEMORY" 
        DATA_RATE      => "SDR",        -- Specify data rate of "DDR" or "SDR" 
        DATA_WIDTH     => 6,            -- Specify data width - 
        --  NETWORKING SDR: 2, 3, 4, 5, 6, 7, 8 : DDR 4, 6, 8, 10
        --  MEMORY SDR N/A : DDR 4
        INTERFACE_TYPE => "NETWORKING",  -- Use model - "MEMORY" or "NETWORKING" 
        NUM_CE         => 1,  -- Define number or clock enables to an integer of 1 or 2
        SERDES_MODE    => "MASTER"  --Set SERDES mode to "MASTER" or "SLAVE" 
      )
      PORT MAP (
        Q1        => Q1p(iCh),             -- 1-bit registered SERDES output
        Q2        => Q2p(iCh),             -- 1-bit registered SERDES output
        Q3        => Q3p(iCh),             -- 1-bit registered SERDES output
        Q4        => Q4p(iCh),             -- 1-bit registered SERDES output
        Q5        => Q5p(iCh),             -- 1-bit registered SERDES output
        Q6        => Q6p(iCh),             -- 1-bit registered SERDES output
        SHIFTOUT1 => OPEN,                 -- 1-bit cascade Master/Slave output
        SHIFTOUT2 => OPEN,                 -- 1-bit cascade Master/Slave output
        BITSLIP   => bitSlipP(iCh),        -- 1-bit Bitslip enable input
        CE1       => '1',               -- 1-bit clock enable input
        CE2       => '1',               -- 1-bit clock enable input
        CLK       => adcLCLK,           -- 1-bit master clock input
        CLKB      => adcLCLKb,  -- 1-bit secondary clock input for DATA_RATE=DDR
        CLKDIV    => adclk_rec,         -- 1-bit divided clock input
        D         => adcDelayedDataP(iCh),  -- 1-bit data input, connects to IODELAY or input buffer
        OCLK      => '0',               -- 1-bit fast output clock input
        RST       => bitSlipPRST(iCh),  -- 1-bit asynchronous reset input
        SHIFTIN1  => '0',               -- 1-bit cascade Master/Slave input
        SHIFTIN2  => '0'                -- 1-bit cascade Master/Slave input
      );
    datap_iserdes_ctrl_inst : width_pulse_sync
      GENERIC MAP (
        DATA_WIDTH => CTRL_DATA_WIDTH,
        MODE       => 1
      )
      PORT MAP (
        RESET => RESET,
        CLK   => CLK,
        PW    => bitSlipPCtrlPW(iCh),
        START => bitSlipPCtrlStart(iCh),
        BUSY  => bitSlipPCtrlBusy(iCh),
        --
        CLKO  => adclk_rec,
        RSTO  => bitSlipPRST(iCh),
        PO    => bitSlipP(iCh)
    );

    datan_iserdes_nodelay_inst : ISERDES_NODELAY
      GENERIC MAP (
        BITSLIP_ENABLE => TRUE,         -- TRUE/FALSE to enable bitslip controller
        --   Must be "FALSE" when interface type is "MEMORY" 
        DATA_RATE      => "SDR",        -- Specify data rate of "DDR" or "SDR" 
        DATA_WIDTH     => 6,            -- Specify data width - 
        --  NETWORKING SDR: 2, 3, 4, 5, 6, 7, 8 : DDR 4, 6, 8, 10
        --  MEMORY SDR N/A : DDR 4
        INTERFACE_TYPE => "NETWORKING",  -- Use model - "MEMORY" or "NETWORKING" 
        NUM_CE         => 1,  -- Define number or clock enables to an integer of 1 or 2
        SERDES_MODE    => "MASTER"  --Set SERDES mode to "MASTER" or "SLAVE" 
      )
      PORT MAP (
        Q1        => Q1n(iCh),          -- 1-bit registered SERDES output
        Q2        => Q2n(iCh),          -- 1-bit registered SERDES output
        Q3        => Q3n(iCh),          -- 1-bit registered SERDES output
        Q4        => Q4n(iCh),          -- 1-bit registered SERDES output
        Q5        => Q5n(iCh),          -- 1-bit registered SERDES output
        Q6        => Q6n(iCh),          -- 1-bit registered SERDES output
        SHIFTOUT1 => OPEN,              -- 1-bit cascade Master/Slave output
        SHIFTOUT2 => OPEN,              -- 1-bit cascade Master/Slave output
        BITSLIP   => bitSlipN(iCh),     -- 1-bit Bitslip enable input
        CE1       => '1',               -- 1-bit clock enable input
        CE2       => '1',               -- 1-bit clock enable input
        CLK       => adcLCLKb,          -- 1-bit master clock input
        CLKB      => adcLCLK,  -- 1-bit secondary clock input for DATA_RATE=DDR
        CLKDIV    => adclk_rec,         -- 1-bit divided clock input
        D         => adcDelayedDataN(iCh),  -- 1-bit data input, connects to IODELAY or input buffer
        OCLK      => '0',               -- 1-bit fast output clock input
        RST       => bitSlipNRST(iCh),  -- 1-bit asynchronous reset input
        SHIFTIN1  => '0',               -- 1-bit cascade Master/Slave input
        SHIFTIN2  => '0'                -- 1-bit cascade Master/Slave input
      );
    datan_iserdes_ctrl_inst : width_pulse_sync
      GENERIC MAP (
        DATA_WIDTH => CTRL_DATA_WIDTH,
        MODE       => 1
      )
      PORT MAP (
        RESET => RESET,
        CLK   => CLK,
        PW    => bitSlipNCtrlPW(iCh),
        START => bitSlipNCtrlStart(iCh),
        BUSY  => bitSlipNCtrlBusy(iCh),
        --
        CLKO  => adclk_rec,
        RSTO  => bitSlipNRST(iCh),
        PO    => bitSlipN(iCh)
    );

    -- register data output
    PROCESS (adclk_rec) IS
    BEGIN
      IF rising_edge(adclk_rec) THEN
        DATA(iCh) <= (Q1p(iCh), NOT Q1n(iCh), Q2p(iCh), NOT Q2n(iCh), Q3p(iCh), NOT Q3n(iCh),
                      Q4p(iCh), NOT Q4n(iCh), Q5p(iCh), NOT Q5n(iCh), Q6p(iCh), NOT Q6n(iCh));
      END IF;
    END PROCESS;
  END GENERATE;

END Behavioral;
