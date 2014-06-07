--------------------------------------------------------------------------------
-- File       : gig_eth.vhd
-- Author     : Yuan Mei.
-- -----------------------------------------------------------------------------
-- (c) Copyright 2014 Yuan Mei. All rights reserved.
-- -----------------------------------------------------------------------------
-- Description:
--   Connect V5 EMAC TO TCP server COM5402 and package as a single module            
--------------------------------------------------------------------------------

LIBRARY UNISIM;
USE UNISIM.VComponents.ALL;

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
USE work.com5402pkg.ALL;

ENTITY gig_eth IS
  PORT (
    -- asynchronous reset
    RESET                : IN  std_logic;
    -- clocks
    CLK125               : IN  std_logic;  -- 125MHz
    REFCLK               : IN  std_logic;  -- 200MHz
    -- GMII Interface - EMAC0
    GMII_TXD_0           : OUT std_logic_vector(7 DOWNTO 0);
    GMII_TX_EN_0         : OUT std_logic;
    GMII_TX_ER_0         : OUT std_logic;
    GMII_TX_CLK_0        : OUT std_logic;
    GMII_RXD_0           : IN  std_logic_vector(7 DOWNTO 0);
    GMII_RX_DV_0         : IN  std_logic;
    GMII_RX_ER_0         : IN  std_logic;
    GMII_RX_CLK_0        : IN  std_logic;
    MII_TX_CLK_0         : IN  std_logic;
    GMII_COL_0           : IN  std_logic;
    GMII_CRS_0           : IN  std_logic;
    PHY_RST_n            : OUT std_logic;
    -- TCP
    TCP_CONNECTION_RESET : IN  std_logic;
    TX_TDATA             : IN  std_logic_vector(7 DOWNTO 0);
    TX_TVALID            : IN  std_logic;
    TX_TREADY            : OUT std_logic;
    RX_TDATA             : OUT std_logic_vector(7 DOWNTO 0);
    RX_TVALID            : OUT std_logic;
    RX_TREADY            : IN  std_logic;
    -- FIFO
    TCP_USE_FIFO         : IN  std_logic;
    TX_FIFO_WRCLK        : IN  std_logic;
    TX_FIFO_Q            : IN  std_logic_vector(31 DOWNTO 0);
    TX_FIFO_WREN         : IN  std_logic;
    TX_FIFO_FULL         : OUT std_logic;
    RX_FIFO_RDCLK        : IN  std_logic;
    RX_FIFO_Q            : OUT std_logic_vector(31 DOWNTO 0);
    RX_FIFO_RDEN         : IN  std_logic;
    RX_FIFO_EMPTY        : OUT std_logic
  );
END gig_eth;

ARCHITECTURE wrapper OF gig_eth IS

  COMPONENT emac
    PORT (
      -- 125MHz clock for MAC
      GTX_CLK_0         : IN  std_logic;
      -- 200MHz reference clock for RGMII IODELAYs
      REFCLK            : IN  std_logic;
      -- Asynchronous Reset
      RESET             : IN  std_logic;
      -- GMII Interface - EMAC0
      GMII_TXD_0        : OUT std_logic_vector(7 DOWNTO 0);
      GMII_TX_EN_0      : OUT std_logic;
      GMII_TX_ER_0      : OUT std_logic;
      GMII_TX_CLK_0     : OUT std_logic;
      GMII_RXD_0        : IN  std_logic_vector(7 DOWNTO 0);
      GMII_RX_DV_0      : IN  std_logic;
      GMII_RX_ER_0      : IN  std_logic;
      GMII_RX_CLK_0     : IN  std_logic;
      MII_TX_CLK_0      : IN  std_logic;
      GMII_COL_0        : IN  std_logic;
      GMII_CRS_0        : IN  std_logic;
      PHY_RST_n         : OUT std_logic;
      -- Local link Receiver Interface - EMAC0
      RX_LL_DATA_0      : OUT std_logic_vector(7 DOWNTO 0);
      RX_LL_SOF_N_0     : OUT std_logic;
      RX_LL_EOF_N_0     : OUT std_logic;
      RX_LL_SRC_RDY_N_0 : OUT std_logic;
      RX_LL_DST_RDY_N_0 : IN  std_logic;
      -- Local link Transmitter Interface - EMAC0
      TX_LL_DATA_0      : IN  std_logic_vector(7 DOWNTO 0);
      TX_LL_SOF_N_0     : IN  std_logic;
      TX_LL_EOF_N_0     : IN  std_logic;
      TX_LL_SRC_RDY_N_0 : IN  std_logic;
      TX_LL_DST_RDY_N_0 : OUT std_logic
    );
  END COMPONENT;

  COMPONENT COM5402 IS
    GENERIC (
      CLK_FREQUENCY   : integer               := 56;
      -- CLK frequency in MHz. Needed to compute actual delays.
      TX_IDLE_TIMEOUT : integer RANGE 0 TO 50 := 50;
      -- inactive input timeout, expressed in 4us units. -- 50*4us = 200us 
      -- Controls the transmit stream segmentation: data in the elastic buffer will be transmitted if
      -- no input is received within TX_IDLE_TIMEOUT, without waiting for the transmit frame to be filled with MSS data bytes.
      SIMULATION      : std_logic             := '0'
     -- 1 during simulation with Wireshark .cap file, '0' otherwise
     -- Wireshark many not be able to collect offloaded checksum computations.
     -- when SIMULATION =  '1': (a) IP header checksum is valid if 0000,
     -- (b) TCP checksum computation is forced to a valid 00001 irrespective of the 16-bit checksum
     -- captured by Wireshark.
      );
    PORT (
      --//-- CLK, RESET
      CLK         : IN std_logic;
      -- All signals are synchronous with CLK
      -- CLK must be a global clock 125 MHz or faster to match the Gbps MAC speed.
      ASYNC_RESET : IN std_logic;  -- to be phased out. replace with SYNC_RESET
      SYNC_RESET  : IN std_logic;

      --//-- CONFIGURATION
      -- configuration signals are synchonous with CLK
      -- Synchronous with CLK clock.
      MAC_ADDR        : IN std_logic_vector(47 DOWNTO 0);
      IPv4_ADDR       : IN std_logic_vector(31 DOWNTO 0);
      IPv6_ADDR       : IN std_logic_vector(127 DOWNTO 0);
      SUBNET_MASK     : IN std_logic_vector(31 DOWNTO 0);
      GATEWAY_IP_ADDR : IN std_logic_vector(31 DOWNTO 0);
      -- local IP address. 4 bytes for IPv4, 16 bytes for IPv6
      -- Natural order (MSB) 172.16.1.128 (LSB) as transmitted in the IP frame.

      --// User-initiated connection reset for stream I
      CONNECTION_RESET : IN std_logic_vector((NTCPSTREAMS-1) DOWNTO 0);

      --//-- Protocol -> Transmit MAC Interface
      -- 32-bit CRC is automatically appended by the MAC layer. User should not supply it.
      -- Synchonous with the user-side CLK
      MAC_TX_DATA       : OUT std_logic_vector(7 DOWNTO 0);
      -- MAC reads the data at the rising edge of CLK when MAC_TX_DATA_VALID = '1'
      MAC_TX_DATA_VALID : OUT std_logic;
      -- data valid
      MAC_TX_SOF        : OUT std_logic;
      -- start of frame: '1' when sending the first byte.
      MAC_TX_EOF        : OUT std_logic;
      -- '1' when sending the last byte in a packet to be transmitted. 
      -- Aligned with MAC_TX_DATA_VALID
      MAC_TX_CTS        : IN  std_logic;
      -- MAC-generated Clear To Send flow control signal, indicating room in the 
      -- MAC tx elastic buffer for a complete maximum size frame 1518B. 
      -- The user should check that this signal is high before deciding to send
      -- sending the next frame. 
      -- Note: MAC_TX_CTS may go low while the frame is transfered in. Ignore it as space is guaranteed
      -- at the start of frame.

      --//-- Receive MAC -> Protocol
      -- Valid rx packets only: packets with bad CRC or invalid address are discarded.
      -- The 32-bit CRC is always removed by the MAC layer.
      -- Synchonous with the user-side CLK
      MAC_RX_DATA       : IN std_logic_vector(7 DOWNTO 0);
      -- USER reads the data at the rising edge of CLK when MAC_RX_DATA_VALID = '1'
      MAC_RX_DATA_VALID : IN std_logic;
      -- data valid
      MAC_RX_SOF        : IN std_logic;
      -- '1' when sending the first byte in a received packet. 
      -- Aligned with MAC_RX_DATA_VALID
      MAC_RX_EOF        : IN std_logic;
      -- '1' when sending the last byte in a received packet. 
      -- Aligned with MAC_RX_DATA_VALID

      --//-- Application <- UDP rx
      UDP_RX_DATA         : OUT std_logic_vector(7 DOWNTO 0);
      UDP_RX_DATA_VALID   : OUT std_logic;
      UDP_RX_SOF          : OUT std_logic;
      UDP_RX_EOF          : OUT std_logic;
      -- 1 CLK pulse indicating that UDP_RX_DATA is the last byte in the UDP data field.
      -- ALWAYS CHECK UDP_RX_DATA_VALID at the end of packet (UDP_RX_EOF = '1') to confirm
      -- that the UDP packet is valid. External buffer may have to backtrack to the the last
      -- valid pointer to discard an invalid UDP packet.
      -- Reason: we only knows about bad UDP packets at the end.
      UDP_RX_DEST_PORT_NO : IN  std_logic_vector(15 DOWNTO 0);

      --//-- Application -> UDP tx
      UDP_TX_DATA           : IN  std_logic_vector(7 DOWNTO 0);
      UDP_TX_DATA_VALID     : IN  std_logic;
      UDP_TX_SOF            : IN  std_logic;  -- 1 CLK-wide pulse to mark the first byte in the tx UDP frame
      UDP_TX_EOF            : IN  std_logic;  -- 1 CLK-wide pulse to mark the last byte in the tx UDP frame
      UDP_TX_CTS            : OUT std_logic;
      UDP_TX_ACK            : OUT std_logic;  -- 1 CLK-wide pulse indicating that the previous UDP frame is being sent
      UDP_TX_NAK            : OUT std_logic;  -- 1 CLK-wide pulse indicating that the previous UDP frame could not be sent
      UDP_TX_DEST_IP_ADDR   : IN  std_logic_vector(127 DOWNTO 0);
      UDP_TX_DEST_PORT_NO   : IN  std_logic_vector(15 DOWNTO 0);
      UDP_TX_SOURCE_PORT_NO : IN  std_logic_vector(15 DOWNTO 0);

      --//-- Application <- TCP rx
      -- NTCPSTREAMS can operate independently. Only one stream active at any given time.
      -- Data is pushed out. Limited flow-control here. Receipient must be able to accept data
      -- at any time (in other words, it is the receipient's responsibility to have elastic 
      -- buffer if needed).
      TCP_RX_DATA       : OUT SLV8xNTCPSTREAMStype;
      TCP_RX_DATA_VALID : OUT std_logic_vector((NTCPSTREAMS-1) DOWNTO 0);
      TCP_RX_RTS        : OUT std_logic;
      TCP_RX_CTS        : IN  std_logic;
      -- Optional Clear-To-Send. pull to '1' when output flow control is unused.
      -- WARNING: pulling CTS down will stop the flow for ALL streams.

      --//-- Application -> TCP tx
      -- NTCPSTREAMS can operate independently and concurrently. No scheduling arbitration needed here.
      TCP_TX_DATA       : IN  SLV8xNTCPSTREAMStype;
      TCP_TX_DATA_VALID : IN  std_logic_vector((NTCPSTREAMS-1) DOWNTO 0);
      TCP_TX_CTS        : OUT std_logic_vector((NTCPSTREAMS-1) DOWNTO 0);
      -- Clear To Send = transmit flow control. 
      -- App is responsible for checking the CTS signal before sending APP_DATA

      --//-- TEST POINTS, COMSCOPE TRACES
      CS1     : OUT std_logic_vector(7 DOWNTO 0);
      CS1_CLK : OUT std_logic;
      CS2     : OUT std_logic_vector(7 DOWNTO 0);
      CS2_CLK : OUT std_logic;
      TP      : OUT std_logic_vector(10 DOWNTO 1)
    );
  END COMPONENT;

  COMPONENT fifo8to32
    PORT (
      rst    : IN  std_logic;
      wr_clk : IN  std_logic;
      rd_clk : IN  std_logic;
      din    : IN  std_logic_vector(7 DOWNTO 0);
      wr_en  : IN  std_logic;
      rd_en  : IN  std_logic;
      dout   : OUT std_logic_vector(31 DOWNTO 0);
      full   : OUT std_logic;
      empty  : OUT std_logic
    );
  END COMPONENT;

  COMPONENT fifo32to8
    PORT (
      rst    : IN  std_logic;
      wr_clk : IN  std_logic;
      rd_clk : IN  std_logic;
      din    : IN  std_logic_vector(31 DOWNTO 0);
      wr_en  : IN  std_logic;
      rd_en  : IN  std_logic;
      dout   : OUT std_logic_vector(7 DOWNTO 0);
      full   : OUT std_logic;
      empty  : OUT std_logic
    );
  END COMPONENT;

  ------------------------------------------------------------------------------
  -- internal signals used in this top level wrapper.
  ------------------------------------------------------------------------------

  -- emac
  SIGNAL emac_ll_tx_sof           : std_logic;
  SIGNAL emac_ll_tx_eof           : std_logic;
  SIGNAL emac_ll_tx_src_rdy       : std_logic;
  SIGNAL emac_ll_tx_dst_rdy       : std_logic;
  SIGNAL emac_ll_tx_data          : std_logic_vector (7 DOWNTO 0);
  SIGNAL emac_ll_rx_sof           : std_logic;
  SIGNAL emac_ll_rx_eof           : std_logic;
  SIGNAL emac_ll_rx_src_rdy       : std_logic;
  SIGNAL emac_ll_rx_dst_rdy       : std_logic;
  SIGNAL emac_ll_rx_data          : std_logic_vector (7 DOWNTO 0);
  -- tcp
  SIGNAL connection_reset_i       : std_logic_vector((NTCPSTREAMS-1) DOWNTO 0);
  SIGNAL tcp_tx_data_valid_i      : std_logic_vector((NTCPSTREAMS-1) DOWNTO 0);
  --
  SIGNAL tcp_rx_data              : std_logic_vector(7 DOWNTO 0);
  SIGNAL tcp_rx_data_valid        : std_logic;
  SIGNAL tcp_rx_rts               : std_logic;
  SIGNAL tcp_rx_cts               : std_logic;
  SIGNAL tcp_tx_data              : std_logic_vector(7 DOWNTO 0);
  SIGNAL tcp_tx_data_valid        : std_logic;
  SIGNAL tcp_tx_cts               : std_logic;
  --
  SIGNAL tcp_rx_data_slv8x        : SLV8xNTCPSTREAMStype;
  SIGNAL tcp_tx_data_slv8x        : SLV8xNTCPSTREAMStype;
  SIGNAL tcp_rx_data_valid_vector : std_logic_vector((NTCPSTREAMS-1) DOWNTO 0);
  SIGNAL tcp_tx_cts_vector        : std_logic_vector((NTCPSTREAMS-1) DOWNTO 0);
  --
  SIGNAL rx_fifo_full             : std_logic;
  SIGNAL tx_fifo_dout             : std_logic_vector(7 DOWNTO 0);
  SIGNAL tx_fifo_rden             : std_logic;
  SIGNAL tx_fifo_empty            : std_logic;

  ------------------------------------------------------------------------------
  -- Begin architecture
  ------------------------------------------------------------------------------

BEGIN

  ---------------------------------------------< EMAC
  emac_inst : emac
    PORT MAP (
      -- 125MHz clock for MAC
      GTX_CLK_0         => CLK125,
      -- 200MHz reference clock for RGMII IODELAYs
      REFCLK            => REFCLK,
      -- Asynchronous Reset
      RESET             => RESET,
      -- GMII Interface - EMAC0
      GMII_TXD_0        => GMII_TXD_0,
      GMII_TX_EN_0      => GMII_TX_EN_0,
      GMII_TX_ER_0      => GMII_TX_ER_0,
      GMII_TX_CLK_0     => GMII_TX_CLK_0,
      GMII_RXD_0        => GMII_RXD_0,
      GMII_RX_DV_0      => GMII_RX_DV_0,
      GMII_RX_ER_0      => GMII_RX_ER_0,
      GMII_RX_CLK_0     => GMII_RX_CLK_0,
      MII_TX_CLK_0      => MII_TX_CLK_0,
      GMII_COL_0        => GMII_COL_0,
      GMII_CRS_0        => GMII_CRS_0,
      PHY_RST_n         => PHY_RST_n,
      -- Local link Receiver Interface - EMAC0
      RX_LL_DATA_0      => emac_ll_rx_data,
      RX_LL_SOF_N_0     => emac_ll_rx_sof,
      RX_LL_EOF_N_0     => emac_ll_rx_eof,
      RX_LL_SRC_RDY_N_0 => emac_ll_rx_src_rdy,
      RX_LL_DST_RDY_N_0 => emac_ll_rx_dst_rdy,
      -- Local link Transmitter Interface - EMAC0
      TX_LL_DATA_0      => emac_ll_tx_data,
      TX_LL_SOF_N_0     => NOT emac_ll_tx_sof,
      TX_LL_EOF_N_0     => NOT emac_ll_tx_eof,
      TX_LL_SRC_RDY_N_0 => NOT emac_ll_tx_src_rdy,
      TX_LL_DST_RDY_N_0 => emac_ll_tx_dst_rdy
    );
  emac_ll_rx_dst_rdy <= '0';            -- '0' is enabled
  ---------------------------------------------> EMAC

  ---------------------------------------------< tcp_server
  tcp_rx_data          <= tcp_rx_data_slv8x(0);
  tcp_tx_data_slv8x(0) <= tcp_tx_data;
  tcp_tx_cts           <= tcp_tx_cts_vector(0);
  tcp_rx_data_valid    <= tcp_rx_data_valid_vector(0);
  connection_reset_i   <= (OTHERS => TCP_CONNECTION_RESET);
  tcp_tx_data_valid_i  <= (OTHERS => tcp_tx_data_valid);
  tcp_server_inst : COM5402
    GENERIC MAP (
      CLK_FREQUENCY   => 125,
      -- CLK frequency in MHz. Needed to compute actual delays.
      TX_IDLE_TIMEOUT => 50,
      -- inactive input timeout, expressed in 4us units. -- 50*4us = 200us 
      -- Controls the transmit stream segmentation: data in the elastic buffer will be transmitted if
      -- no input is received within TX_IDLE_TIMEOUT, without waiting for the transmit frame to be filled with MSS data bytes.       
      SIMULATION      => '0'
     -- 1 during simulation with Wireshark .cap file, '0' otherwise
     -- Wireshark many not be able to collect offloaded checksum computations.
     -- when SIMULATION =  '1': (a) IP header checksum is valid if 0000,
     -- (b) TCP checksum computation is forced to a valid 00001 irrespective of the 16-bit checksum
     -- captured by Wireshark.
    )
    PORT MAP (
      --//-- CLK, RESET
      CLK         => CLK125,
      -- All signals are synchronous with CLK
      -- CLK must be a global clock 125 MHz or faster to match the Gbps MAC speed.
      ASYNC_RESET => RESET,  -- to be phased out. replace with SYNC_RESET
      SYNC_RESET  => RESET,

      --//-- CONFIGURATION
      -- configuration signals are synchonous with CLK
      -- Synchronous with CLK clock.
      MAC_ADDR        => x"00183e010f00",
      IPv4_ADDR       => x"c0a80202",
      IPv6_ADDR       => (OTHERS => '0'),
      SUBNET_MASK     => x"ffffff00",
      GATEWAY_IP_ADDR => x"c0a80201",
      -- local IP address. 4 bytes for IPv4, 16 bytes for IPv6
      -- Natural order (MSB) 172.16.1.128 (LSB) as transmitted in the IP frame.

      --// User-initiated connection reset for stream I
      CONNECTION_RESET => connection_reset_i,

      --//-- Protocol -> Transmit MAC Interface
      -- 32-bit CRC is automatically appended by the MAC layer. User should not supply it.
      -- Synchonous with the user-side CLK
      MAC_TX_DATA       => emac_ll_tx_data,
      -- MAC reads the data at the rising edge of CLK when MAC_TX_DATA_VALID = '1'
      MAC_TX_DATA_VALID => emac_ll_tx_src_rdy,
      -- data valid
      MAC_TX_SOF        => emac_ll_tx_sof,
      -- start of frame: '1' when sending the first byte.       
      MAC_TX_EOF        => emac_ll_tx_eof,
      -- '1' when sending the last byte in a packet to be transmitted. 
      -- Aligned with MAC_TX_DATA_VALID
      MAC_TX_CTS        => NOT emac_ll_tx_dst_rdy,
      -- MAC-generated Clear To Send flow control signal, indicating room in the 
      -- MAC tx elastic buffer for a complete maximum size frame 1518B. 
      -- The user should check that this signal is high before deciding to send
      -- sending the next frame. 
      -- Note: MAC_TX_CTS may go low while the frame is transfered in. Ignore it as space is guaranteed
      -- at the start of frame.

      --//-- Receive MAC -> Protocol
      -- Valid rx packets only: packets with bad CRC or invalid address are discarded.
      -- The 32-bit CRC is always removed by the MAC layer.
      -- Synchonous with the user-side CLK
      MAC_RX_DATA       => emac_ll_rx_data,
      -- USER reads the data at the rising edge of CLK when MAC_RX_DATA_VALID = '1'
      MAC_RX_DATA_VALID => NOT emac_ll_rx_src_rdy,
      -- data valid
      MAC_RX_SOF        => NOT emac_ll_rx_sof,
      -- '1' when sending the first byte in a received packet. 
      -- Aligned with MAC_RX_DATA_VALID
      MAC_RX_EOF        => NOT emac_ll_rx_eof,
      -- '1' when sending the last byte in a received packet. 
      -- Aligned with MAC_RX_DATA_VALID

      --//-- Application <- UDP rx
      UDP_RX_DATA         => OPEN,
      UDP_RX_DATA_VALID   => OPEN,
      UDP_RX_SOF          => OPEN,
      UDP_RX_EOF          => OPEN,
      -- 1 CLK pulse indicating that UDP_RX_DATA is the last byte in the UDP data field.
      -- ALWAYS CHECK UDP_RX_DATA_VALID at the end of packet (UDP_RX_EOF = '1') to confirm
      -- that the UDP packet is valid. External buffer may have to backtrack to the the last
      -- valid pointer to discard an invalid UDP packet.
      -- Reason: we only knows about bad UDP packets at the end.
      UDP_RX_DEST_PORT_NO => (OTHERS => '0'),

      --//-- Application -> UDP tx
      UDP_TX_DATA           => (OTHERS => '0'),
      UDP_TX_DATA_VALID     => '0',
      UDP_TX_SOF            => '0',  -- 1 CLK-wide pulse to mark the first byte in the tx UDP frame
      UDP_TX_EOF            => '0',  -- 1 CLK-wide pulse to mark the last byte in the tx UDP frame
      UDP_TX_CTS            => OPEN,
      UDP_TX_ACK            => OPEN,  -- 1 CLK-wide pulse indicating that the previous UDP frame is being sent
      UDP_TX_NAK            => OPEN,  -- 1 CLK-wide pulse indicating that the previous UDP frame could not be sent
      UDP_TX_DEST_IP_ADDR   => (OTHERS => '0'),
      UDP_TX_DEST_PORT_NO   => (OTHERS => '0'),
      UDP_TX_SOURCE_PORT_NO => (OTHERS => '0'),

      --//-- Application <- TCP rx
      -- NTCPSTREAMS can operate independently. Only one stream active at any given time.
      -- Data is pushed out. Limited flow-control here. Receipient must be able to accept data
      -- at any time (in other words, it is the receipient's responsibility to have elastic 
      -- buffer if needed).
      TCP_RX_DATA       => tcp_rx_data_slv8x,
      TCP_RX_DATA_VALID => tcp_rx_data_valid_vector,
      TCP_RX_RTS        => tcp_rx_rts,
      TCP_RX_CTS        => tcp_rx_cts,
      -- Optional Clear-To-Send. pull to '1' when output flow control is unused.
      -- WARNING: pulling CTS down will stop the flow for ALL streams.

      --//-- Application -> TCP tx
      -- NTCPSTREAMS can operate independently and concurrently. No scheduling arbitration needed here.
      TCP_TX_DATA       => tcp_tx_data_slv8x,
      TCP_TX_DATA_VALID => tcp_tx_data_valid_i,
      TCP_TX_CTS        => tcp_tx_cts_vector,
      -- Clear To Send = transmit flow control. 
      -- App is responsible for checking the CTS signal before sending APP_DATA

      --//-- TEST POINTS, COMSCOPE TRACES
      CS1     => OPEN,
      CS1_CLK => OPEN,
      CS2     => OPEN,
      CS2_CLK => OPEN,
      TP      => OPEN
    );
  ---------------------------------------------> tcp_server
  rx_fifo_inst : fifo8to32
    PORT MAP (
      rst    => RESET,
      wr_clk => CLK125,
      rd_clk => RX_FIFO_RDCLK,
      din    => tcp_rx_data,
      wr_en  => tcp_rx_data_valid,
      rd_en  => RX_FIFO_RDEN,
      dout   => RX_FIFO_Q,
      full   => rx_fifo_full,
      empty  => RX_FIFO_EMPTY
    );
  tcp_rx_cts <= (NOT rx_fifo_full) WHEN TCP_USE_FIFO = '1' ELSE
                RX_TREADY;
  RX_TDATA  <= tcp_rx_data;
  RX_TVALID <= tcp_rx_data_valid;

  tx_fifo_inst : fifo32to8
    PORT MAP (
      rst    => RESET,
      wr_clk => TX_FIFO_WRCLK,
      rd_clk => CLK125,
      din    => TX_FIFO_Q,
      wr_en  => TX_FIFO_WREN,
      rd_en  => tx_fifo_rden,
      dout   => tx_fifo_dout,
      full   => TX_FIFO_FULL,
      empty  => tx_fifo_empty
    );
  tcp_tx_data_valid <= ((NOT tx_fifo_empty) AND tcp_tx_cts) WHEN TCP_USE_FIFO = '1' ELSE
                       TX_TVALID;
  tx_fifo_rden <= tcp_tx_data_valid;

  tcp_tx_data <= tx_fifo_dout WHEN TCP_USE_FIFO = '1' ELSE
                 TX_TDATA;
  TX_TREADY <= tcp_tx_cts;

END wrapper;
