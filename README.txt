Genesys Virtex-5 1-gigabit ethernet (TCP) control board
--------------------------------------------------------------------------------
Run the entire synthesis / build on command line:
Create a top-level directory, in parallel to src and ipcore_dir, cd
into it, then type:
/opt/Xilinx/14.7/ISE_DS/ISE/bin/lin64/xtclsh ../src/V5TCP.tcl rebuild_project
--------------------------------------------------------------------------------
Generating a PROM file (MCS):
In iMPACT, select BPI Flash Configure Single FPGA
Virtex5 32M, MCS, x16, no extra data
BPI PROM, 28F256P30, 16 bit
Erase before programming

Mode switch: M2 M1 M0
0 1 1 BPID
0 1 0 BPIU <- use this one
1 0 1 JTAG
--------------------------------------------------------------------------------
