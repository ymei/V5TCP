#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

""" Perform digital threshold scan of the Topmetal-II- sensor
    one row at a time
    Control RIGOL function generator
"""

import time
import os
import sys
import subprocess
import shutil
import math
try:
    import usbtmc
except:
    import visa

def writeSleep(instr, string, sleeptime=0.5):
    instr.write(string)
    time.sleep(sleeptime)

def tailPulse(instr, xp=16, np=1024, sigma=0.001):
    amax = 16383
    vals=[0 for i in xrange(np)]
    for i in xrange(np):
        if i<xp:
            vals[i]=amax
        else:
            vals[i]=int(amax*(1-math.exp(-(i-xp)*sigma)))
        #print(vals[i])
    string = "DATA:DAC VOLATILE"
    for i in xrange(np):
        string +=(",%d"% vals[i])
    writeSleep(instr, string, 1.0)
    writeSleep(instr, "FUNC:USER VOLATILE")

def fungen(amp=0.015, freq=100):
    try:
        rm = visa.ResourceManager()         #start visa32.dll
        rm.list_resources()                 #list the connected instrument
        instr = rm.open_resource('USB0::0x1AB1::0x0588::DG1D131402088::INSTR')
    except:
        instr = usbtmc.Instrument(0x1ab1, 0x0588) # RIGOL TECHNOLOGIES,DG1022 ,DG1D131402088,,00.02.00.06.00.02.07
        instr.timeout=10
        # print(instr.ask("*IDN?"))

    writeSleep(instr, "OUTP:LOAD 50")
    tailPulse(instr, xp=512, np=1024, sigma=0.01)
    writeSleep(instr, "FREQ %d" % freq)
    writeSleep(instr, "VOLT:UNIT VPP") 
    writeSleep(instr, "VOLTage:LOW 0")
    writeSleep(instr, "VOLT:HIGH %f" % amp)
    writeSleep(instr, "OUTP ON")

    instr.close()

def main():
    fungen(0.015)
    col = 3
    dname="Gring15"
    if not os.path.exists(dname):
        os.mkdir(dname)
    for row in xrange(72):
        for i in xrange(300):
            vr8b = 0.450 + i * 0.001
            fname = "%s/r%02dv%03d.dat" % (dname, row, i)
            subprocess.call("./TMII-Digital 192.168.2.3 1024 %s 0 %d %d 0xf 0.695 0.987 1.650 0.800 0.600 0.0 0.0" % (fname, row, col), shell=True)
            time.sleep(0.1)
            subprocess.call("./TMII-Digital 192.168.2.3 1024 %s 30000 %d %d 0xf 0.695 0.987 %g 0.800 0.600 0.0 0.0" % (fname, row, col, vr8b), shell=True)

if __name__ == "__main__":
    sys.exit(main())
