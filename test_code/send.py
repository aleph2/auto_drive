#!/usr/bin/python

import sys
import serial
from struct import *
import time

ser = serial.Serial('/dev/tty.usbserial-FTYMD4NJ1', 9600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
#print 'you input is:' + chr(int(sys.argv[1]))
print 'in hex format'
tmp = int(sys.argv[1])
tmp1 = int(sys.argv[2])
print list(bin(tmp))[2:]
packed_bytes = unpack('H', pack('h', tmp))
print pack('BB', tmp, tmp1)
ser.write(pack('BB', tmp, tmp1))
#if (tmp1 == 2):
#    for i in range(128):
#        ser.write(pack('BB', i, tmp1))
#        time.sleep(0.001)
#        print i,
#else:
#    ser.write(pack('BB', tmp, tmp1))
