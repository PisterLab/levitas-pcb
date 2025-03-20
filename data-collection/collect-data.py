#!/usr/bin/env python3

# This script records data sent by the microcontroller over USB.
#
# Writes data to file 'data.dat' in current directory (be careful
# not to unintentionally overwrite old data).
#
# Runs in an infinite loop. When you have collected enough data,
# manually stop the program (e.g., with "Ctrl+C" on most terminals).
#
# NOTE: change the serial port from /dev/ttyACM0 to the correct port.

import serial # "pyserial" module
import time

ser = serial.Serial('/dev/ttyACM0', 115200)

with open('data.dat', 'w') as datafile:
    while True:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(data.decode('utf-8', errors='ignore'))
            datafile.write(data.decode('utf-8', errors='ignore'))
        datafile.flush()

