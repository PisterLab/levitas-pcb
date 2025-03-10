#!/usr/bin/env python3

import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200)

with open('data.md', 'w') as datafile:
    while True:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(data.decode('utf-8', errors='ignore'))
            datafile.write(data.decode('utf-8', errors='ignore'))
        datafile.flush()

# 9 seconds


