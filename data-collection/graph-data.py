#!/usr/bin/env python3

# This script graphs data from a data file recorded by "collect-data.py".

import numpy as np
import matplotlib.pyplot as plt

data = []

with open('data.dat', 'r') as infile:
    for line in infile:
        values_raw = line.split()
        if len(values_raw) < 9:
            # sometimes the first or last line wasn't fully read if the
            # data collection program start/end didn't sync up with the
            # microcontroller, so ignore those lines
            continue
        values = []
        values.append(int(values_raw[0],0)) # time since program started in microseconds (integer)
        # WARNING: CAPACITIVE SENSORS 1,2,3,4 CORRESPOND TO HIGH VOLTAGE OUTPUTS 4,3,2,1
        # we don't have an exact correlation between sensor integer output and capacitance; requires empirical calibration
        values.append(int(values_raw[1],0)) # output of capacitive sensor 1 (integer)
        values.append(int(values_raw[2],0)) # output of capacitive sensor 2 (integer)
        values.append(int(values_raw[3],0)) # output of capacitive sensor 3 (integer)
        values.append(int(values_raw[4],0)) # output of capacitive sensor 4 (integer)
        # WARNING: CAPACITIVE SENSORS 1,2,3,4 CORRESPOND TO HIGH VOLTAGE OUTPUTS 4,3,2,1
        values.append(float(values_raw[5])) # high voltage setpoint of HV1 on PCB, in volts (float)
        values.append(float(values_raw[6])) # high voltage setpoint of HV2 on PCB, in volts (float)
        values.append(float(values_raw[7])) # high voltage setpoint of HV3 on PCB, in volts (float)
        values.append(float(values_raw[8])) # high voltage setpoint of HV4 on PCB, in volts (float)
        #print(values)
        data.append(values)
data = np.array(data)

times = np.linspace(0,9,data.shape[0]) # get time values
times = data[:,0]/1e6 # convert to seconds
times = times - times[0] # start at 0s

# plot sensor outputs and high voltage
fig, (ax1, ax2) = plt.subplots(2, 1)
ax1.plot(times, data[:,1], label='sensor 1')
ax1.plot(times, data[:,2], label='sensor 2')
ax1.plot(times, data[:,3], label='sensor 3')
ax1.plot(times, data[:,4], label='sensor 4')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Sensor value (high value = low capacitance)')
ax1.legend()

ax2.plot(times, data[:,5], label='hv 1')
ax2.plot(times, data[:,6], label='hv 2')
ax2.plot(times, data[:,7], label='hv 3')
ax2.plot(times, data[:,8], label='hv 4')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('High voltage setpoint (V)')
ax2.legend()

plt.show()

