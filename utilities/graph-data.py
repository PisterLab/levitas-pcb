#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

data = []

with open('data.dat', 'r') as infile:
    for line in infile:
        values = [int(x, 0) for x in line.split()]
        #print(values)
        data.append(values)
data = np.array(data)

times = np.linspace(0,9,data.shape[0])
# length is 55074 values in an estimated 9 seconds -> about 163us resolution, or 6.1kHz

print(data.shape)

plt.plot(times, data[:,0], label='sensor 1')
plt.plot(times, data[:,1], label='sensor 2')
plt.plot(times, data[:,2], label='sensor 3')
plt.plot(times, data[:,3], label='sensor 4')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Sensor value (∝1/capacitance)')
plt.show()

# over 9 seconds
