# Bill of Materials

MOTHERBOARD

|--------------------+-----------+-----------------------------------------+----------+--------------------------|
| TOTAL QTY TO ORDER | PER BOARD | part                                    | COST PER | DigiKey part             |
|--------------------+-----------+-----------------------------------------+----------+--------------------------|
|                  5 |         1 | RP2350 Pico 2 board (SC1631)            | $5       | 2648-SC1631CT-ND         |
|--------------------+-----------+-----------------------------------------+----------+--------------------------|
|                    |         2 | 220ohm R thru-hole                      |          | S220QCT-ND               |
|                    |         1 | LED 5mm thru-hole white                 |          | BL-BZX3V4V-1-B02-ND      |
|                    |         1 | LED 5mm thru-hole red                   |          | 754-WP7113SURCK-ND       |
|--------------------+-----------+-----------------------------------------+----------+--------------------------|
|                    |         4 | 100nF 0603 SMD caps                     |          | 311-1523-1-ND            |
|                    |         3 | 4.7uF 0805 SMD caps                     |          | 1276-1244-1-ND           |
|--------------------+-----------+-----------------------------------------+----------+--------------------------|
|                  5 |         1 | MAX5715AAUD+T DAC                       | $14.6    | 175-MAX5715AAUD+TCT-ND   |
|--------------------+-----------+-----------------------------------------+----------+--------------------------|
|                  5 |         1 | HV264 opamp                             | $7.8     | 150-HV264TS-GCT-ND       |
|                    |         1 | 1N4004 diode thru-hole                  |          | 4878-1N4004CT-ND         |
|--------------------+-----------+-----------------------------------------+----------+--------------------------|
|                    |         4 | 5kohm, thru-hole, 400V                  |          | CF12JT4K70CT-ND          |
|                 12 |         4 | PTC fuse PTCCL05H100SBE                 | $0.99    | BC3368-ND                |
|--------------------+-----------+-----------------------------------------+----------+--------------------------|
|                  0 |         1 | Q025-5C power supply                    |          | -                        |
|                  5 |         1 | TVS diode P6KE6.8A-E3/54                | $0.32    | P6KE6.8A-E3/54GICT-ND    |
|                 10 |         1 | 1Mohm, thru-hole, 400V                  | $0.03    | RNV14FAL1M00             |
|                    |         1 | 10nF, thru-hole, 400V                   |          | 495-75587-1-ND           |
|--------------------+-----------+-----------------------------------------+----------+--------------------------|
|                  3 |         1 | Stemma MPU-6050 accelerometer breakout  | $12.95   | 1528-3886-ND             |
|--------------------+-----------+-----------------------------------------+----------+--------------------------|
|                  6 |         2 | ZIF socket                              | $24.49   | A305-ND                  |
|                    |        10 | jumpers                                 |          |                          |
|                    |         3 | 20x breakaway male header pins          |          | 2057-PH1-20-UA-ND        |
|                    |         3 | 20x female header pins                  |          | S7053-ND                 |
|--------------------+-----------+-----------------------------------------+----------+--------------------------|
|                    |         1 | solder paste?                           |          | 315-NC191AX35T5-ND       |
|                    |         1 | rubber feet?                            |          | 3M156065-ND              |
|--------------------+-----------+-----------------------------------------+----------+--------------------------|
|                    |         8 | 100nF 0603 SMD caps                     |          | 311-1523-1-ND            |
|                    |         8 | 4.7uF 0805 SMD caps                     |          | 1276-1244-1-ND           |
|--------------------+-----------+-----------------------------------------+----------+--------------------------|
|                 15 |         4 | cap sensor FDC2112QDNTRQ1               | $6.59    | 296-44560-1-ND           |
|                    |         8 | resistor SMD 4.7k 0603                  |          | 311-4.7KGRCT-ND          |
|                    |         8 | resistor SMD 470  0603                  |          | A129684CT-ND             |
|                    |         8 | resistor SMD 0  0603 (just in case)     |          | RMCF0603ZT0R00CT-ND      |
|                    |         4 | inductor SMD 18uH 0603 MLF1608C180KTA00 |          | 445-1028-1-ND            |
|                    |         4 | cap SMD 33pF 0603                       |          | 478-KGM15ACG1H330JTCT-ND |
|--------------------+-----------+-----------------------------------------+----------+--------------------------|



Generate PCB gerbers according to
https://www.pcbway.com/blog/help_center/Generate_Gerber_file_from_Kicad.html
- use copper, paste, silkscreen, mask, edge.cuts
- uncheck "use extended x2 format"
- drill file with "minimal header" and "suppress leading zeros"

When ordering:
- remove label option
- extra urgent



# Accelerometer

Attached for board leveling and reference data.

Using breakout board instead of individual chip because I don't trust my soldering for something that small.

Can use either Adafruit Stemma MPU-6050 breakout or Stemma ADXL345 breakout (same board size, slightly more expensive).

# Motherboard/daughterboard connection

We attach the sensor daughterboard with an IC ZIF socket (well, two sockets). We use this instead of standard 0.1" header pins and sockets because inserting or removing header pins takes significant force, which can significantly shake or warp the daughterboard and might break the fragile MEMS devices on top. Ultimately, of course, we want to build a levitation system capable of sustaining high external forces, but our initial prototypes may not immediately reach that goal. That said, the motherboard PCB has footprints such that it can be assembled with standard female header pins instead of ZIF sockets with no other change in functionality.

The specific ZIF sockets se use are 32-6554-11 "CONN IC DIP SOCKET ZIF 32POS GLD" or 32-6554-10 "CONN IC DIP SOCKET ZIF 32POS TIN" from Aries Electronics due to their relatively high availability and robust construction.

