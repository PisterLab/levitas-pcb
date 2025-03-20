# LeviTAS PCB

This is a PCB test platform originally build for the LeviTAS project.

A Raspberry Pi Pico microcontroller controls four 0-240V high voltage outputs and can simultaneously read from four FDC2112 capacitance sensors. The architecture and code should be straightforward to re-engineered to add more channels.

![Full assembled PCB and enclosure](/media/assembly.jpg)

# Programming / Firmware

This is how to program the microcontroller.

## Introduction

This project uses the Raspberry Pi RP2350 microcontroller, which is mounted to a Raspberry Pi Pico 2 dev board that we solder onto the main project PCB. The dev board is cheap and convenient because in addition to the RP2350 itself it includes flash memory, an oscillator, and power circuitry that we'd otherwise have to include anyway. The programming is the same either way.

The RP2350 is uniquely good for our use case because we need (a) four I2C buses running in parallel to read FDC2112 capacitance sensors, for which we use the PIO modules available only on the RP2040 and RP2350, and (b) we'd like floating-point math support built into the hardware, which the RP2350 has but the RP2040 does not. An FPGA might accomplish roughly the same tasks but is much more expensive and complicated to use.

Programming the RP2350 microcontroller is officially supported in C++ or MicroPython. We use C++ to more easily write programs that satisfy tight timing constraints (e.g., making a control loop as fast as possible). See the Documentation section below for links to the official documentation.

Note we're specifically using the 2.0.0 version of the SDK (because that's what we started with and haven't bothered to update it). Updating this requires updating the pico-sdk subfolder, every mention of the SDK version in other configuration files, and the version of the picotool utility used to upload the software. To minimize compilation issues, we also specifically chose to put the SDK files inside this git repo instead of requiring them to be downloaded to an external directory and linked.

## Installation and Running

Use the following steps to set up a software build environment for the Pico firmware on Linux or Windows, or follow the [official getting started guide](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) for more generic instructions.

On Linux:
- Install gcc-arm-none-eabi compiler (arm-non-eabi-gcc on nixos)
- Install newlib
- Install cmake and gnumake
- Install picotool
- Install picotool udev rules
- In `firmware` folder, run `make build` to compile code and `make reload` to download and run (or `make` to do both).
- Use your favorite serial monitor to see data the Pico sends over USB.

On Windows:
- Install [Microsoft VS Code](https://code.visualstudio.com/) (This is preferable to VSCodium because the easiest way to read data from the Pico over USB is the Microsoft VS Code Serial Monitor extension, which doesn't work with VSCodium)
- Install the official [Raspberry Pi Pico Visual Studio Code extension](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico)
- Open the `firmware` folder in VS Code. The Pi Pico extension should automatically detect this as a Pico project, and enable several commands under its quick access menu (click the extension icon on the Activity Bar, then under "Project" should be "Debug Project", "Compile Project", "Run Project (USB)", etc).
- Use the following commands:
  - "Compile Project" will compile (only) the software and put the compiled executables in a `build` subfolder, but note that "Configure CMake" must be run first.
  - "Configure CMake" will create a `build` subfolder and prepare the compilation based on configuration information in the `CMakeLists.txt` file. This is run automatically when the folder is opened in VS Code if recognized by the Pico extension. NOTE: THIS MUST BE MANUALLY RE-RUN EVERY TIME YOU CHANGE `CMakeLists.txt` FOR THE CHANGES TO BE APPLIED!
  - "Run Project (USB)" will load the program onto a Rasberry Pi Pico 2 board connected over USB (specifically, by running the `picotool` command line utility). If the Pico is in BOOTSEL mode (achieved by holding down the BOOTSEL button before plugging in USB) it will probably work immediately. If not, uploading can still work if USB communication was enabled on the already-running Pico program (which it is for this software, as per `CMakeLists.txt`), though you might need to press "Run Project (USB)" twice.
  - "Debug Project" and "Flash Project (SWD)" do not work without use of the Raspberry Pi Debug Probe (which we don't have).
- To see data the Pico is sending over USB, open the "SERIAL MONITOR" tab on the lower VS Code panel that also has "TERMINAL", "DEBUG CONSOLE", etc., set to 115200 baud and the correct COM port. Note this requires `pico_enable_stdio_usb(levitas 1)` to be set in `CMakeLists.txt` and `stdio_init_all();` to be called in `main.c` (which they are).

## Notable Firmware files

Inside the `firmware` folder:

- `CMakeLists.txt` configures compilation, and must be manually changed to include new source code files or other settings (e.g., importing a new sublibrary for the Pico SDK)
- `src/main.c` is the main source code file and holds the main LeviTAS control loop.
  - by default, this only prints capacitive sensor readings over USB; high voltage is set to zero
  - uncomment code in the main loop to enable high voltage output
- `src/pio_i2c.c`, `src/pio_i2c.h`, and `src/pio_i2c.pio` together form a library that can run I2C communication via the Pico PIOs. `src/main.c` uses this library to run four I2C ports in parallel (to connect to four FDC2112 chips).
- the following test code is unused but can be enabled by changing `CMakeLists.txt` to make them the main project file instead of `main.c`:
  - `src/test_fdc2112_i2c.c` tests only two of the four connected FDC2112 chips by using the two hardware built-in I2C modules (instead of PIO)
  - `src/test_fdc2112_pio_i2c.c` tests all four connected FDC2112 chips using the PIO library, and is almost identical to code in `src/main.c`.
  - `src/test_max5715_spi.c` tests the MAX5715 DAC and high voltage output, and is almost identical to code in `src/main.c`.
  - `src/test_mpu6050_i2c.c` displays data the MPU6050 accelerometer on the PCB motherboard using one of the hardware built-in I2C modules.

Other files:
- `Makefile` is used only for compiling on Linux, not for VS Code
- `.vscode` and `pico_sdk_import.cmake` are used only for VS Code with the official Pico VS Code extension, not on Linux
- the `build` subfolder is automatically generated by cmake when compiling and contains compiled executables in several formats, including UF2.

## Data Collection

The `data-collection/collect-data.py` and `data-collection/graph-data.py` Python programs in the `data-collection` subfolder of the main git repo can be used to automatically read the data the microcontroller sends over USB, then graph it. This provides much more useful data than manually looking at the printed output in a serial monitor.

## Some Useful Notes

There are several ways to upload code to a RP2040 or RP2350 chip:
1. Put into BOOTSEL mode (on Pi Pico dev board, hold down BOOTSEL buton when connecting power/USB), at which point the built-in ROM makes it act like a USB mass storage device. Upload file. (To verify file uploaded correctly, use picotool)
2. Put into BOOTSEL mode, use picotool to upload file
3. If USB communication enabled, can use picotool to force exit into BOOTSEL mode (use "picotool load -f" or "-F") then reboot (convenient!). There's no need to press the BOOTSEL button anymore. Note that sometimes this doesn't work on the first try, but repeating the process usually works.
4. Use SWD debugging interface.
Using the BOOTSEL mode (options 1,2) is necessary on a new chip, but afterward, as long as a program with USB communication enabled is running, option 3 is much more convenient. This is what we generally use.

Notes on the C/C++ SDK (from PDF manual):
- Consists of a large number of CMake libraries (in pico-sdk/src/rp2_common), which must be linked in the per-project CMakeLists.txt
  - The default library used in most examples is pico_stdlib, which includes several other libraries
  - the include files are typically in pico-sdk/src/rp2_common/(CMake library name)/include/(pico, hardware, gpio, etc)/(header.h)
- Apparently has some ability to build/debug running on the host operating system instead of the RP2040?
- Definitions for dev boards are in pico-sdk/src-boards/include/boards/
- The PDF manual has a bunch of examples
- Standard stdin/stdout can use (chapter 2.7):
  - UART (easy) (enabled by default) (disable by calling pico_enable_stdio_uart in CMakeLists.txt)
  - a USB CDC ACM virtual serial port (via TinyUSB) (enable by linking pico_stdio_usb or calling pico_enable_stdio_usb in CMakeLists.txt)
        (this is generally my preference?)
  - serial wire debug link (semihosting)
  - Segger RTT
- Can use bi_decl() in C code to store information in compiled binaries that picotool can access (e.g., comments, pin numbers)

## Documentation

There is a large amount of good official documentation at the following links. The most useful documentation is noted.

https://www.raspberrypi.com/documentation/microcontrollers/

RP2350 microcontroller:
- https://www.raspberrypi.com/products/rp2350/
- (good details on RP2350 hardware/capabilities) https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf
- https://datasheets.raspberrypi.com/rp2350/hardware-design-with-rp2350.pdf
- https://datasheets.raspberrypi.com/rp2350/rp2350-product-brief.pdf

Pico and Pico 2 dev boards (which are almost identical):
- https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf
- (good software getting-started guide) https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf

Programming with C/C++:
- https://github.com/raspberrypi/pico-sdk
- (code examples) https://github.com/raspberrypi/pico-examples
- https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html
- https://www.raspberrypi.com/documentation/pico-sdk/index_doxygen.html
- (good software getting-started guide) https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf
- (very thorough reference) https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf

Picotool:
- An official (and open source) CLI tool to work with compiled binaries and communicate with boards when in BOOTSEL mode (using the PICOBOOT interface; see RP2040 datasheet). Used to load and verify programs.
- https://github.com/raspberrypi/picotool

Raspberry Pi Debug Probe:
- Device to use UART or SWD debugging. Optional. We don't have one.
- https://www.raspberrypi.com/products/debug-probe/
- https://www.raspberrypi.com/documentation/microcontrollers/debug-probe.html

# More pictures

![Stacked PCBs](/media/levitas_pcb_render.jpg)
![Daughterboard](/media/levitas_pcb_render-daughterboard-bottom.jpg)
