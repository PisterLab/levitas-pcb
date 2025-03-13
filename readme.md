# LeviTAS PCB

This is a PCB test platform originally build for the LeviTAS project.

A Raspberry Pi Pico microcontroller controls four 0-240V high voltage outputs and can simultaneously read from four FDC2112 capacitance sensors. The architecture and code should be straightforward to re-engineered to add more channels.

![Full assembled PCB and enclosure](/media/assembly.jpg)

# Programming / Firmware

This is how to program the microcontroller.

## Introduction

This project uses the Raspberry Pi RP2350 microcontroller, which is mounted to a Raspberry Pi Pico 2 dev board that we solder onto the main project PCB. The dev board is cheap and convenient because in addition to the RP2350 itself it includes flash memory, an oscillator, and power circuitry that we'd otherwise have to include anyway. The programming is the same either way.

Programming the RP2350 microcontroller is officially supported in C++ or MicroPython. We use C++ to more easily write programs that satisfy tight timing constraints (e.g., making a control loop as fast as possible). See the Documentation section below for links to the official documentation.

## Documentation

https://www.raspberrypi.com/documentation/microcontrollers/

RP2350 microcontroller:
- https://www.raspberrypi.com/products/rp2350/
- https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf
  ^^^This one is useful.
- https://datasheets.raspberrypi.com/rp2350/hardware-design-with-rp2350.pdf
- https://datasheets.raspberrypi.com/rp2350/rp2350-product-brief.pdf

Pico and Pico 2 dev boards (which are almost identical):
- https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf
- https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf
  ^^^ Basic getting-started guide using C/C++

Programming with C/C++ (note: I like this method):
https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html
https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf
^^^ Basic getting-started guide using C/C++.
https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf
^^^ Very thorough.
https://www.raspberrypi.com/documentation/pico-sdk/index_doxygen.html
https://github.com/raspberrypi/pico-sdk
https://github.com/raspberrypi/pico-examples

Picotool:
An official (and open source) CLI tool to work with compiled binaries and communicate with boards when in BOOTSEL mode (using the PICOBOOT interface; see RP2040 datasheet). Can be used to load and verify programs.
https://github.com/raspberrypi/picotool

Raspberry Pi Debug Probe:
Device to use UART or SWD debugging. Optional, but works well with RP2040.
https://www.raspberrypi.com/products/debug-probe/
https://www.raspberrypi.com/documentation/microcontrollers/debug-probe.html

## Installation

On Linux:
- install gcc-arm-none-eabi compiler (arm-non-eabi-gcc on nixos)
- install newlib (?)
- install cmake and gnumake
- install picotool
- install picotool udev rules
- in main code repo, download Pico SDK https://github.com/raspberrypi/pico-sdk
    - SDK might expect picotool version to match
    - make sure to download TinyUSB git submodule (git submodule update --init)
      (pico-sdk/lib/btstack, cyw43-driver, lwip, mbedtls, tinyusb)
    - (guess: for reliability, after downloading all this, remove git and commit everything directly in main project?)
    - example:
        cd main_project_dir
        git clone --depth 1 --branch 2.0.0 https://github.com/raspberrypi/pico-sdk
        cd pico-sdk
        git submodule update --init
        rm -rf .git
        cd ..
        chown root:users -R pico-sdk
- follow Pico SDK README instructions

On Windows:
- follow getting started guide: https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf
- install VSCode or VSCodium
- install the official (Raspberry Pi Pico Visual Studio Code extension)[https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico]

## Notes

How to upload code to RP2040:
- Put into BOOTSEL mode, at which point ROM makes it act like a USB mass storage device. Upload file. (To verify file uploaded correctly, use picotool)
- Put into BOOTSEL mode, use picotool to upload file
- If USB communication enabled, can use picotool to force exit into BOOTSEL mode (use "picotool load -f" or "-F") then reboot (convenient!)
- Use SWD debugging interface.
To put chip into BOOTSEL mode when booted, on Pi Pico dev board hold down BOOTSEL buton when connecting power/USB.
GUESS: picotool is probably preferred. Use BOOTSEL mode at first or if need be, and switch to flash picotool communication afterward?

Notes on C/C++ SDK (from PDF manual):
- Consists of a large number of CMake libraries (in pico-sdk/src/rp2_common), which must be linked in the per-project CMakeLists.txt
  - The default library used in most examples is pico_stdlib, which includes several other libraries
  - the include files are typically in pico-sdk/src/rp2_common/(CMake library name)/include/(pico, hardware, gpio, etc)/(header.h)
- Apparently has some ability to build/debug running on the host operating system instead of the RP2040?
- Definitions for dev boards are in pico-sdk/src-boards/include/boards/
- The PDF manual has a bunch of examples
- Standard stdin/stdout can use (chapter 2.7):
    UART (easy) (enabled by default) (disable by calling pico_enable_stdio_uart in CMakeLists.txt)
    a USB CDC ACM virtual serial port (via TinyUSB) (enable by linking pico_stdio_usb or calling pico_enable_stdio_usb in CMakeLists.txt)
        (this is generally my preference?)
    serial wire debug link (semihosting)
    Segger RTT
- can use bi_decl() in C code to store information in compiled binaries that picotool can access (e.g., comments, pin numbers)

# More pictures

![Stacked PCBs](/media/levitas_pcb_render.jpg)
![Daughterboard](/media/levitas_pcb_render-daughterboard-bottom.jpg)
