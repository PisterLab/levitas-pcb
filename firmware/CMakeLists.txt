# NOTE:
# This file must be manually changed to include new source files
# (via add_executable) and imports (via target_link_libraries)

cmake_minimum_required(VERSION 3.13)

# explicitly use the RP2040 on pico dev board
# (even though this is the default)
set(PICO_BOARD pico2)
set(PICO_PLATFORM rp2350)

# initialize pico-sdk from submodule
# this must happen before project()
include(pico-sdk/pico_sdk_init.cmake)

# define project name
project(levitas)

# initialize Raspberry Pi Pico SDK
pico_sdk_init()

# define local project files
add_executable(levitas
    # main program
    src/main.c

    # PIO i2c library
    src/pio_i2c.c
    src/pio_i2c.h

    # (to run tests, enable any of the following and disable main.c)
    # test fdc2112 cap sensors with hardware i2c (worse than PIO)
    # src/test_fdc2112_i2c.c
    # test fdc2112 cap sensors with PIO i2c (better than hardware i2c)
    # src/test_fdc2112_pio_i2c.c
    # test mpu6050 accelerometer with hardware i2c
    # src/test_mpu6050_i2c.c
    # test max5715 DAC with hardware spi
    # src/test_max5715_spi.c
)

# take PIO file and generate a header file (to be included in C files)
pico_generate_pio_header(levitas ${CMAKE_CURRENT_LIST_DIR}/src/pio_i2c.pio)

# add libraries (pico_stdlib for general use, etc)
target_link_libraries(levitas
    pico_stdlib
    hardware_i2c
    hardware_spi
    hardware_pio
    pico_multicore
    pico_util
    pico_time
)

# UART on pins 0/1 is enabled by default, disable it
pico_enable_stdio_uart(levitas 0)

# enable USB virtual serial port
# (i.e., appear as a serial port when plugged in over USB, use w/ printf)
# (this is equivalent to including pico_stdio_usb)
# (includes functionality to reset over USB)
pico_enable_stdio_usb(levitas 1)

# create map/bin/hex/uf2 files in addition to ELF
pico_add_extra_outputs(levitas)
