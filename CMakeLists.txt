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
    #src/main.c
    #src/i2c.c
    #src/mpu6050_i2c.c
    src/spi.c
)

# add libraries (pico_stdlib for general use, etc)
target_link_libraries(levitas
    pico_stdlib
    hardware_i2c
    hardware_spi
)

# UART on pins 0/1 is enabled by default, disable it
pico_enable_stdio_uart(levitas 0)

# enable USB virtual serial port
# (this is equivalent to including pico_stdio_usb)
# (includes functionality to reset over USB)
pico_enable_stdio_usb(levitas 1)

# create map/bin/hex/uf2 files in addition to ELF
pico_add_extra_outputs(levitas)
