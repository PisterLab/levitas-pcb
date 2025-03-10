// The following code reads data from the MPU6050 accelerometer
// over I2C then prints the result to a serial port over USB.
//
// Run this to confirm the sensor is working.

// Most of this code is adapted from the example code here:
// https://github.com/raspberrypi/pico-examples/blob/master/i2c/mpu6050_i2c/mpu6050_i2c.c
// Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
// SPDX-License-Identifier: BSD-3-Clause

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

// By default these devices are on bus address 0x68
static int addr = 0x68;

static void mpu6050_reset() {

    // for gyro and accel features, see page 10 of datasheet
    // gyro full-scale range: see FS_SEL value?
    // accel full-scale range: see AFS_SEL value?
    // etc.

    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(i2c1, addr, buf, 2, false);
    sleep_ms(100); // Allow device to reset and stabilize

    // Clear sleep mode (0x6B register, 0x00 value)
    buf[1] = 0x00;  // Clear sleep mode by writing 0x00 to the 0x6B register
    i2c_write_blocking(i2c1, addr, buf, 2, false);
    sleep_ms(10); // Allow stabilization after waking up
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c1, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c1, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c1, addr, &val, 1, true);
    i2c_read_blocking(i2c1, addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c1, addr, &val, 1, true);
    i2c_read_blocking(i2c1, addr, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

int main() {

    // enable all available stdio outputs
    // (currently only USB in CMakeLists.txt)
    // USB serial port defaults to 115200 baud
    stdio_init_all();

    // hopefully reduce 3.3V bus noise by setting
    // Pico dev board regular to PWM mode
#define PICO_POWER_PIN 23
    gpio_init(PICO_POWER_PIN);
    gpio_set_dir(PICO_POWER_PIN, GPIO_OUT);
    gpio_put(PICO_POWER_PIN, false);

    // Use hardware I2C on pins 2/3 at 400kHz
    // Note pins 2/3 are SDA/SCL specifically for the i2c1 (as opposed
    // to i2c0) hardware I2C block (see RP2350 or Pico datasheet); we'll
    // use that in each i2c read/write command
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);
    // show the I2C pins in picotool
    bi_decl(bi_2pins_with_func(2, 3, GPIO_FUNC_I2C));

    // initialize sensor
    mpu6050_reset();

    int16_t acceleration[3], gyro[3], temp;

    while (1) {
        // get data from sensor
        mpu6050_read_raw(acceleration, gyro, &temp);

        // These are the raw numbers from the chip, so will need tweaking to be really useful.
        // See the datasheet for more information
        printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
        printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        // Temperature is simple so use the datasheet calculation to get deg C.
        // Note this is chip temperature.
        printf("Temp. = %f\n", (temp / 340.0) + 36.53);

        sleep_ms(100);
    }
}
