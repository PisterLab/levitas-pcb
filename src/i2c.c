// Sweep through all 7-bit I2C addresses, to see if any slaves are present on
// the I2C bus. Print out a table that looks like this:
//
// I2C Bus Scan
//    0 1 2 3 4 5 6 7 8 9 A B C D E F
// 00 . . . . . . . . . . . . . . . .
// 10 . . @ . . . . . . . . . . . . .
// 20 . . . . . . . . . . . . . . . .
// 30 . . . . @ . . . . . . . . . . .
// 40 . . . . . . . . . . . . . . . .
// 50 . . . . . . . . . . . . . . . .
// 60 . . . . . . . . . . . . . . . .
// 70 . . . . . . . . . . . . . . . .
// E.g. if addresses 0x12 and 0x34 were acknowledged.

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

    // accelerometer I2C pins
//#define ACCEL_I2C i2c1
//#define ACCEL_I2C I2C_INSTANCE(1)
#define LEVITAS_ACCEL_SDA_PIN 2
#define LEVITAS_ACCEL_SCL_PIN 3

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void scan_bus(){
    printf("\nI2C Bus Scan\n");
    printf("   0 1 2 3 4 5 6 7 8 9 A B C D E F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }
        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr)) {
            ret = PICO_ERROR_GENERIC;
        } else {
            //ret = 0;
            ret = i2c_read_blocking(I2C_INSTANCE(1), addr, &rxdata, 1, false);
        }

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : " ");
    }
    printf("Done.\n");
}

int main() {

    // enable all available stdio outputs
    // (currently only USB in CMakeLists.txt)
    // USB serial port defaults to 115200 baud
    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    //i2c_init(i2c_default, 100 * 1000); // i2c standard mode
    //i2c_init(i2c_default, 400 * 1000); // i2c fast mode
    i2c_init(I2C_INSTANCE(1), 100 * 1000); // i2c standard mode
    // PICO_DEFAULT_I2C_INSTANCE
    //gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    //gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    //gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    //gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    gpio_set_function(LEVITAS_ACCEL_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(LEVITAS_ACCEL_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(LEVITAS_ACCEL_SDA_PIN);
    gpio_pull_up(LEVITAS_ACCEL_SCL_PIN);

    // Make the I2C pin information available to "picotool info --all"
    //bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
    // i2c default: 0
    // GP4 (pcb pin 6): i2c0 sda default
    // GP5 (pcb pin 7): i2c0 scl default

    // pico pins 4/5 = GPIO 2/3, which is I2C1 not I2C0
    while(true){
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(500);
        printf("LeviTAS v202501-A\n");
        //scan_bus(); // should report BH1750 at address 0x23
    }

    // connect to ADXL345 accelerometer to test i2c bus
    // ADXL345 should be at address 0x53
    // connect to BH1750 light sensor to test i2c bus
    // BH1750 should be at address 0x23


    static uint8_t accel_addr = 0x53;
    uint8_t accel_reg = 0x34; // start of Y-axis registers
    uint8_t accel_data[] = {0, 0}; // to store read data


    sleep_ms(100);
    // set power mode and data rage in 0x2C register
    int accel_setting = 0x08;
    i2c_write_blocking(i2c_default, 0x2C, &accel_setting, 1, false);
    // set data format in 0x31 register
    accel_setting=0x08;
    i2c_write_blocking(i2c_default, 0x31, &accel_setting, 1, false);
    // enter measurement mode by setting measure bit D3 in POWER_CTL register 0x2D
    uint8_t start_reg = 0x2D;
    uint8_t start_command = 0x08; // 00001000 = 0x08
    i2c_write_blocking(i2c_default, start_reg, &start_command, 1, false);

    // BH1750 light sensor
    // first, put sensor into continuous-read low-resolution (fast) mode
    //static uint8_t light_addr = 0x23
    //uint8_t light_mode = 0x13; // 00010011 = 0x13
    //i2c_write_blocking(i2c_default, light_addr, &light_mode, 1, false);
    //uint8_t lux_data[] = {0, 0}; // to store read data

    /*
    while (true) {

        // read y-axis accelerometer data
        // register 0x34, 0x35
        sleep_ms(100);
        i2c_write_blocking(i2c_default, accel_addr, &accel_reg, 1, true); // first send register address
        i2c_read_blocking(i2c_default, accel_addr, accel_data, 2, false); // then read from it
        printf("Read accel data: %x %x \n", accel_data[0], accel_data[1]);

        // get reading from BH1750 light sensor
        //sleep_ms(24); // low resolution sensing time
        //i2c_read_blocking(i2c_default, light_addr, lux_data, 2, false);
        //printf("Read light data: %x %x \n", lux_data[0], lux_data[1]);
    }
    */

    return 0;
}
