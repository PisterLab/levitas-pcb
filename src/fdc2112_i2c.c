#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define fdc2112_addr 0x2b


// I2C address of FDC2112 chip
#define FDC2112_I2C_ADDR 0x2b
// I2C-accessible registers of FDC2112 chip
#define FDC2112_REG_DATA_CH0 0x00
#define FDC2112_REG_RCOUNT_CH0 0x08
#define FDC2112_REG_OFFSET_CH0 0x0c
#define FDC2112_REG_SETTLECOUNT_CH0 0x10
#define FDC2112_REG_CLOCK_DIVIDERS_CH0 0x14
#define FDC2112_REG_STATUS 0x18
#define FDC2112_REG_ERROR_CONFIG 0x19
#define FDC2112_REG_CONFIG 0x1a
#define FDC2112_REG_MUX_CONFIG 0x1b
#define FDC2112_REG_RESET_DEV 0x1c
#define FDC2112_REG_DRIVE_CURRENT_CH0 0x1e
#define FDC2112_REG_MANUFACTURER_ID 0x7e
#define FDC2112_REG_DEVICE_ID 0x7f

// write a 16-bit value to the given register
void fdc2112_write_register(i2c_inst_t* i2c_inst, uint8_t reg, uint16_t data){
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = data >> 8; // send MSB byte first
    buf[2] = data & 0xff; // send LSB byte last
    i2c_write_blocking(i2c_inst, FDC2112_I2C_ADDR, buf, 3, false);
    // datasheet requires 1.3us minimum bus free time between I2C STOP and START
    // TODO: don't use sleep functions in main loop?
    sleep_us(2);
}

// read a 16-bit value from given register
uint16_t fdc2112_read_register(i2c_inst_t* i2c_inst, uint8_t reg){
    uint8_t buf_reg[1];
    buf_reg[0] = reg;
    uint8_t buf_data[] = {0x00, 0x00};
    i2c_write_blocking(i2c_inst, FDC2112_I2C_ADDR, buf_reg, 1, true);
    i2c_read_blocking(i2c_inst, FDC2112_I2C_ADDR, buf_data, 2, false);
    // result is MSB then LSB byte
    return (((uint16_t)buf_data[0]) << 8) | ((uint16_t)buf_data[1]);
}

static void fdc2112_startup(i2c_inst_t* i2c_inst){
    // Chip is in sleep state by default and must be enabled.
    // To do so, set a bit in the CONFIG register, and set the
    // rest of the configuration while we're at it.

    // Set register 0x1a to 00010100 00000001 = 0x1401
    // (sense on channel 0, no sleep mode, full current, internal oscillator)

    fdc2112_write_register(i2c_inst, FDC2112_REG_RCOUNT_CH0, 0x8329);
    fdc2112_write_register(i2c_inst, FDC2112_REG_SETTLECOUNT_CH0, 0x000a);
    fdc2112_write_register(i2c_inst, FDC2112_REG_CLOCK_DIVIDERS_CH0, 0x2002);
    fdc2112_write_register(i2c_inst, FDC2112_REG_ERROR_CONFIG, 0x0000);
    // 0000 0010 0000 1101 = 0x020d
    fdc2112_write_register(i2c_inst, FDC2112_REG_MUX_CONFIG, 0x020d);
    fdc2112_write_register(i2c_inst, FDC2112_REG_DRIVE_CURRENT_CH0, 0x0000);
    // 0001 0100 0000 0001 = 0x1401
    fdc2112_write_register(i2c_inst, FDC2112_REG_CONFIG, 0x1401);
}

int main() {

    // enable all available stdio outputs
    // (currently only USB in CMakeLists.txt)
    // USB serial port defaults to 115200 baud
    stdio_init_all();

    // set up LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    i2c_init(i2c0, 400 * 1000); // i2c fast mode
    i2c_init(i2c1, 400 * 1000); // i2c fast mode
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_set_function(6, GPIO_FUNC_I2C);
    gpio_set_function(7, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);
    gpio_pull_up(6);
    gpio_pull_up(7);

    // call many times just in case
    sleep_ms(500);
    fdc2112_startup(i2c0);
    fdc2112_startup(i2c1);

    while(true){
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(50);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(50);
        printf("LeviTAS v202501-A\n");

        // read device ID (should be 0x3054)

        //uint16_t devid = fdc2112_read_register(FDC2112_REG_DEVICE_ID);
        uint16_t devid0 = fdc2112_read_register(i2c0, FDC2112_REG_DATA_CH0);
        printf("Data 0: 0x%x", (devid0 >> 8));
        printf(" %x\n", (devid0 & 0xff));

        uint16_t devid1 = fdc2112_read_register(i2c1, FDC2112_REG_DATA_CH0);
        printf("Data 1: 0x%x", (devid1 >> 8));
        printf(" %x\n", (devid1 & 0xff));

    }

    return 0;
}
