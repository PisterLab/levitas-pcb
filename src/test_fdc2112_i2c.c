// The following code reads data from two fdc2112 capacitive
// sensors, one after the other, using the hardware I2C built into
// the RP2350 chip. The data is then printed out via USB.
//
// NOTE: it is better to use test_fdc2112_pio_i2c.c, which uses the
// RP2350 PIO modules for I2C and able to read all four capacitive
// sensors, and do so more simultaneously (plus/minus only a microsecond
// or so).

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
    // set assorted chip configuration values.
    // see manual or test_fdc2112_pio_i2c.c for more detail.

    // F_ref = 43MHz -> 2866 cycles per sample at 15kHz -> 10 bits precision? -> 1024 clock cycles for rcount -> rcount = 0x0040
    //fdc2112_write_register(i2c_inst, FDC2112_REG_RCOUNT_CH0, 0x8329);
    // rcount > 8
    fdc2112_write_register(i2c_inst, FDC2112_REG_RCOUNT_CH0, 0x0040);

    //fdc2112_write_register(i2c_inst, FDC2112_REG_SETTLECOUNT_CH0, 0x000a);
    fdc2112_write_register(i2c_inst, FDC2112_REG_SETTLECOUNT_CH0, 0x000a);
    // settlecount > 3

    fdc2112_write_register(i2c_inst, FDC2112_REG_CLOCK_DIVIDERS_CH0, 0x2002);
    fdc2112_write_register(i2c_inst, FDC2112_REG_ERROR_CONFIG, 0x0000);
    // 0000 0010 0000 1101 = 0x020d
    fdc2112_write_register(i2c_inst, FDC2112_REG_MUX_CONFIG, 0x020d);
    fdc2112_write_register(i2c_inst, FDC2112_REG_DRIVE_CURRENT_CH0, 0x0000);

    // this I2C configuration makes the chip start measurements
    // 0001 0100 0000 0001 = 0x1401
    fdc2112_write_register(i2c_inst, FDC2112_REG_CONFIG, 0x1401);
}

int main() {

    // enable all available stdio outputs
    // (currently only USB in CMakeLists.txt)
    // USB serial port defaults to 115200 baud
    stdio_init_all();

    // set up LED
    //gpio_init(PICO_DEFAULT_LED_PIN);
    //gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    i2c_init(i2c0, 400 * 1000); // i2c fast mode
    i2c_init(i2c1, 400 * 1000); // i2c fast mode

    // choose which two capacitive sensor chips to connect to
    // (can only do two at once, and one must be on the i2c0
    // hardware pins while the other is on the i2c1 pins)
    gpio_set_function(4, GPIO_FUNC_I2C); // i2c0 SDA, Pico 6, MISO-d
    gpio_set_function(5, GPIO_FUNC_I2C); // i2c0 SCL, Pico 7, CS-d
    gpio_set_function(6, GPIO_FUNC_I2C); // i2c1 SDA, Pico 9, SCLK-d
    gpio_set_function(7, GPIO_FUNC_I2C); // i2c1 SCL, Pico 10, MOSI-d
    //gpio_set_function(8, GPIO_FUNC_I2C); // i2c0 SDA, Pico 11, SDA-d
    //gpio_set_function(9, GPIO_FUNC_I2C); // i2c0 SCL, Pico 12, SCL-d
    //gpio_set_function(10, GPIO_FUNC_I2C); // i2c1 SDA, Pico 14, SDA4-d
    //gpio_set_function(11, GPIO_FUNC_I2C); // i2c1 SCL, Pico 15, SCL4-d
    gpio_pull_up(4);
    gpio_pull_up(5);
    gpio_pull_up(6);
    gpio_pull_up(7);
    gpio_pull_up(8);
    gpio_pull_up(9);
    gpio_pull_up(10);
    gpio_pull_up(11);

    sleep_ms(500);
    fdc2112_startup(i2c0);
    fdc2112_startup(i2c1);

    gpio_put(PICO_DEFAULT_LED_PIN, true);
    while(true){
        //gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(10);

        // read data from chips:

        uint16_t devid0 = fdc2112_read_register(i2c0, FDC2112_REG_DATA_CH0);
        printf("Data 0: 0x%02x", (devid0 >> 8));
        printf("%02x\n", (devid0 & 0xff));

        uint16_t devid1 = fdc2112_read_register(i2c1, FDC2112_REG_DATA_CH0);
        printf("Data 1: 0x%02x", (devid1 >> 8));
        printf("%02x\n", (devid1 & 0xff));
    }

    return 0;
}
