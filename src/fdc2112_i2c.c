#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define capsense_i2c i2c0
#define fdc2112_addr 0x2b

static void fdc2112_startup(){
    // Chip is in sleep state by default and must be enabled.
    // To do so, set a bit in the CONFIG register, and set the
    // rest of the configuration while we're at it.

    // Set register 0x1a to 00010100 10000001 = 0x1481
    // (sense on channel 0, no sleep mode, full current, internal oscillator, no INTB)

    uint8_t buf[] = {0x1a, 0x14, 0x81};
    i2c_write_blocking(capsense_i2c, fdc2112_addr, buf, 3, false);
    sleep_ms(1);
}

int main() {

    // enable all available stdio outputs
    // (currently only USB in CMakeLists.txt)
    // USB serial port defaults to 115200 baud
    stdio_init_all();

    // set up LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    //i2c_init(capsense_i2c, 100 * 1000); // i2c standard mode
    i2c_init(capsense_i2c, 100 * 1000); // i2c fast mode
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
    fdc2112_startup();
    sleep_ms(100);
    fdc2112_startup();
    sleep_ms(100);
    fdc2112_startup();
    sleep_ms(100);
    fdc2112_startup();

    while(true){
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(50);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(50);
        printf("LeviTAS v202501-A\n");

        // read device ID (should be 0x3054)
        uint8_t buffer[] = {0x00, 0x00};
        uint8_t reg[] = {0x7f};
        i2c_write_blocking(capsense_i2c, fdc2112_addr, reg, 1, true);
        i2c_read_blocking(capsense_i2c, fdc2112_addr, buffer, 2, false);
        printf("Data: 0x%x%x\n", buffer[0], buffer[1]);

    }

    return 0;
}
