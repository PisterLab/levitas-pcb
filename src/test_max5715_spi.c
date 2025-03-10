#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"

// SPI1 CS is GPIO 13 = Pico pin 17
// SPI1 SCLK is GPIO 14 = Pico pin 19
// SPI1 MOSI is GPIO 15 = Pico pin 20
// SPI1 MISO is not used
#define HVAWG_CS_PIN 13
#define HVAWG_SCLK_PIN 14
#define HVAWG_MOSI_PIN 15

static inline void HVAWG_init_command(){
    // To start command, raise CS for >= 20ns, then lower
    gpio_put(HVAWG_CS_PIN, 1);
    asm volatile("nop \n nop \n nop \n nop"); // assume <= 150MHz clock
    gpio_put(HVAWG_CS_PIN, 0);
}

void HVAWG_reset(){
    // all commands are init followed by 3 bytes = 24 bits

    // reset all settings to default
    // SW_RESET = 0101 0001 x x = 0x510000
    uint8_t buf_reset[] = {0x51, 0x00, 0x00};
    HVAWG_init_command();
    spi_write_blocking(spi1, buf_reset, 3);

    // set internal reference to always on, 4.096V
    // REF = 0111 0111 x x = 0x770000
    uint8_t buf_ref[] = {0x77, 0x00, 0x00};
    HVAWG_init_command();
    spi_write_blocking(spi1, buf_ref, 3);

    // let all DACs be controlled by LOAD commands
    // (already the default; redundant)
    // CONFIG = 01101000 00001111 x = 0x680f00
    //uint8_t buf_config[] = {0x68, 0x0f, 0x00};
    //HVAWG_init_command();
    //spi_write_blocking(spi1, buf_config, 3);

    // power on DACs
    // (already the default; redundant)
    // POWER = 01000000 00001111 x = 0x400f00
    //uint8_t buf_power[] = {0x40, 0x0f, 0x00};
    //HVAWG_init_command();
    //spi_write_blocking(spi1, buf_power, 3);
}

void HVAWG_write(int state){
    // all commands are init followed by 3 bytes = 24 bits

    // CODEn = 0000(dac-select) (data) (data)xxxx
    // CODEn_LOAD_ALL = 0010(dac-select) (data) (data)xxxx

    // DAC selection (see table 3):
    // 0x00, 0x01, 0x02, 0x03 (or 0x04, 0x08 for all DACs)

    if(state==0){
        // CODEn = 00000001 11111111 11110000 = 0x01fff0
        // CODEn = 00000010 11111111 11110000 = 0x02fff0
        // CODEn = 00000100 11111111 11110000 = 0x04fff0
        // CODEn_LOAD_ALL = 00101000 11111111 11110000 = 0x28fff0

        HVAWG_init_command();
        uint8_t buf1[] = {0x00, 0x00, 0x00};
        spi_write_blocking(spi1, buf1, 3);

        HVAWG_init_command();
        //uint8_t buf2[] = {0x01, 0xff, 0xf0};
        uint8_t buf2[] = {0x01, 0xff, 0xf0};
        spi_write_blocking(spi1, buf2, 3);

        HVAWG_init_command();
        uint8_t buf3[] = {0x02, 0x00, 0x00};
        spi_write_blocking(spi1, buf3, 3);

        HVAWG_init_command();
        //uint8_t buf4[] = {0x23, 0xff, 0xf0};
        uint8_t buf4[] = {0x23, 0xff, 0xf0};
        spi_write_blocking(spi1, buf4, 3);

    }else if(state==1){

        HVAWG_init_command();
        uint8_t buf1[] = {0x00, 0xff, 0xf0};
        spi_write_blocking(spi1, buf1, 3);

        HVAWG_init_command();
        uint8_t buf2[] = {0x01, 0x00, 0x00};
        spi_write_blocking(spi1, buf2, 3);

        HVAWG_init_command();
        uint8_t buf3[] = {0x02, 0xff, 0xf0};
        spi_write_blocking(spi1, buf3, 3);

        HVAWG_init_command();
        uint8_t buf4[] = {0x23, 0x00, 0x00};
        spi_write_blocking(spi1, buf4, 3);

    }

}

int main() {

    // enable all available stdio outputs
    // (currently only USB in CMakeLists.txt)
    // USB serial port defaults to 115200 baud
    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    spi_init(spi1, 5000*1000);
    gpio_set_function(HVAWG_SCLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(HVAWG_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_init(HVAWG_CS_PIN);
    gpio_set_dir(HVAWG_CS_PIN, GPIO_OUT);
    gpio_put(HVAWG_CS_PIN, 0); // active low

    // wait at least 200us for DAC to initialize
    sleep_ms(250);
    HVAWG_reset();

    printf("Hello, world!\n");

    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        HVAWG_write(0);
        sleep_us(25);

        gpio_put(PICO_DEFAULT_LED_PIN, false);
        HVAWG_write(1);
        sleep_us(25);
    }
}
