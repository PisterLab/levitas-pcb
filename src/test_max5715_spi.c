// The following code sets the PCB high voltage generator
// to output given voltages. This works by sending SPI
// commands to the MAX5715 DAC chip, whose output is amplified
// by the HV264 amplifier.
//
// NOTE: DAC voltage outputs 1-4 do not directly correspond with
// PCB voltage outputs 1-4 or capacitance sensors 1-4!

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


// Sending data to DAC requires sending
// this HVAWG_init_command followed by 3 bytes = 24 bits via SPI.
static inline void HVAWG_init_command(){
    // To start command, raise CS for >= 20ns, then lower
    gpio_put(HVAWG_CS_PIN, 1);
    asm volatile("nop \n nop \n nop \n nop"); // assume <= 150MHz clock
    gpio_put(HVAWG_CS_PIN, 0);
}

// Configure DAC.
void HVAWG_reset(){

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

// Write values to high voltage DAC. Values 1-4 are mapped to
// HV1, HV2, HV3, HV4 on the PCB, respectively.
//
// Values are integers from 0 to 4096-1. This function is separate
// from HVAWG_write_voltage() for convenience because it can
// (approximately) guarantee that different inputs will result in
// different outputs, whereas two sufficiently close voltages might
// have the same integer code and thus same DAC output.
void HVAWG_write_count(uint val1, uint val2, uint val3, uint val4){

        // confirm there are only 12 bits to avoid
        // accidentally overwriting parts of SPI command
        if(val1 > 4095){ val1 = 4095; }
        if(val2 > 4095){ val2 = 4095; }
        if(val3 > 4095){ val3 = 4095; }
        if(val4 > 4095){ val4 = 4095; }

        // CODEn = 00000001 11111111 11110000 = 0x01fff0
        // CODEn = 00000010 11111111 11110000 = 0x02fff0
        // CODEn = 00000100 11111111 11110000 = 0x04fff0
        // CODEn_LOAD_ALL = 00101000 11111111 11110000 = 0x28fff0

        // Note that DAC channel 1,2,3,4 are labelled on the PCB
        // as high voltage outputs 4,3,2,1. Write the following data
        // such that HVAWG_write_count(10,0,0,0) will write a high
        // voltage to PCB HV1, for example.

        // use CODEn commands (see manual page 18) to
        // load values into first three DAC registers but without
        // updating DAC output yet
        HVAWG_init_command();
        uint8_t buf1[] = {0x03, val1 >> 4, (val1 & 0xf)<<4};
        spi_write_blocking(spi1, buf1, 3);
        HVAWG_init_command();
        uint8_t buf2[] = {0x02, val2 >> 4, (val2 & 0xf)<<4};
        spi_write_blocking(spi1, buf2, 3);
        HVAWG_init_command();
        uint8_t buf3[] = {0x01, val3 >> 4, (val3 & 0xf)<<4};
        spi_write_blocking(spi1, buf3, 3);

        // use CODEn_LOAD_ALL command (see manual page 18) to
        // load in value for the fourth and last channel, then
        // update the DAC output on all four channels at once
        HVAWG_init_command();
        uint8_t buf4[] = {0x20, val4 >> 4, (val4 & 0xf)<<4};
        spi_write_blocking(spi1, buf4, 3);
}

void HVAWG_write_voltage(float V1, float V2, float V3, float V4){
    // The DAC takes a 12-bit input command,
    // where the minimum (0b000000000000) outputs 0V
    // and the maximum (0b111111111111) outputs 4.096V
    // (as determined by the internal reference in HVAWG_reset().)
    // This is amplified approximately 66.7x by the HV264 amplifier.
    // The final output voltage is capped by the power supply,
    // which is usually about 240V but can dip to 220-230V when
    // driving many capacitive sensors at high frequency (which
    // requires greater current). Note this means it is possible
    // to tell the DAC to output a voltage higher than the amplifier
    // and power supply will do (i.e., 4.096*66.7=273V), in which
    // case the output will be capped to the power supply voltage.

    // convert voltage to DAC value (integer from 0 to 2^12-1):
    // divide by 66.7x HV264 gain, 4.096V DAC reference voltage,
    // multiply by 2^12 bits = 4096, and convert to integer.
    uint value1 = (uint)(V1/66.7/4.096*4096);
    uint value2 = (uint)(V2/66.7/4.096*4096);
    uint value3 = (uint)(V3/66.7/4.096*4096);
    uint value4 = (uint)(V4/66.7/4.096*4096);

    // write value to HVAWG
    HVAWG_write_count(value1, value2, value3, value4);
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
        HVAWG_write_voltage(0,0,0,0);
        sleep_ms(10);

        gpio_put(PICO_DEFAULT_LED_PIN, false);
        HVAWG_write_voltage(10,10,10,10);
        sleep_ms(10);
    }
}
