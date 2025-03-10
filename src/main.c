// This file is the entry point (and main source code file) for the
// current LeviTAS PCB project.
//
// It contains code to read from four FDC2112 capacitance sensors,
// write voltages to a 4-channel MAX5715 DAC (connected to a HV264
// amplifier), and write data out via a serial port (over USB).

// include necessary standard libraries
#include "pico/stdlib.h"        // standard functions
#include <stdio.h>              // printf
#include "pico/multicore.h"     // send USB printf on second core
#include "pico/util/queue.h"    // queue for multicore communication
#include "pico/time.h"          // assorted time functions
#include "hardware/spi.h"       // SPI for MAX5715
// also include our custom I2C PIO library
#include "pio_i2c.h"

// ===========================================================================
// ===========================================================================
// ========== FDC2112 capacitance sensors ====================================
// ===========================================================================
// ===========================================================================
// (This section contains data and functions to use the capacitance sensors.)
// (adapted from test_fdc2112_pio_i2c.c)

// I2C address of all FDC2112 chips (each on its own I2C bus)
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

typedef struct {
    // store four capacitance values
    int16_t data0;
    int16_t data1;
    int16_t data2;
    int16_t data3;
} levitation_state_t;
queue_t data_queue;

// write a 16-bit value to the given register
void fdc2112_write_register(PIO pio, uint sm, uint8_t reg, uint16_t data){
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = data >> 8; // send MSB byte first
    buf[2] = data & 0xff; // send LSB byte last
    int err = pio_i2c_write_blocking(pio, sm, FDC2112_I2C_ADDR, buf, 3, false);
}
void fdc2112_write_register_4(PIO pio, uint sm0, uint sm1, uint sm2, uint sm3, uint8_t reg, uint16_t data){
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = data >> 8; // send MSB byte first
    buf[2] = data & 0xff; // send LSB byte last
    pio_i2c_write_blocking_4(pio, sm0, sm1, sm2, sm3, FDC2112_I2C_ADDR, buf, 3, false);
}

// read a 16-bit value from given register
uint16_t fdc2112_read_register(PIO pio, uint sm, uint8_t reg){
    uint8_t buf_reg[1];
    buf_reg[0] = reg;
    uint8_t buf_data[] = {0x00, 0x00};
    pio_i2c_write_blocking(pio, sm, FDC2112_I2C_ADDR, buf_reg, 1, true); // shouldn't stop
    pio_i2c_read_blocking(pio, sm, FDC2112_I2C_ADDR, buf_data, 2);
    // result is MSB then LSB byte
    return (((uint16_t)buf_data[0]) << 8) | ((uint16_t)buf_data[1]);
}
void fdc2112_read_register_4(PIO pio, uint sm0, uint sm1, uint sm2, uint sm3, uint8_t reg, uint16_t* result){
    uint8_t buf_reg[1];
    buf_reg[0] = reg;
    uint8_t buf_data0[] = {0x00, 0x00};
    uint8_t buf_data1[] = {0x00, 0x00};
    uint8_t buf_data2[] = {0x00, 0x00};
    uint8_t buf_data3[] = {0x00, 0x00};
    pio_i2c_write_blocking_4(pio, sm0, sm1, sm2, sm3, FDC2112_I2C_ADDR, buf_reg, 1, true); // shouldn't stop
    pio_i2c_read_blocking_4(pio, sm0, sm1, sm2, sm3, FDC2112_I2C_ADDR, buf_data0, buf_data1, buf_data2, buf_data3, 2);
    // result is MSB then LSB byte
    result[0] =  (((uint16_t)buf_data0[0]) << 8) | ((uint16_t)buf_data0[1]);
    result[1] =  (((uint16_t)buf_data1[0]) << 8) | ((uint16_t)buf_data1[1]);
    result[2] =  (((uint16_t)buf_data2[0]) << 8) | ((uint16_t)buf_data2[1]);
    result[3] =  (((uint16_t)buf_data3[0]) << 8) | ((uint16_t)buf_data3[1]);
}

// read a 16-bit value (assuming register was previously indicated, e.g., by a full read)
uint16_t fdc2112_read_register_nowrite(PIO pio, uint sm){
    uint8_t buf_data[] = {0x00, 0x00};
    //pio_i2c_write_blocking(pio, sm, FDC2112_I2C_ADDR, buf_reg, 1, true); // shouldn't stop
    pio_i2c_read_blocking(pio, sm, FDC2112_I2C_ADDR, buf_data, 2);
    // result is MSB then LSB byte
    return (((uint16_t)buf_data[0]) << 8) | ((uint16_t)buf_data[1]);
}
void fdc2112_read_register_nowrite_4(PIO pio, uint sm0, uint sm1, uint sm2, uint sm3, uint16_t* result){
    uint8_t buf_data0[] = {0x00, 0x00};
    uint8_t buf_data1[] = {0x00, 0x00};
    uint8_t buf_data2[] = {0x00, 0x00};
    uint8_t buf_data3[] = {0x00, 0x00};
    pio_i2c_read_blocking_4(pio, sm0, sm1, sm2, sm3, FDC2112_I2C_ADDR, buf_data0, buf_data1, buf_data2, buf_data3, 2);
    // result is MSB then LSB byte
    result[0] =  (((uint16_t)buf_data0[0]) << 8) | ((uint16_t)buf_data0[1]);
    result[1] =  (((uint16_t)buf_data1[0]) << 8) | ((uint16_t)buf_data1[1]);
    result[2] =  (((uint16_t)buf_data2[0]) << 8) | ((uint16_t)buf_data2[1]);
    result[3] =  (((uint16_t)buf_data3[0]) << 8) | ((uint16_t)buf_data3[1]);
}

static void fdc2112_startup(PIO pio, uint sm){
    // Chip is in sleep state by default and must be enabled.
    // To do so, set a bit in the CONFIG register. First, set the
    // rest of the configuration while we're at it.

    // ===== Measurement time behavior =====
    //
    // First we set the RCOUNT and SETTLECOUNT registers.
    // These control how long the chip takes to read a given signal
    // (see manual pages 15, 16).
    //
    // If the chip is used to read capacitance from multiple channels,
    // SETTLECOUNT determines the amount of time a measurement is
    // allowed to stabilize after switching channels. This does not
    // apply to us, so we set SETTLECOUNT and otherwise ignore it.
    // (TODO CONFIRM THIS WORKS)
    //
    // The chip takes a configurable amount of time to take a measurement.
    // Longer measurement times give more accurate values. The time is
    // determined by RCOUNT (specifically, (RCOUNT*16+4)/f_{REF0} where
    // f_{REF0} is the reference oscillator frequency, about 40MHz).
    // Measuring n bits of precision requires 2^n = (RCOUNT*16).
    // In our case, we want to take measurements within maybe 10us. We
    // choose RCOUNT=33=0x0021 giving 9.04 bits of precision in 13.us.
    // (Note the minimum is RCOUNT>8, which we satisfy; see page 13)
    //
    // (but register map on page 25 says min is 0x0100 or 0x0080?)
    //fdc2112_write_register(pio, sm, FDC2112_REG_RCOUNT_CH0, 0x0040);
    fdc2112_write_register(pio, sm, FDC2112_REG_RCOUNT_CH0, 0x0021);
    // determines the sensor activation time
    // must have settlecount > 3
    fdc2112_write_register(pio, sm, FDC2112_REG_SETTLECOUNT_CH0, 0x000a);

    // ===== Small things =====
    //
    // The chip has some error reporting functions; leave them off
    fdc2112_write_register(pio, sm, FDC2112_REG_ERROR_CONFIG, 0x0000);
    //
    // Prepare to only read capacitance from a single channel on the chip
    // (this setting can choose to alternate readings between channels)
    // 0000 0010 0000 1101 = 0x020d
    fdc2112_write_register(pio, sm, FDC2112_REG_MUX_CONFIG, 0x020d);

    // ===== Gain and offset settings =====
    //
    // The following clock divider setting changes some measurement
    // parameters:
    // f_{REF0}=f_{CLK}/CH0_FREQ_DIVIDER
    // f_{IN0}=f_{SENSOR0}/CH0_FIN_SEL
    fdc2112_write_register(pio, sm, FDC2112_REG_CLOCK_DIVIDERS_CH0, 0x2002);
    //
    // ...and the resulting digital data is 16 bits, of which only
    // 12 bits can be read over I2C. However, the data can be
    // bit-shifted and/or given an offset before I2C reading in order
    // to see the LSB (e.g., if there is only a small capacitance
    // change in a relatively large capacitance).
    //
    // set gain to a 3-bit shift (2^3=8x multiplier):
    // (see manual page 35)
    // 0 0000 00 0 | 0000 0000
    //fdc2112_write_register(pio, sm, FDC2112_REG_RESET_DEV, 0x0400);
    fdc2112_write_register(pio, sm, FDC2112_REG_RESET_DEV, 0x0400);
    // offset:
    // (see manual page 27)
    fdc2112_write_register(pio, sm, FDC2112_REG_OFFSET_CH0, 0x0000);

    // ===== Drive current =====
    //
    // The current the sensor uses to read capacitance can be adjusted.
    // This results in slightly different capacitance readings.
    // Supposedly there's an optimum value that can be found by tweaking
    // until the sensor output drive signal is about 1.8V, but I wasn't
    // able to measureme of the signal (probably too high frequency?).
    //
    // Using the default values (0x0000) seems to work fine.
    //
    // Determines the drive current:
    // (see chart on manual page 36)
    //
    // code  resulting capacitance readings
    // ====================================
    // 00000 0x01b8 0x01d3 0x01d3 0x01c9
    // 00001 0x01ba 0x01d5 0x01d5 0x01cb
    // 00010 0x01bb 0x01d7 0x01d7 0x01cc
    // 00011 0x01bd 0x01d9 0x01d9 0x01cd
    // ...
    // 01001 0x01c3 0x01df 0x01df 0x01d4
    //
    // Result: higher current -> slightly higher reading?
    // (TBD: measure voltage)
    fdc2112_write_register(pio, sm, FDC2112_REG_DRIVE_CURRENT_CH0, 0x0000);

    // ===== Start measurement =====
    //
    // Upon startup, the chip is in a sleep state. Use the following command
    // to start the chip so we can read capacitance. Note that afterward,
    // additional configuration changes might not work until the chip is
    // powered off and on again (which is why we do this last).
    //
    // At the same time, this register also choose which oscillator to use.
    // We will use the chip's internal oscillator (we don't have an external
    // oscillator connected).
    //
    // Set register 0x1a to 00010100 00000001 = 0x1401
    // (sense on channel 0, no sleep mode, full current, internal oscillator)
    // - this also tells the chip to use its internal reference oscillator
    //   - f_{CLK} = f_{INTCLK}
    //   - f_{INTCLK} = internal clock = 43MHz (approximately; ranges 35-55MHz with manufacturing)
    fdc2112_write_register(pio, sm, FDC2112_REG_CONFIG, 0x1401);
}

// ===========================================================================
// ===========================================================================
// ========== MAX5715 DAC ====================================================
// ===========================================================================
// ===========================================================================
// (This section contains data and functions to use the high voltage output.)
// (adapted from test_max5715_spi.c)

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

// ===========================================================================
// ===========================================================================
// ========== Main code ======================================================
// ===========================================================================
// ===========================================================================
// (This section contains the main function.)

// Print to USB with second core
// This means printing won't take time away from the main sensor read loop
// (though it still adds a little bit of jitter, visible via oscilloscope)
void core2_main(){
    while(true){
        levitation_state_t datapoint;
        queue_remove_blocking(&data_queue, &datapoint);
        printf("Data: 0x%04x 0x%04x 0x%04x 0x%04x\n", datapoint.data0, datapoint.data1, datapoint.data2, datapoint.data3);
    }
}

int main() {

    // enable all available stdio outputs
    // (currently only USB in CMakeLists.txt)
    // USB serial port defaults to 115200 baud
    stdio_init_all();

    // launch second core
    // queue holds at most 2 data points
    // (will use this to send data over USB without affecting (much) timing of main sensor readings)
    queue_init(&data_queue, sizeof(levitation_state_t), 2);
    multicore_launch_core1(core2_main);

    // set up LED (in case we want to do anything with it)
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // set up FDC2112 sensors
    PIO pio = pio0; // choose a PIO
    uint sm0 = 0; // choose PIO state machine
    uint sm1 = 1; // choose PIO state machine
    uint sm2 = 2; // choose PIO state machine
    uint sm3 = 3; // choose PIO state machine
    uint offset = pio_add_program(pio, &pio_i2c_program); // load program into PIO memory, return instruction memory offset it is loaded at
    pio_i2c_program_init(pio, sm0, offset, 4, 5); // start PIO state machine at given instruction offset with given SDA and SCL pin
    pio_i2c_program_init(pio, sm1, offset, 6, 7); // start PIO state machine at given instruction offset with given SDA and SCL pin
    pio_i2c_program_init(pio, sm2, offset, 8, 9); // start PIO state machine at given instruction offset with given SDA and SCL pin
    pio_i2c_program_init(pio, sm3, offset, 10, 11); // start PIO state machine at given instruction offset with given SDA and SCL pin
    // wait just in case fdc2112 sensors need to initialize
    sleep_ms(500);
    fdc2112_startup(pio, sm0);
    fdc2112_startup(pio, sm1);
    fdc2112_startup(pio, sm2);
    fdc2112_startup(pio, sm3);
    uint16_t result[] = {0x00, 0x00, 0x00, 0x00};
    // make a first reading explicitly identifying the chip registers
    // we want to read from in the future. Afterward, we can use the
    // faster read_register_nowrite function which will just read the
    // same data we get here.
    fdc2112_read_register_4(pio, sm0, sm1, sm2, sm3, FDC2112_REG_DATA_CH0, result);

    // set up MAX5715 DAC
    spi_init(spi1, 5000*1000);
    gpio_set_function(HVAWG_SCLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(HVAWG_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_init(HVAWG_CS_PIN);
    gpio_set_dir(HVAWG_CS_PIN, GPIO_OUT);
    gpio_put(HVAWG_CS_PIN, 0); // active low
    // wait at least 200us for DAC to initialize
    sleep_ms(250);
    HVAWG_reset();
    HVAWG_write_voltage(0,0,0,0); // start by applying zero volts everywhere

    // start main loop
    const uint64_t LOOP_TIME_US = 150; // try to loop this long on average
    absolute_time_t loop_start_time = get_absolute_time();
    while(true){
        //gpio_put(PICO_DEFAULT_LED_PIN, true);
        //gpio_put(PICO_DEFAULT_LED_PIN, false);

        // read quickly from the chip registers
        fdc2112_read_register_nowrite_4(pio, sm0, sm1, sm2, sm3, result);

        // send data to second core for printing
        levitation_state_t entry = {result[0], result[1], result[2], result[3]};
        // NOTE: THIS CAUSES SIGNIFICANT (maybe ~20us) TIMING JITTER
        queue_try_add(&data_queue, &entry); // nonblocking

        // Loop length will vary slightly due to multicore communication,
        // so keep a timer to try to control average loop time, making each
        // loop slightly longer or shorter as necessary.
        loop_start_time = delayed_by_us(loop_start_time, LOOP_TIME_US);
        absolute_time_t now_time = get_absolute_time();
        if(loop_start_time < now_time){
            // If a loop took far too long (1.5-2x longer than expected or
            // more, we will have desynchronized from our loop time count
            // and should restart as quickly as possible.
            loop_start_time = now_time;
        }
        sleep_until(loop_start_time); // takes absolute_time_t
    }

    return 0;
}

