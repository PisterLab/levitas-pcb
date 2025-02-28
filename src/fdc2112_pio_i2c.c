#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h" // send USB printf on second core
#include "pico/util/queue.h" // queue for multicore communication
#include "pio_i2c.h"

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

typedef struct
{
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
    // To do so, set a bit in the CONFIG register, and set the
    // rest of the configuration while we're at it.

    // Set register 0x1a to 00010100 00000001 = 0x1401
    // (sense on channel 0, no sleep mode, full current, internal oscillator)


    // F_ref = 43MHz -> 2866 cycles per sample at 15kHz -> 10 bits precision? -> 1024 clock cycles for rcount -> rcount = 0x0040
    //fdc2112_write_register(i2c_inst, FDC2112_REG_RCOUNT_CH0, 0x8329);
    // rcount > 8
    fdc2112_write_register(pio, sm, FDC2112_REG_RCOUNT_CH0, 0x0040);

    //fdc2112_write_register(i2c_inst, FDC2112_REG_SETTLECOUNT_CH0, 0x000a);
    fdc2112_write_register(pio, sm, FDC2112_REG_SETTLECOUNT_CH0, 0x000a);
    // settlecount > 3

    fdc2112_write_register(pio, sm, FDC2112_REG_CLOCK_DIVIDERS_CH0, 0x2002);
    fdc2112_write_register(pio, sm, FDC2112_REG_ERROR_CONFIG, 0x0000);
    // 0000 0010 0000 1101 = 0x020d
    fdc2112_write_register(pio, sm, FDC2112_REG_MUX_CONFIG, 0x020d);
    fdc2112_write_register(pio, sm, FDC2112_REG_DRIVE_CURRENT_CH0, 0x0000);
    // 0001 0100 0000 0001 = 0x1401
    fdc2112_write_register(pio, sm, FDC2112_REG_CONFIG, 0x1401);
}

// Use queue
void core2_main(){
    while(true){
        levitation_state_t datapoint;
        queue_remove_blocking(&data_queue, &datapoint);
        printf("Data: 0x%04x 0x%04x 0x%04x 0x%04x\n", datapoint.data0, datapoint.data1, datapoint.data2, datapoint.data3);
        //sleep_ms(10);
    }
}

int main() {

    // enable all available stdio outputs
    // (currently only USB in CMakeLists.txt)
    // USB serial port defaults to 115200 baud
    stdio_init_all();


    // launch second core
    // queue holds at most 2 data points
    queue_init(&data_queue, sizeof(levitation_state_t), 2);
    multicore_launch_core1(core2_main);


    // set up LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // set up I2C
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

    //i2c_init(i2c0, 400 * 1000); // i2c fast mode
    //i2c_init(i2c1, 400 * 1000); // i2c fast mode
    //gpio_set_function(4, GPIO_FUNC_I2C); // i2c0 SDA, Pico 6, MISO-d
    //gpio_set_function(5, GPIO_FUNC_I2C); // i2c0 SCL, Pico 7, CS-d
    //gpio_set_function(6, GPIO_FUNC_I2C); // i2c1 SDA, Pico 9, SCLK-d
    //gpio_set_function(7, GPIO_FUNC_I2C); // i2c1 SCL, Pico 10, MOSI-d
    //gpio_set_function(8, GPIO_FUNC_I2C); // i2c0 SDA, Pico 11, SDA-d
    //gpio_set_function(9, GPIO_FUNC_I2C); // i2c0 SCL, Pico 12, SCL-d
    //gpio_set_function(10, GPIO_FUNC_I2C); // i2c1 SDA, Pico 14, SDA4-d
    //gpio_set_function(11, GPIO_FUNC_I2C); // i2c1 SCL, Pico 15, SCL4-d
    //gpio_pull_up(4);
    //gpio_pull_up(5);
    //gpio_pull_up(6);
    //gpio_pull_up(7);
    //gpio_pull_up(8);
    //gpio_pull_up(9);
    //gpio_pull_up(10);
    //gpio_pull_up(11);


    /*
    for(uint i=20; i>0; i--){
        sleep_ms(300);
        printf("count %d\n", i);
    }
    */

    // call many times just in case
    sleep_ms(500);

    fdc2112_startup(pio, sm0);
    fdc2112_startup(pio, sm1);
    fdc2112_startup(pio, sm2);
    fdc2112_startup(pio, sm3);

    uint16_t result[] = {0x00, 0x00, 0x00, 0x00};
    fdc2112_read_register_4(pio, sm0, sm1, sm2, sm3, FDC2112_REG_DATA_CH0, result);

    gpio_put(PICO_DEFAULT_LED_PIN, true);
    while(true){
        //gpio_put(PICO_DEFAULT_LED_PIN, true);
        //gpio_put(PICO_DEFAULT_LED_PIN, false);

        fdc2112_read_register_nowrite_4(pio, sm0, sm1, sm2, sm3, result);

        sleep_us(50);

        // send data to second core for printing
        levitation_state_t entry = {result[0], result[1], result[2], result[3]};
        queue_try_add(&data_queue, &entry); // nonblocking

    }

    return 0;
}

