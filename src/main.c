#include <stdio.h>
#include "pico/stdlib.h"

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

    // LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    while (true) {
        printf("Hello, world!\n");
        //puts("Hello, world!\n");
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(250);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(250);
    }
}
