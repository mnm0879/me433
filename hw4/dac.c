#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "math.h"

static void write_register(uint8_t reg, uint16_t data) {
    // reg = which one writing to (A = 0, B = 1)
    // data = the value being written
    uint8_t buf[2];

    // determine which register
    buf[0] = reg;

    // always operate with buffer on, gain selection of 1x, and shutdown turned off
    buf[0] = (buf[0] << 7) | 0b1110000;

    // load the first 4 bits into D9-D6
    buf[0] = buf[0] | (data >> 6);

    // set it equal to data without the first 4 bits, then bitshift it twice
    buf[1] = data & 0b0000111111;
    buf[1] = buf[1] << 2;

    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);
    spi_write_blocking(spi_default, buf, 2);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    sleep_ms(10);
}

int main() {
    stdio_init_all();

    // This example will use SPI0 at 100kHz
    int spi_freq = 100000;
    spi_init(spi_default, spi_freq); // crank this bish up eventually
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);

    write_register(0, 1023); // trying to set A to 3.3
    write_register(1, 500);    // trying to set B to 0

    // generate the two waves
    int write_freq = 100; // frequency at which the wave is written
    int sin_wave[write_freq * 2];
    int triangle_wave[write_freq * 2];

    // load in sine wave
    for(int i = 0; i < write_freq * 2; i++) {
        sin_wave[i] = (int) (511.5 * sin(8 * 3.14159265358979/write_freq * i) + 511.5);
    }

    // load in triangle wave
    int direction = 1;
    int curr = 0;
    int increment = 45;
    for(int i = 0; i < write_freq * 2; i++) {
        if(direction == 1) {
            curr = curr + increment;
        }
        if(direction == -1) {
            curr = curr - increment;
        }
        if(curr > 1023) {
            curr = 1024 - increment;
            direction = -1;
        }
        else if(curr < 0) {
            curr = 0;
            direction = 1;
        }
        triangle_wave[i] = curr;
    }

    int idx = 0;

    while (1) {
        if(idx >= 2 * write_freq) {
            idx = 0;
        }

        write_register(0, sin_wave[idx]);
        write_register(1, triangle_wave[idx]);
        printf("Writing %d\r\n", triangle_wave[idx]);
        idx++;

        sleep_ms((int) 1000.0/write_freq);
    }
}