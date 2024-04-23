/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "ssd1306.h"
#include "font.h"
#include <stdio.h>
#include "hardware/adc.h"

#define MAX_LENGTH 50

void ssd1306_drawColumn(unsigned char x, unsigned char y, unsigned char data) {
    
    // iterate over every value in the 2 bytes and load them in the pixels
    for(unsigned char i = 0; i < 8; i++) {
        ssd1306_drawPixel(x, y + i, (data >> i) & 0b1);
    }
}

void ssd1306_drawChar(unsigned char x, unsigned char y, unsigned char value) {

    // offset the value by the designated amount
    value = value - 32;

    // iterate over the array and load it in
    for(unsigned char i = 0; i < 5; i++) {
        ssd1306_drawColumn(x + i, y, ASCII[value][i]);
    }
}

void ssd1306_writeMessage(unsigned char x, unsigned char y, unsigned char * message) {

    for(unsigned char i = 0; i < MAX_LENGTH; i++) {
        if(message[i] == 0) {
            return;
        }
        ssd1306_drawChar(x + 6 * i, y, message[i]);
    }
}

int main() {

    // need this for it to behave like a COM port.
    stdio_init_all();

    // for debugging
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // initialize adc
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);

    // need to initialize stuff
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);

    ssd1306_setup();

    sleep_ms(250);

    char message[MAX_LENGTH];
    unsigned int t, t_prev;
    t = 0;

    while(1) {
        // load in voltage
        sprintf(message, "Voltage: %f", (float) adc_read() * 3.3 / 4096);
        ssd1306_writeMessage(1, 1, message);

        // get frames per second
        t_prev = t;
        t = to_us_since_boot(get_absolute_time());
        sprintf(message, "FPS: %f", 1000000.0/(t - t_prev));
        ssd1306_writeMessage(1, 10, message);

        // write to the screen
        ssd1306_update();
        ssd1306_clear();        
    }
}
