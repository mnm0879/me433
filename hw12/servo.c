/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>

#define SERVO_PIN 15

const uint16_t wrap = 50000;

void init_servo() {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    float div = 50; // must be between 1-255
    pwm_set_clkdiv(slice_num, div); // divide
    pwm_set_wrap(slice_num, wrap);
    pwm_set_enabled(slice_num, true); // turn on the PWM

    pwm_set_gpio_level(SERVO_PIN, 0); // initialize to zero
}

void write_servo_angle(float angle) {
    float low_bound = .025;
    float up_bound = .125;
    float duty = (angle / 180) * (up_bound - low_bound) + low_bound;
    pwm_set_gpio_level(SERVO_PIN, duty * wrap); // set the duty cycle to 50%
}

void move_servo_angle(float angle1, float angle2, float duration_ms) {
    float dt = 10; // 10 ms update rate
    float dtheta = (angle2 - angle1)/duration_ms; // change in angle per second
    int steps = (int) duration_ms/dt;

    for(int i = 0; i < steps; i++) {
        write_servo_angle(dtheta * dt * i + angle1);
        sleep_ms(dt);
    }
}

int main() {

    // need this for it to behave like a COM port.
    stdio_init_all();

    // setup output
    init_servo();

    // for debugging
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (true) {
        gpio_put(LED_PIN, 0);
        move_servo_angle(0, 180, 4000.0);
        printf("Went from 0 to 180\r\n");
        gpio_put(LED_PIN, 1);
        move_servo_angle(180, 0, 4000.0);
        printf("Went from 180 to 0\r\n");
    }
}
