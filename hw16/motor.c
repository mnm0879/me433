/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>

#define LWHEEL_PWM_PIN 15
#define LWHEEL_STATE_PIN 14
#define RWHEEL_PWM_PIN 13
#define RWHEEL_STATE_PIN 12

// define pwm parameters
const uint16_t wrap = 12500;
const float div = 1; // must be between 1-255

void init_motors() {
    // LEFT SIDE
    gpio_set_function(LWHEEL_PWM_PIN, GPIO_FUNC_PWM);
    uint lslice_num = pwm_gpio_to_slice_num(LWHEEL_PWM_PIN);
    pwm_set_clkdiv(lslice_num, div); // divide
    pwm_set_wrap(lslice_num, wrap);
    pwm_set_enabled(lslice_num, true); // turn on the PWM

    pwm_set_gpio_level(LWHEEL_PWM_PIN, 0); // initialize to zero

    gpio_init(LWHEEL_STATE_PIN);
    gpio_set_dir(LWHEEL_STATE_PIN, GPIO_OUT);

    // RIGHT SIDE
    gpio_set_function(RWHEEL_PWM_PIN, GPIO_FUNC_PWM);
    uint rslice_num = pwm_gpio_to_slice_num(RWHEEL_PWM_PIN);
    pwm_set_clkdiv(rslice_num, div); // divide
    pwm_set_wrap(rslice_num, wrap);
    pwm_set_enabled(rslice_num, true); // turn on the PWM

    pwm_set_gpio_level(RWHEEL_PWM_PIN, 0); // initialize to zero

    gpio_init(RWHEEL_STATE_PIN);
    gpio_set_dir(RWHEEL_STATE_PIN, GPIO_OUT);
}

void write_motor_powers(float lpower, float rpower) {
    // check if reversed
    if(lpower >= 0) {
        pwm_set_gpio_level(LWHEEL_PWM_PIN, wrap * lpower);
        gpio_put(LWHEEL_STATE_PIN, 0);
    }
    else {
        lpower = 1 + lpower;
        pwm_set_gpio_level(LWHEEL_PWM_PIN, wrap * lpower);
        gpio_put(LWHEEL_STATE_PIN, 1);
    }
    if(rpower >= 0) {
        pwm_set_gpio_level(RWHEEL_PWM_PIN, wrap * rpower);
        gpio_put(RWHEEL_STATE_PIN, 0);
    }
    else {
        rpower = 1 + rpower;
        pwm_set_gpio_level(RWHEEL_PWM_PIN, wrap * rpower);
        gpio_put(RWHEEL_STATE_PIN, 1);
    }
}

void motors_from_centerline(int centerline) {
    // this is for the left motor
    float cutoff = 30.0;
    float minimum = .25;

    // first go through the process for left motor, then do the same for 100-pos and right motor
    float lpower = 1;
    if(centerline < cutoff) {
        lpower = centerline/cutoff;
        lpower = lpower * (1 - minimum) + minimum;
    }
    float rpower = 1;
    centerline = 100 - centerline;
    if(centerline < cutoff) {
        rpower = centerline/cutoff;
        rpower = rpower * (1 - minimum) + minimum;
    }
    
    // write the powers
    printf("Got a left motor power of %f\r\n", lpower);
    printf("Got a right motor power of %f\r\n", rpower);
    write_motor_powers(lpower, rpower);
}

int main() {

    // need this for it to behave like a COM port.
    stdio_init_all();

    // setup output
    init_motors();

    // for debugging
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // initialize scanf
    int pos;

    while (true) {
        printf("Please enter the read position:\t\r\n");
        scanf("%d", &pos);
        motors_from_centerline(pos);
    }
}
