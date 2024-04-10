#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

int main() {

    // initialize stdio
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    // initialize gpio
    const uint LED_PIN = 15;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    const uint BUTTON_PIN = 14;
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);

    // initialize adc
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);
    printf("Start!\n");
 
    // main loop
    while (1) {
        char message[100];
        int nsamples;
        uint16_t samples[100];

        // turn on an LED
        gpio_put(LED_PIN, 1);

        // wait for a button to press
        while(!gpio_get(BUTTON_PIN)) {}

        // turn off LED
        gpio_put(LED_PIN, 0);

        // ask for number of samples between 1 and 100
        printf("Please enter the desired number of samples, between 1 and 100\r\n");

        // read number entered by user
        scanf("%d", &nsamples);
        if(nsamples < 1 || nsamples > 100) {
            printf("Please enter a number of samples between 1 and 100");
        }
        else {
            // read voltage that number of times at 100Hz
            for(int i = 0; i < nsamples; i++) {
                samples[i] = adc_read();
                sleep_ms(10);
            }

            // print back voltages in units of volts
            for(int i = 0; i < nsamples; i++) {
                printf("Voltage #%d: %f\r\n", i + 1, (float) samples[i] * 3.3 / 4096);
            }
        }
    }
}