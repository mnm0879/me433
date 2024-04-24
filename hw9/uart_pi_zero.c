/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/uart.h"
#include "hardware/irq.h"

#define MAX_LENGTH 50

// DEFINITIONS FOR UART
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// PINS
#define UART_TX_PIN 0
#define UART_RX_PIN 1

static unsigned char buffer[MAX_LENGTH];
static uint8_t idx = 0;

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

// RX interrupt handler
void on_uart_rx() {
    while (uart_is_readable(UART_ID)) {      
        uint8_t ch = uart_getc(UART_ID);
        if(ch == '\n' || ch == '\r') {
            // insert null character and reset index
            buffer[idx] = 0;
            idx = 0;

            // printf the buffer
            printf(buffer);
            printf("\r\n");
        }
        else {
            buffer[idx] = ch;
            idx++;
            gpio_put(LED_PIN, !gpio_get(LED_PIN));
        }
    }
}


int main() {

    // need this for it to behave like a COM port.
    stdio_init_all();

    // LED init
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // uart initialization
    uart_init(UART_ID, 2400);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

    // storage variables
    int recieved;
    char message[MAX_LENGTH];

    while(1) {
        // inquiry
        printf("Please enter an integer: \r\n");
        scanf("%d", &recieved);

        // echo back to computer
        printf("Recieved integer: %d\r\n", recieved);

        // send over UART
        sprintf(message, "%d", recieved);
        uart_puts(uart0, message);
    }
}
