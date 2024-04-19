#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

 /* Example code to talk to a BMP280 temperature and pressure sensor

    NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
    GPIO (and therefore I2C) cannot be used at 5v.

    You will need to use a level shifter on the I2C lines if you want to run the
    board at 5v.

    Connections on Raspberry Pi Pico board, other boards may vary.

    GPIO PICO_DEFAULT_I2C_SDA_PIN (on Pico this is GP4 (pin 6)) -> SDA on BMP280
    board
    GPIO PICO_DEFAULT_I2C_SCK_PIN (on Pico this is GP5 (pin 7)) -> SCL on
    BMP280 board
    3.3v (pin 36) -> VCC on BMP280 board
    GND (pin 38)  -> GND on BMP280 board
 */
#define ADDR _u(0b0100000)
#define IODIR 0x00
#define PORT 0x09 // reads
#define LAT 0x0A // write


void chip_init() {
    // use the "handheld device dynamic" optimal setting (see datasheet)
    uint8_t buf[2];

    // send register number followed by its corresponding value
    buf[0] = IODIR; // register 0 is TRIS
    buf[1] = 0b00; // i/o in binary. this is all output
    i2c_write_blocking(i2c_default, ADDR, buf, 2, false);
}

// generic version would have REG_CONFIG and reg_config_val as arguments
void chip_set(uint8_t reg, uint8_t val) {
    // use the "handheld device dynamic" optimal setting (see datasheet)
    uint8_t buf[2];

    // send register number followed by its corresponding value
    buf[0] = reg; // register 0 is TRIS
    buf[1] = val; // i/o in binary, ex: 0b01111111
    i2c_write_blocking(i2c_default, ADDR, buf, 2, false);
}

unsigned char chip_read(uint8_t reg) {
    // BMP280 data registers are auto-incrementing and we have 3 temperature and
    // pressure registers each, so we start at 0xF7 and read 6 bytes to 0xFC
    // note: normal mode does not require further ctrl_meas and config register writes

    uint8_t buf[1];
    i2c_write_blocking(i2c_default, ADDR, &reg, 1, true);  // true to keep master control of bus
    i2c_read_blocking(i2c_default, ADDR, buf, 1, false);  // false - finished with bus

    return buf[0];
}

int main() {
    stdio_init_all();

    // I2C is "open drain", pull ups to keep signal high when no data is being sent
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // configure chip
    chip_init();

    sleep_ms(250); // sleep so that data polling and register update don't collide

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    uint8_t LED_VAL = 1;

    while (1) {
        //first thing. do a blink
        // chip_set(LAT, 0b10000000);
        // sleep_ms(250);
        // printf("HIGH\r\n");
        // chip_set(LAT, 0b00000000);
        // sleep_ms(250);
        // printf("LOW\r\n");

        //now read a button
        uint8_t state = chip_read(PORT);
        printf("State: %04x\r\n", state);
        if(state == 1) {
            chip_set(LAT, 0b10000000);
        }
        else {
            chip_set(LAT, 0b00000000);
        }
        sleep_ms(1);
    }
}
