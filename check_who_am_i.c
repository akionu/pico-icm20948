#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

static int addr = 0x68;

int main (void) {
    stdio_init_all();

    i2c_init(i2c1, 400 * 1000); // 100kHz
    gpio_set_function(14, GPIO_FUNC_I2C);
    gpio_set_function(15, GPIO_FUNC_I2C);

    uint8_t buf[2];
    uint8_t val = 0x00;
    
    while (1) {
        i2c_write_blocking(i2c1, addr, &val, 1, true);
        i2c_read_blocking(i2c1, addr, buf, 2, false);
        printf("WHO_AM_I: 0x%X\n", buf[0]); // sensor return 0xEA
        sleep_ms(1000);
    }
    
    return 0;
}