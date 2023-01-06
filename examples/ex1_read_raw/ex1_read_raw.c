#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../../src/pico-icm20948.h"

i2c_inst_t icm20948_i2c = {i2c1_hw, false};
icm20948_config_t config = {0x68, 0x0C, &icm20948_i2c};
icm20984_data_t data;

int main(void) {
    stdio_init_all();

    i2c_init(&icm20948_i2c, 400 * 1000); // 400kHz
    gpio_set_function(14, GPIO_FUNC_I2C);
    gpio_set_function(15, GPIO_FUNC_I2C);

    sleep_ms(2000);
    printf("hello, this is pico!\n");
    if (icm20948_init(&config) == 0) printf("successfully initialized!\n");

    int16_t accel_raw[3] = {0}, gyro_raw[3] = {0}, mag_raw[3] = {0}, temp_raw = 0;
    float accel_g[3] = {0}, gyro_dps[3] = {0}, mag_ut[3] = {0}, temp_c = 0;

    while(1) {
        icm20948_read_raw_accel(&config, accel_raw);
        icm20948_read_raw_gyro(&config, gyro_raw);
        icm20948_read_raw_mag(&config, mag_raw);
        icm20948_read_temp_c(&config, &temp_c);

        for (uint8_t i = 0; i < 3; i++) {
            accel_g[i] = (float)accel_raw[i] / 16384.0f;
            gyro_dps[i] = (float)gyro_raw[i] / 131.0f;
            mag_ut[i] = ((float)mag_raw[i] / 20) * 3;
        }

        #if 1
        // accel(g)   = raw_value / (65535 / full_scale)
        // ex) if full_scale == +-4g then accel = raw_value / (65535 / 8) = raw_value / 8192
        // gyro(dps)  = raw_value / (65535 / full_scale)
        // ex) if full_scale == +-250dps then gyro = raw_value / (65535 / 500) = raw_value / 131
        // mag(uT)    = raw_value / (32752 / 4912) = (approx) (raw_value / 20) * 3
        // temp  = ((raw_value - ambient_temp) / speed_of_sound) + 21
        printf("accel. x: %+2.5f, y: %+2.5f, z:%+2.5f\n", accel_g[0], accel_g[1], accel_g[2]);
        printf("gyro.  x: %+2.5f, y: %+2.5f, z:%+2.5f\n", gyro_dps[0], gyro_dps[1], gyro_dps[2]);
        printf("mag.   x: %+2.5f, y: %+2.5f, z:%+2.5f\n", mag_ut[0], mag_ut[1], mag_ut[2]);
        printf("temp: %+2.5f\n", temp_c);
        #endif
        sleep_ms(10);
    }
    
    return 0;
}