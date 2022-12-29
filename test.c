#include <stdio.h>
#include <time.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "./MadgwickAHRS/MadgwickAHRS.h"
#include "pico-icm20948.c"

typedef struct icm20984_data {
    int16_t accel_raw[3];
    int16_t accel_bias[3];
    int16_t gyro_raw[3];
    int16_t gyro_bias[3];
    int16_t mag_raw[3];
    int16_t temp_raw;
} icm20984_data_t;

icm20984_data_t data;

bool dataflag = false;

bool read_icm20948(repeating_timer_t *rt) {
    icm20948_read_cal_accel(&data.accel_raw[0], &data.accel_bias[0]);
    icm20948_read_cal_gyro(&data.gyro_raw[0], &data.gyro_bias[0]);
    icm20948_read_raw_temp(&data.temp_raw);
    icm20948_read_raw_mag(&data.mag_raw[0]);
    dataflag = true;
    return true;
}

void q2e(float ret[]) {
    // 0: roll, 1: pitch, 2: yaw
    ret[0] = -1.0f * asinf(2.0f * q1 * q3 + 2.0f * q0 * q2);
    ret[1] = atan2f(2.0f * q2 * q3 - 2.0f * q0 * q1, 2.0f * q0 * q0 + 2.0f * q3 * q3 - 1.0f);
    ret[2] = atan2f(2.0f * q1 * q2 - 2.0f * q0 * q3, 2.0f * q0 * q0 + 2.0f * q1 * q1 - 1.0f);
    return;
}

int main(void) {
    stdio_init_all();

    i2c_init(I2C_BUS, 400 * 1000); // 400kHz
    gpio_set_function(14, GPIO_FUNC_I2C);
    gpio_set_function(15, GPIO_FUNC_I2C);

    sleep_ms(2000);
    printf("hello, this is pico!\n");
    if (icm20948_init() == 0) printf("successfully initialized!\n");
    icm20948_cal_gyro(&data.gyro_bias[0]);
    printf("calibrated gyro: %d %d %d\n", data.gyro_bias[0], data.gyro_bias[1], data.gyro_bias[2]);
    icm20948_cal_accel(&data.accel_bias[0]);
    printf("calibrated accel: %d %d %d\n", data.accel_bias[0], data.accel_bias[1], data.accel_bias[2]);
    sleep_ms(2000);

    static repeating_timer_t timer;
    add_repeating_timer_ms(-10, &read_icm20948, NULL, &timer);

    int16_t accel_raw[3] = {0}, gyro_raw[3] = {0}, mag_raw[3] = {0}, temp_raw = 0;
    float accel_g[3] = {0}, gyro_dps[3] = {0}, mag_ut[3] = {0}, temp_c = 0;

    while(1) {
        if (dataflag) {
            dataflag = false;
            // 0: x, 1: y, 2: z
            accel_g[0] = (float)data.accel_raw[0] / 16384.0f;
            accel_g[1] = (float)data.accel_raw[1] / 16384.0f;
            accel_g[2] = (float)data.accel_raw[2] / 16384.0f;
            gyro_dps[0] = (float)data.gyro_raw[0] / 131.0f;
            gyro_dps[1] = (float)data.gyro_raw[1] / 131.0f;
            gyro_dps[2] = (float)data.gyro_raw[2] / 131.0f;
            mag_ut[0] = (float)data.mag_raw[1];
            mag_ut[1] = (float)-data.mag_raw[0];
            mag_ut[2] = (float)-data.mag_raw[2];
            temp_c = (((float)data.temp_raw - 21.0f) / 333.87) + 21.0f;

            //MadgwickAHRSupdate(gyro_dps[0], gyro_dps[1], gyro_dps[2], accel_g[0] * 9.8, accel_g[1] * 9.8, accel_g[2] * 9.8, mag_ut[0], mag_ut[1], mag_ut[2]);
            MadgwickAHRSupdateIMU(gyro_dps[0], gyro_dps[1], gyro_dps[2], accel_g[0] * 9.8, accel_g[1] * 9.8, accel_g[2] * 9.8);
            float euler[3];
            q2e(euler);

            //printf("q0 %0.1f, q1 %0.1f, q2 %0.1f, q3 %0.1f\n", q0, q1, q2, q3);
            //printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler[0] * 57.29578f, euler[1] * 57.29578f, euler[2] * 57.29578f + 180.0f);
            printf("D %0.1f %0.1f %0.1f\n", euler[2] * 57.29578f, euler[1] * 57.29578f, euler[0] * 57.29578f);

            #if 0
            // accel(g)   = raw_value / (65535 / full_scale)
            // ex) if full_scale == +-4g then accel = raw_value / (65535 / 8) = raw_value / 8192
            // gyro(dps)  = raw_value / (65535 / full_scale)
            // ex) if full_scale == +-250dps then gyro = raw_value / (65535 / 500) = raw_value / 131
            // mag(uT)    = raw_value / (32752 / 4912) = (approx) raw_value / 20 * 3
            // temp  = ((raw_value - ambient_temp) / speed_of_sound) + 21
            printf("accel. x: %+2.5f, y: %+2.5f, z:%+2.5f\n", accel_g[0], accel_g[1], accel_g[2]);
            printf("gyro.  x: %+2.5f, y: %+2.5f, z:%+2.5f\n", gyro_dps[0], gyro_dps[1], gyro_dps[2]);
            printf("mag.   x: %+2.5f, y: %+2.5f, z:%+2.5f\n", mag_ut[0], mag_ut[1], mag_ut[2]);
            printf("temp: %+2.5f\n", temp_c);
            #endif
        }
    }
    
    return 0;
}