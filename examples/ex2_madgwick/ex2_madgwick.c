#include <stdio.h>
#include <time.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../../MadgwickAHRS/MadgwickAHRS.h"
#include "../../src/pico-icm20948.h"

#define I2C_SDA 14
#define I2C_SCL 15

i2c_inst_t icm20948_i2c = {i2c1_hw, false};
icm20948_config_t config = {0x68, 0x0C, &icm20948_i2c};
madgwick_ahrs_t filter = {0.5f, {1.0f, 0.0f, 0.0f, 0.0f}};
icm20984_data_t data;

bool dataflag = false;

bool read_icm20948(repeating_timer_t *rt) {
    icm20948_read_cal_accel(&config, &data.accel_raw[0], &data.accel_bias[0]);
    icm20948_read_cal_gyro(&config, &data.gyro_raw[0], &data.gyro_bias[0]);
    icm20948_read_cal_mag(&config, &data.mag_raw[0], &data.mag_bias[0]);
    icm20948_read_temp_c(&config, &data.temp_c);
    dataflag = true;
    return true;
}

void q2e(madgwick_ahrs_t *data, float euler[]) {
    // 0: roll, 1: pitch, 2: yaw
    euler[0] = -1.0f * asinf(2.0f * (data->q[1]) * (data->q[3]) + 2.0f * (data->q[0]) * (data->q[2]));
    euler[1] = atan2f(2.0f * (data->q[2]) * (data->q[3]) - 2.0f * (data->q[0]) * (data->q[1]), 2.0f * (data->q[0]) * (data->q[0]) + 2.0f * (data->q[3]) * (data->q[3]) - 1.0f);
    euler[2] = atan2f(2.0f * (data->q[1]) * (data->q[2]) - 2.0f * (data->q[0]) * (data->q[3]), 2.0f * (data->q[0]) * (data->q[0]) + 2.0f * (data->q[1]) * (data->q[1]) - 1.0f);
    return;
}

int main(void) {
    stdio_init_all();

    i2c_init(&icm20948_i2c, 400 * 1000); // 400kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    sleep_ms(2000);
    printf("hello, this is pico!\n");
    if (icm20948_init(&config) == 0) printf("successfully initialized!\n");
    icm20948_cal_gyro(&config, &data.gyro_bias[0]);
    printf("calibrated gyro: %d %d %d\n", data.gyro_bias[0], data.gyro_bias[1], data.gyro_bias[2]);
    icm20948_cal_accel(&config, &data.accel_bias[0]);
    printf("calibrated accel: %d %d %d\n", data.accel_bias[0], data.accel_bias[1], data.accel_bias[2]);
// set mag_bias manually (add cal func later)
    data.mag_bias[0] = -102; data.mag_bias[1] = 38; data.mag_bias[2] = 146;
    sleep_ms(2000);
    printf("calibration done\n");

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

            MadgwickAHRSupdate(&filter, gyro_dps[0], gyro_dps[1], gyro_dps[2], accel_g[0] * 9.8, accel_g[1] * 9.8, accel_g[2] * 9.8, mag_ut[0], mag_ut[1], mag_ut[2]);
            //MadgwickAHRSupdateIMU(&filter, gyro_dps[0], gyro_dps[1], gyro_dps[2], accel_g[0] * 9.8, accel_g[1] * 9.8, accel_g[2] * 9.8);
            float euler[3];
            q2e(&filter, euler); // rad

            //printf("(data->q[0]) %0.1f, (data->q[1]) %0.1f, (data->q[2]) %0.1f, (data->q[3]) %0.1f\n", (data->q[0]), (data->q[1]), (data->q[2]), (data->q[3]));
            //printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler[0] * 57.29578f, euler[1] * 57.29578f, euler[2] * 57.29578f + 180.0f);
            printf("D %0.1f %0.1f %0.1f\n", euler[2] * 57.29578f, euler[1] * 57.29578f, euler[0] * 57.29578f);

            //printf("%.0f,%.0f,%.0f\n", mag_ut[0], mag_ut[1], mag_ut[2]);
            #if 0
            // accel(g)   = raw_value / (65535 / full_scale)
            // ex) if full_scale == +-4g then accel = raw_value / (65535 / 8) = raw_value / 8192
            // gyro(dps)  = raw_value / (65535 / full_scale)
            // ex) if full_scale == +-250dps then gyro = raw_value / (65535 / 500) = raw_value / 131
            // mag(uT)    = raw_value / (32752 / 4912) = (approx) raw_value / 20 * 3
            printf("accel. x: %+2.5f, y: %+2.5f, z:%+2.5f\n", accel_g[0], accel_g[1], accel_g[2]);
            printf("gyro.  x: %+2.5f, y: %+2.5f, z:%+2.5f\n", gyro_dps[0], gyro_dps[1], gyro_dps[2]);
            printf("mag.   x: %+2.5f, y: %+2.5f, z:%+2.5f\n", mag_ut[0], mag_ut[1], mag_ut[2]);
            printf("temp: %+2.5f\n", data.temp_c);
            #endif
        }
    }
    
    return 0;
}