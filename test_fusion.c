#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "./Fusion/Fusion.h"
#include "pico-icm20948.c"

#define SAMPLE_PERIOD (0.01f)
#define SAMPLE_RATE (100)

typedef struct raw_data {
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t mag_raw[3];
    int16_t temp_raw;
} raw_data_t;

raw_data_t raw;

bool dataflag = false;

bool read_icm20948(repeating_timer_t *rt) {
    icm20948_read_raw_accel(&raw.accel_raw[0]);
    icm20948_read_raw_gyro(&raw.gyro_raw[0]);
    icm20948_read_raw_temp(&raw.temp_raw);
    icm20948_read_raw_mag(&raw.mag_raw[0]);
    dataflag = true;
    return true;
}

int main(void) {
    stdio_init_all();

    i2c_init(I2C_BUS, 400 * 1000); // 400kHz
    gpio_set_function(14, GPIO_FUNC_I2C);
    gpio_set_function(15, GPIO_FUNC_I2C);

    sleep_ms(2000);
    printf("hello, this is pico!\n");
    if (icm20948_init() == 0) printf("successfully initialized!\n");

    static repeating_timer_t timer;
    add_repeating_timer_ms(-10, &read_icm20948, NULL, &timer);

    int16_t accel_raw[3] = {0}, gyro_raw[3] = {0}, mag_raw[3] = {0}, temp_raw = 0;
    float accel_g[3] = {0}, gyro_dps[3] = {0}, mag_ut[3] = {0}, temp_c = 0;

    const FusionMatrix gyroscopeMisalignment =  {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

    FusionOffset offset;
    FusionAhrs ahrs;

    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    const FusionAhrsSettings settings = {
            .gain = 0.9f,
            .accelerationRejection = 10.0f,
            .magneticRejection = 20.0f,
            .rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    while(1) {
        if (dataflag) {
            dataflag = false;
            // 0: x, 1: y, 2: z
        accel_g[0] = (float)(-1 * raw.accel_raw[1]) / 8192;
        accel_g[1] = (float)raw.accel_raw[0] / 8192;
        accel_g[2] = (float)raw.accel_raw[2] / 8192;
        gyro_dps[0] = (float)(-1 * raw.gyro_raw[1]) / 131;
        gyro_dps[1] = (float)raw.gyro_raw[0] / 131;
        gyro_dps[2] = (float)raw.gyro_raw[2] / 131;
        mag_ut[0] = (float)raw.mag_raw[1] / 20 * 3;
        mag_ut[1] = (float)raw.mag_raw[0] / 20 * 3;
        mag_ut[2] = (float)(-1 * raw.mag_raw[1]) / 20 * 3;
        temp_c = (((float)raw.temp_raw - 21) / 333.87) + 21;

        FusionVector gyro = {gyro_dps[0], gyro_dps[1], gyro_dps[2]};
        FusionVector accel = {accel_g[0], accel_g[1], accel_g[2]};
        FusionVector mag = {mag_ut[0] * 10, mag_ut[1] * 10, mag_ut[2] * 10};

        gyro = FusionCalibrationInertial(gyro, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
        accel = FusionCalibrationInertial(accel, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
        mag = FusionCalibrationMagnetic(mag, softIronMatrix, hardIronOffset);

        gyro = FusionOffsetUpdate(&offset, gyro);

        FusionAhrsUpdate(&ahrs, gyro, accel, mag, SAMPLE_PERIOD);
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

        printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",
                            euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
                            earth.axis.x, earth.axis.y, earth.axis.z);

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