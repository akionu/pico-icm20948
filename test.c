#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_BUS (__CONCAT(i2c, 1))

static uint8_t addr_accel_gyro = 0x68;
static uint8_t addr_mag_read = 0x0C;

void icm20948_set_mag_mode(uint8_t mode);

int8_t icm20948_init() {
    uint8_t reg[2], buf;

    // check if the accel/gyro could be accessed
    reg[0] = 0x00;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, reg, 1, true);
    i2c_read_blocking(I2C_BUS, addr_accel_gyro, &buf, 1, false);
#ifndef NDEBUG    
    printf("AG. WHO_AM_I: 0x%X\n", buf);
#endif
    if (buf != 0xEA) return -1;

    // wake up accel/gyro! (PWR_MGMT_2)
    // first write register then, write value
    reg[0] = 0x06; reg[1] = 0x00;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, reg, 2, false);

    // wake up mag! (INT_PIN_CFG, BYPASS_EN = 1)
    reg[0] = 0x0F; reg[1] = 0x02;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, reg, 2, false);

    // check if the mag could be accessed
    reg[0] = 0x00;
    i2c_write_blocking(I2C_BUS, addr_mag_read, reg, 1, true);
    i2c_read_blocking(I2C_BUS, addr_mag_read, &buf, 1, false);
#ifndef NDEBUG    
    printf("MAG. COMPANY_ID: 0x%X\n", buf);
#endif
    if (buf != 0x48) return -1;

    // 10 Hz continuos data output 
    icm20948_set_mag_mode(1);

    return 0;
}

// "static" with function means not to tell this function to linker
// so, we can't call this function from other source files.
static void icm20948_read_raw_accel(int16_t accel[3]) {
    uint8_t buf[6];

    // accel: 2 bytes each axis
    uint8_t reg = 0x2D;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, &reg, 1, true);
    i2c_read_blocking(I2C_BUS, addr_accel_gyro, buf, 6, false);

    for (uint8_t i = 0; i < 3; i++) accel[i] = (buf[i * 2] << 8 | buf[(i * 2) + 1]);
    
    return;
}

static void icm20948_read_raw_gyro(int16_t gyro[3]) {
    uint8_t buf[6];

    // gyro: 2byte each axis
    uint8_t reg = 0x33;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, &reg, 1, true);
    i2c_read_blocking(I2C_BUS, addr_accel_gyro, buf, 6, false);

    for (uint8_t i = 0; i < 3; i++) gyro[i] = (buf[i * 2] << 8 | buf[(i * 2) + 1]);

    return;
}

static void icm20948_read_raw_temp(int16_t *temp) {
    uint8_t reg = 0x39, buf[2];
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, &reg, 1, true);
    i2c_read_blocking(I2C_BUS, addr_accel_gyro, buf, 2, false);

    *temp = (buf[0] << 8 | buf[1]);

    return;
}

void icm20948_set_mag_mode(uint8_t mode) {
    // Single measurement              : mode == 0
    // Continuous measurement in  10 Hz: mode == 1
    // Continuous measurement in  20 Hz: mode == 2
    // Continuous measurement in  50 Hz: mode == 3
    // Continuous measurement in 100 Hz: mode == 4
    uint8_t reg[2];

    switch (mode) {
        case 0:
            // single shot
            // after measure, transits to power-down mode automatically
            reg[1] = 0x01;
            break;
        case 1:
            // 10Hz continuous
            reg[1] = 0x02;
            break;
        case 2:
            // 20Hz continuous
            reg[1] = 0x04;
            break;
        case 3:
            // 50Hz continuous
            reg[1] = 0x06;
            break;
        case 4:
            // 100Hz continuous
            reg[1] = 0x08;
            break;
        default:
            return;
    }

    reg[0] = 0x31;
    i2c_write_blocking(I2C_BUS, addr_mag_read, reg, 2, false);

    return;
}

static void icm20948_read_raw_mag(int16_t mag[3]) {
    uint8_t buf[8];

    uint8_t reg = 0x11;
    i2c_write_blocking(I2C_BUS, addr_mag_read, &reg, 1,true);
    i2c_read_blocking(I2C_BUS, addr_mag_read, buf, 8, false);

    for (int i = 0; i < 3; i++) mag[i] = (buf[(i * 2) + 1] << 8 | buf[(i * 2)]);

#ifndef NDEBUG
    if ((buf[6] & 0x08) == 0x08) printf("mag: ST1: Sensor overflow\n");

    // printf below works only if we read 0x10
    //if ((buf[0] & 0x01) == 0x01) printf("mag: ST1: Data overrun\n");
    //if ((buf[0] & 0x02) != 0x02) printf("mag: ST1: Data is NOT ready\n");
#endif

    return;
}

int main (void) {
    stdio_init_all();

    i2c_init(I2C_BUS, 400 * 1000); // 400kHz
    gpio_set_function(14, GPIO_FUNC_I2C);
    gpio_set_function(15, GPIO_FUNC_I2C);

    sleep_ms(2000);
    printf("hello, this is pico!\n");
    icm20948_init();

    int16_t accel[3], gyro[3], mag[3], temp;

    while(1) {
        icm20948_read_raw_accel(accel);
        icm20948_read_raw_gyro(gyro);
        icm20948_read_raw_temp(&temp);
        icm20948_read_raw_mag(mag);

        printf("accel. x: %+2.5f, y: %+2.5f, z:%+2.5f\n", (float)accel[0] / 16384, (float)accel[1] / 16384, (float)accel[2] / 16384);
        printf("gyro.  x: %+2.5f, y: %+2.5f, z:%+2.5f\n", (float)gyro[0] / 250, (float)gyro[1] / 250, (float)gyro[2] / 250);
        printf("mag.   x: %+2.5f, y: %+2.5f, z:%+2.5f\n", (float)mag[0] / 4900, (float)mag[1] / 4900, (float)mag[2] / 4900);
        printf("temp: %+2.5f\n", (((float)temp - 21) / 333.87) + 21);

        sleep_ms(100);
    }
    
    return 0;
}