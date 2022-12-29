#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "icm20948-register.h"

#define I2C_BUS (__CONCAT(i2c, 1))
uint8_t addr_accel_gyro = 0x68;
uint8_t addr_mag = 0x0C;

int8_t icm20948_init();
void icm20948_read_raw_accel(int16_t accel[3]);
void icm20948_read_raw_gyro(int16_t gyro[3]);
void icm20948_read_raw_temp(int16_t *temp);
void icm20948_set_mag_mode(uint8_t mode);
void icm20948_read_raw_mag(int16_t mag[3]);

void icm20948_cal_gyro(int16_t gyro_bias[3]);

int8_t icm20948_init() {
    uint8_t reg[2], buf;

    // wake up accel/gyro!
    // first write register then, write value
    reg[0] = PWR_MGMT_1; reg[1] = 0x00;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, reg, 2, false);

    // switch to user bank to 0
    reg[0] = REG_BANK_SEL; reg[1] = 0x00;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, reg, 2, false);

    // auto select clock source
    reg[0] = PWR_MGMT_1; reg[1] = 0x01;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, reg, 2, false);

    // disable accel/gyro once
    reg[0] = PWR_MGMT_2; reg[1] = 0x3F;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, reg, 2, false);
    sleep_ms(10);

    // enable accel/gyro (again)
    reg[0] = PWR_MGMT_2; reg[1] = 0x00;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, reg, 2, false);

    // check if the accel/gyro could be accessed
    reg[0] = WHO_AM_I_ICM20948;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, reg, 1, true);
    i2c_read_blocking(I2C_BUS, addr_accel_gyro, &buf, 1, false);
#ifndef NDEBUG    
    printf("AG. WHO_AM_I: 0x%X\n", buf);
#endif
    if (buf != 0xEA) return -1;

    // switch to user bank 2
    reg[0] = REG_BANK_SEL; reg[1] = 0x20;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, reg, 2, false);

    // gyro config
    //
    // set full scale to +-
    // set noise bandwidth to 
    // smaller bandwidth means lower noise level & slower max sample rate
    reg[0] = GYRO_CONFIG_1; reg[1] = 0x29;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, reg, 2, false);
    //
    // set gyro output data rate to 100Hz
    // output_data_rate = 1.125kHz / (1 + GYRO_SMPLRT_DIV)
    // 1125 / 11 = 100
    reg[0] = GYRO_SMPLRT_DIV; reg[1] = 0x0A;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, reg, 2, false);

    // accel config
    //
    // set full scale to +-2g
    // set noise bandwidth to 136Hz
    reg[0] = ACCEL_CONFIG; reg[1] = 0x11;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, reg, 2, false);
    //
    // set accel output data rate to 100Hz
    // output_data_rate = 1.125kHz / (1 + ACCEL_SMPLRT_DIV)
    // 16 bits for ACCEL_SMPLRT_DIV
    reg[0] = ACCEL_SMPLRT_DIV_2; reg[1] = 0x0A;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, reg, 2, false);
    
    // switch to user bank to 0
    reg[0] = REG_BANK_SEL; reg[1] = 0x00;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, reg, 2, false);
    
    // wake up mag! (INT_PIN_CFG, BYPASS_EN = 1)
    reg[0] = INT_PIN_CFG; reg[1] = 0x02;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, reg, 2, false);

    // check if the mag could be accessed
    reg[0] = 0x01;
    i2c_write_blocking(I2C_BUS, addr_mag, reg, 1, true);
    i2c_read_blocking(I2C_BUS, addr_mag, &buf, 1, false);
#ifndef NDEBUG
    printf("MAG. WHO_AM_I: 0x%X\n", buf);
#endif
    if (buf != 0x09) return -1;

    // config mag
    //
    // set mag mode, to measure continuously in 100Hz
    reg[0] = AK09916_CNTL2; reg[1] = 0x08;
    i2c_write_blocking(I2C_BUS, addr_mag, reg, 2, false);

    return 0;
}

void icm20948_read_raw_accel(int16_t accel[3]) {
    uint8_t buf[6];

    // accel: 2 bytes each axis
    uint8_t reg = ACCEL_XOUT_H;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, &reg, 1, true);
    i2c_read_blocking(I2C_BUS, addr_accel_gyro, buf, 6, false);

    for (uint8_t i = 0; i < 3; i++) accel[i] = (buf[i * 2] << 8 | buf[(i * 2) + 1]);
    
    return;
}

void icm20948_read_raw_gyro(int16_t gyro[3]) {
    uint8_t buf[6];

    // gyro: 2byte each axis
    uint8_t reg = GYRO_XOUT_H;
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, &reg, 1, true);
    i2c_read_blocking(I2C_BUS, addr_accel_gyro, buf, 6, false);

    for (uint8_t i = 0; i < 3; i++) gyro[i] = (buf[i * 2] << 8 | buf[(i * 2) + 1]);

    return;
}

void icm20948_read_raw_temp(int16_t *temp) {
    uint8_t reg = TEMP_OUT_H, buf[2];
    i2c_write_blocking(I2C_BUS, addr_accel_gyro, &reg, 1, true);
    i2c_read_blocking(I2C_BUS, addr_accel_gyro, buf, 2, false);

    *temp = (buf[0] << 8 | buf[1]);

    return;
}

#if 0
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

    reg[0] = AK09916_CNTL2;
    i2c_write_blocking(I2C_BUS, addr_mag, reg, 2, false);

    return;
}
#endif

void icm20948_read_raw_mag(int16_t mag[3]) {
    uint8_t buf[8];

    uint8_t reg = AK09916_XOUT_L;
    i2c_write_blocking(I2C_BUS, addr_mag, &reg, 1,true);
    i2c_read_blocking(I2C_BUS, addr_mag, buf, 8, false);

    for (int i = 0; i < 3; i++) mag[i] = (buf[(i * 2) + 1] << 8 | buf[(i * 2)]);

#ifndef NDEBUG
    if ((buf[6] & 0x08) == 0x08) printf("mag: ST1: Sensor overflow\n");

    // printf below works only if we read 0x10
    //if ((buf[0] & 0x01) == 0x01) printf("mag: ST1: Data overrun\n");
    //if ((buf[0] & 0x02) != 0x02) printf("mag: ST1: Data is NOT ready\n");
#endif

    return;
}

void icm20948_cal_gyro(int16_t gyro_bias[3]) {
    int16_t tmp[3] = {0};
    int32_t bias[3] = {0};

    for (uint8_t i = 0; i < 200; i++) {
        icm20948_read_raw_gyro(tmp);
        for (uint8_t j = 0; j < 3; j++) {
            bias[j] += tmp[j];
        }
        sleep_ms(25);
    }
    for (uint8_t i = 0; i < 3; i++) gyro_bias[i] = (int16_t)(bias[i] / 200);
    
    return;
}

void icm20948_read_cal_gyro(int16_t gyro[3], int16_t bias[3]) {
    icm20948_read_raw_gyro(gyro);
    for (uint8_t i = 0; i < 3; i++) gyro[i] -= bias[i];
    return;
}

void icm20948_cal_accel(int16_t accel_bias[3]) {
    int16_t tmp[3] = {0};
    int32_t bias[3] = {0};

    for (uint8_t i = 0; i < 100; i++) {
        icm20948_read_raw_accel(tmp);
        for (uint8_t j = 0; j < 3; j++) {
            if (j == 2) bias[j] += (tmp[j] - 16384);
            else bias[j] += tmp[j];
        }
        sleep_ms(20);
    }
    for (uint8_t i = 0; i < 3; i++) accel_bias[i] = (int16_t)(bias[i] / 100);
    return;
}

void icm20948_read_cal_accel(int16_t accel[3], int16_t bias[3]) {
    icm20948_read_raw_accel(accel);
    for (uint8_t i = 0; i < 3; i++) accel[i] -= bias[i];
    return;
}