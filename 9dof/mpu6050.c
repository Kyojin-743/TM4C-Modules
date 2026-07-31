/*
 * mpu6050.c
 *
 *  Created on: Mar 24, 2026
 *      Author: kyojin
 */

#include <stdint.h>
#include <stdbool.h>

#include "mpu6050.h"
#include "i2c1.h"

//TODO: Add timeout for i2c so if no resp -> can err out (atm always resolves true or inf loop)
bool mpu_writeReg(uint8_t reg, uint8_t data) {
    writeI2c1Register(MPU6050_ADDR, reg, data);
    return true;
}

bool mpu_readReg(uint8_t reg, uint8_t* data) {
    *data = readI2c1Register(MPU6050_ADDR, reg);
    return true;
}

bool mpu_readMulti(uint8_t reg, uint8_t* data, uint8_t len) {
    readI2c1Registers(MPU6050_ADDR, reg, data, len);
    return true;
}

//TODO: make optional to allow isr or polling
uint8_t g_fsr = 0, a_fsr = 0;
bool mpu_init(void) {
    bool ok = true;

//    ok &= mpu_writeReg(MPU6050_PWR_MGMT_1_R, MPU6050_PWR_MGMT_1_RESET);

    //sample rate = gyro_outp_rate / (1 + SMPLRT_DIV)
    ok &= mpu_writeReg(MPU6050_SMPLRT_DIV_R, 0x00);

    ok &= mpu_writeReg(MPU6050_PWR_MGMT_1_R,
                       MPU6050_PWR_MGMT_1_TEMP_DISABLE |
                       MPU6050_PWR_MGMT_1_CLKSEL_GYR_X);

    ok &= mpu_writeReg(MPU6050_CONFIG_R,
                       MPU6050_CONFIG_DLPF_A184_G188 |
                       MPU6050_CONFIG_FSYNC_DISABLE);

    ok &= mpu_writeReg(MPU6050_GYRO_CONFIG_R, MPU6050_GYRO_CONFIG_FSSEL_250);

    ok &= mpu_writeReg(MPU6050_ACCEL_CONFIG_R, MPU6050_ACCEL_CONFIG_FSSEL_2);

    ok &= mpu_writeReg(MPU6050_INT_PIN_CFG_R, MPU6050_INT_PIN_CFG_RD_CLEAR);

    ok &= mpu_writeReg(MPU6050_INT_ENABLE_R, MPU6050_INT_ENABLE_DATA_READY);

    return ok;
}

// MPU6050 Acceleration Conversion
/* +-----+---------+--------------+ *
 * | BIT |   FSR   |   LSB Sens.  | *
 * +-----+---------+--------------+ *
 * |  0  |  ± 2g   |  2^14 LSB/g  | *
 * |  1  |  ± 4g   |  2^13 LSB/g  | *
 * |  2  |  ± 8g   |  2^12 LSB/g  | *
 * |  3  |  ± 16g  |  2^11 LSB/g  | *
 * +-----+---------+--------------+ */

//      MPU6050 Gyroscope Conversion
/* +-----+--------------+-----------------+ *
 * | BIT |     FSR      |     LSB Sens.   | *
 * +-----+--------------+-----------------+ *
 * |  0  |  ± 250  °/s  | 131.0 LSB/(°/s) | *
 * |  1  |  ± 500  °/s  | 65.50 LSB/(°/s) | *
 * |  2  |  ± 1000 °/s  | 32.80 LSB/(°/s) | *
 * |  3  |  ± 2000 °/s  | 16.40 LSB/(°/s) | *
 * +-----+--------------+-----------------+ */
//Range is always ~±32768
#define ACCEL_SENS      16384.0
#define GYRO_SENS       131.0

MpuData mpu_read(void) {
    uint8_t i = 0;
    uint8_t buffer[14];
    MpuData m;
    mpu_readMulti(MPU6050_ACCEL_XOUT_H_R, buffer, 14);
    m.a.x = (float)((int16_t)(buffer[i++] << 8 | buffer[i++])) / ACCEL_SENS;
    m.a.y = (float)((int16_t)(buffer[i++] << 8 | buffer[i++])) / ACCEL_SENS;
    m.a.z = (float)((int16_t)(buffer[i++] << 8 | buffer[i++])) / ACCEL_SENS;
    i+= 2;  //Skip Temp
    m.g.x = (float)((int16_t)(buffer[i++] << 8 | buffer[i++])) / GYRO_SENS;
    m.g.y = (float)((int16_t)(buffer[i++] << 8 | buffer[i++])) / GYRO_SENS;
    m.g.z = (float)((int16_t)(buffer[i++] << 8 | buffer[i++])) / GYRO_SENS;
    return m;
}

