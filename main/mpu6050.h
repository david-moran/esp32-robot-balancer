#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <stdint.h>

#include "esp_err.h"

#define BALANC3R_MPU6050_I2C_ADDRESS    0b1101000


typedef enum {
    GYRO_X_REGISTER=0x43,
    GYRO_Y_REGISTER=0x45,
    GYRO_Z_REGISTER=0x47
} gyro_register_t;



esp_err_t b_mpu6050_write(uint8_t address, uint8_t reg, const uint8_t *bytes,
    uint8_t size);


esp_err_t b_mpu6050_read(uint8_t address, uint8_t reg, uint8_t *bytes,
    uint8_t size);

esp_err_t b_mpu6050_gyro_value(gyro_register_t reg, int16_t *out);

esp_err_t b_mpu6050_calibrate(uint16_t iterations);

#endif // _MPU6050_H_