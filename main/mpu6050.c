#include "mpu6050.h"
#include "config.h"

#include "esp_err.h"
#include "driver/i2c.h"

static int16_t gyro_x_offset = 0;
static int16_t gyro_y_offset = 0;
static int16_t gyro_z_offset = 0;

esp_err_t b_mpu6050_write(
        uint8_t address,
        uint8_t reg,
        const uint8_t *bytes,
        uint8_t size) {

    uint8_t i;
    esp_err_t err;
    i2c_cmd_handle_t cmdh;

    cmdh = i2c_cmd_link_create();

    err = i2c_master_start(cmdh);
    if (err == ESP_FAIL) {
        goto RELEASE_CMD_HANDLER;
    }

    /* Address I2C device */
    err = i2c_master_write_byte(cmdh, (address << 1) | I2C_MASTER_WRITE, 1);
    if (err == ESP_FAIL) {
        goto RELEASE_CMD_HANDLER;
    }
    
    /* Address registry */
    err = i2c_master_write_byte(cmdh, reg | I2C_MASTER_WRITE, 1);
    if (err == ESP_FAIL) {
        goto RELEASE_CMD_HANDLER;
    }

    /* Start writing bytes */
    for (i = 0; i < size; i++) {
        err = i2c_master_write_byte(cmdh, bytes[i], 1);
        if (err == ESP_FAIL) {
            goto RELEASE_CMD_HANDLER;
        }
    }

    err = i2c_master_stop(cmdh);
    if (err == ESP_FAIL) {
        goto RELEASE_CMD_HANDLER;
    }

    err = i2c_master_cmd_begin(
        BALANC3R_I2C_MASTER_PORT, cmdh, 10 / portTICK_PERIOD_MS);

RELEASE_CMD_HANDLER:
    i2c_cmd_link_delete(cmdh);
    return err;
}


esp_err_t b_mpu6050_read(uint8_t address, uint8_t reg, uint8_t *bytes,
       uint8_t size) {
    
    uint8_t i;
    esp_err_t err;
    i2c_cmd_handle_t cmdh;

    cmdh = i2c_cmd_link_create();

    err = i2c_master_start(cmdh);
    if (err == ESP_FAIL) {
        goto RELEASE_CMD_HANDLER;
    }

    /* Address I2C device */
    err = i2c_master_write_byte(cmdh, (address << 1) | I2C_MASTER_WRITE, 1);
    if (err == ESP_FAIL) {
        goto RELEASE_CMD_HANDLER;
    }
    
    /* Address registry */
    err = i2c_master_write_byte(cmdh, reg | I2C_MASTER_WRITE, 1);
    if (err == ESP_FAIL) {
        goto RELEASE_CMD_HANDLER;
    }

    err = i2c_master_start(cmdh);
    if (err == ESP_FAIL) {
        goto RELEASE_CMD_HANDLER;
    }

    err = i2c_master_write_byte(cmdh, (address << 1) | I2C_MASTER_READ, 1);
    if (err == ESP_FAIL) {
        goto RELEASE_CMD_HANDLER;
    }

    for (i = 0; i < size - 1; i++) {
        err = i2c_master_read_byte(cmdh, &bytes[i], 0);
        if (err == ESP_FAIL) {
            goto RELEASE_CMD_HANDLER;
        }
    }

    err = i2c_master_read_byte(cmdh, &bytes[i], 1);
    if (err == ESP_FAIL) {
        goto RELEASE_CMD_HANDLER;
    }

    err = i2c_master_stop(cmdh);
        if (err == ESP_FAIL) {
        goto RELEASE_CMD_HANDLER;
    }


    err = i2c_master_cmd_begin(
        BALANC3R_I2C_MASTER_PORT, cmdh, 10 / portTICK_PERIOD_MS);


RELEASE_CMD_HANDLER:
    i2c_cmd_link_delete(cmdh);
    return err;
}

esp_err_t b_mpu6050_gyro_value(gyro_register_t reg, int16_t *out) {
    esp_err_t err;
    uint8_t data[2];

    err = b_mpu6050_read(BALANC3R_MPU6050_I2C_ADDRESS, reg, data, 2);
    *out =  (~((data[0] << 8) | data[1]) + 1) - (
        reg == GYRO_X_REGISTER ? gyro_x_offset :
        reg == GYRO_Y_REGISTER ? gyro_y_offset :
        reg == GYRO_Z_REGISTER ? gyro_z_offset : 0
    );

    return err;
}

#define __CALIBRATE_ITERATION(X, x) {\
    err = b_mpu6050_gyro_value(GYRO_##X##_REGISTER, &out);\
    if (err == ESP_FAIL) {\
        _gyro_x_offset = 0;\
        _gyro_y_offset = 0;\
        _gyro_z_offset = 0;\
        return err;\
    }\
    _gyro_##x##_offset += out;\
}

esp_err_t b_mpu6050_calibrate(uint16_t iterations) {
    uint16_t i;
    int16_t out,
        _gyro_x_offset = 0,
        _gyro_y_offset = 0,
        _gyro_z_offset = 0;
    esp_err_t err;

    gyro_x_offset = 0;
    gyro_y_offset = 0;
    gyro_z_offset = 0;

    for (i = 0; i < iterations; i++) {
        __CALIBRATE_ITERATION(X, x);
        __CALIBRATE_ITERATION(Y, y);
        __CALIBRATE_ITERATION(Z, z);
    }

    gyro_x_offset = _gyro_x_offset / iterations;
    gyro_y_offset = _gyro_y_offset / iterations;
    gyro_z_offset = _gyro_z_offset / iterations;

    return ESP_OK;
}