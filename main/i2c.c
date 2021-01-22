#include "i2c.h"
#include "driver/i2c.h"
#include "esp_err.h"

#include "config.h"

esp_err_t b_i2c_master_init() {
    esp_err_t err;

    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BALANC3R_I2C_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = BALANC3R_I2C_CLK_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BALANC3R_I2C_CLK_SPEED
    };

    err = i2c_param_config(BALANC3R_I2C_MASTER_PORT, &cfg);
    if (err == ESP_FAIL) {
        return err;
    }

    err = i2c_driver_install(
        BALANC3R_I2C_MASTER_PORT, I2C_MODE_MASTER, 0, 0, 0);

    return err; 
}