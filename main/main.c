#include "i2c.h"
#include "mpu6050.h"
#include "config.h"

#include "driver/i2c.h"
#include "esp_err.h"


void app_main(void) {
   int16_t outX, outY, outZ;
   uint8_t poweron = 0;
   
   ESP_ERROR_CHECK(b_i2c_master_init());
   ESP_ERROR_CHECK(b_mpu6050_write(
      BALANC3R_MPU6050_I2C_ADDRESS, 0x6B, &poweron, 1));

   ESP_ERROR_CHECK(b_mpu6050_calibrate(10));

   while(1) {
      ESP_ERROR_CHECK(b_mpu6050_gyro_value(GYRO_X_REGISTER, &outX));
      ESP_ERROR_CHECK(b_mpu6050_gyro_value(GYRO_Y_REGISTER, &outY));
      ESP_ERROR_CHECK(b_mpu6050_gyro_value(GYRO_Z_REGISTER, &outZ));
      
      printf("X: %f\tY: %f\tZ: %f\n",
         outX / 131.0, outY / 131.0, outZ / 131.0);

      vTaskDelay(100 / portTICK_PERIOD_MS);
   }
}