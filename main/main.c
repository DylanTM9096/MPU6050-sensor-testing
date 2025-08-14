#include <stdio.h>
#include "mpu6050.h" // Custom driver for MPU6050 IMU (accelerometer + gyroscope)
#include "kalman.h" // Kalman filter module for sensor fusion

// MPU6050 Sensor Configuration
#define accel_scale 1 // Accelerometer sensitivity setting
#define gyro_scale 1 // Gyroscope sensitivity setting

static TimerHandle_t mpu6050Timer; // Timer for polling MPU at desired rate
static QueueHandle_t rawImuQueue; // Queue for passing raw IMU data from mpu6050 to kalman

int IMU_timer_period_ms = 10;  // IMU polling rate in milliseconds
float initial_tilt_angle = 89.5f; // Initial tilt offset due to physical mounting

void app_main(void){
    rawImuQueue = xQueueCreate(1, sizeof(mpu6050_data_t)); // Create queue to store 1 set of IMU data
    // Configures MPU6050 with accel/gyro scales and queue to write to
    ESP_ERROR_CHECK(mpu6050_config(accel_scale, gyro_scale, rawImuQueue));
    // Sets up Kalman filter with tilt angle and input queue
    ESP_ERROR_CHECK(kalman_config(rawImuQueue, initial_tilt_angle));

    // Start IMU read timer
    mpu6050Timer = xTimerCreate("MPU6050", pdMS_TO_TICKS(IMU_timer_period_ms), pdTRUE, NULL, IMU_timer);
    if (mpu6050Timer == NULL || xTimerStart(mpu6050Timer, 0) != pdPASS) {
        ESP_LOGE(TAG_MPU6050, "Failed to create/start MPU6050 timer");
    }
    // Task Creation for Kalman filter
    if(xTaskCreate(KalmanTask, "Kalman Update", 4096, NULL, 3, NULL) != pdPASS){
        ESP_LOGE(TAG_Kalman, "Failed to create/start Kalman Update");
    }
}