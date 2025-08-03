#include <stdio.h>
#include "mpu6050.h"
#include "kalman.h"

#define accel_scale 1
#define gyro_scale 1

#define JOYSTICK_INPUT_GPIO GPIO_NUM_18
#define DC_MOTOR_PWM_GPIO GPIO_NUM_19
#define SERVO_MOTOR_PWM_GPIO GPIO_NUM_5
#define DC_MOTOR_FREQ_HZ 2000
#define SERVO_FREQ_HZ 50 // DS3240 servo expects a PWM period of 20ms aka 50hz

static TimerHandle_t mpu6050Timer; // Timer for polling MPU at desired rate
static QueueHandle_t rawImuQueue; // Queue for passing raw IMU data from mpu6050 to kalman

int IMU_timer_period_ms = 10;
float initial_tilt_angle = 89.5f;


void app_main(void){
    rawImuQueue = xQueueCreate(1, sizeof(mpu6050_data_t)); // Create queue to store up to 5 sets of IMU data
    
    ESP_ERROR_CHECK(mpu6050_config(accel_scale, gyro_scale, rawImuQueue));
    ESP_ERROR_CHECK(kalman_config(rawImuQueue, initial_tilt_angle));

    // Start IMU read timer
    mpu6050Timer = xTimerCreate("MPU6050", pdMS_TO_TICKS(IMU_timer_period_ms), pdTRUE, NULL, IMU_timer);
    if (mpu6050Timer == NULL || xTimerStart(mpu6050Timer, 0) != pdPASS) {
        ESP_LOGE(TAG_MPU6050, "Failed to create/start MPU6050 timer");
    }

    if(xTaskCreate(KalmanTask, "Kalman Update", 4096, NULL, 3, NULL) != pdPASS){
        ESP_LOGE(TAG_Kalman, "Failed to create/start Kalman Update");
    }
}