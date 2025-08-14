#include "kalman.h"

#define RAD_TO_DEG 57.2957795f // Convert radians to degrees
#define Q_angle 0.003f // Process noise variance for the angle
#define Q_bias 0.003f // Process noise variance for the gyro bias
#define R_measure 0.001f // Measurement noise variance

static kalman_filter_t kf; // Kalman filter state structure
static bool KALMAN_FILTER = KALMAN_NOT_CONFIGURED; // Filter configuration state
static QueueHandle_t raw_imu_queue = NULL; // Queue holding raw IMU data

// Initialize Kalman filter with queues and initial angle value
esp_err_t kalman_config(QueueHandle_t raw_queue, float initial_angle){
    raw_imu_queue = raw_queue; // Assign raw IMU queue
    kf.angle = initial_angle; // Set initial angle estimate
    kf.bias = 0.0f; // Initialize gyro bias estimate
    kf.P[0][0] = 0.0f; // Initialize error covariance matrix elements to zero
    kf.P[0][1] = 0.0f;
    kf.P[1][0] = 0.0f;
    kf.P[1][1] = 0.0f;
    KALMAN_FILTER = KALMAN_CONFIGURED; // Set as configured
    return ESP_OK;
}

// Perform one Kalman filter update step using elapsed time dt in seconds
static void kalman_filter_update(float angle_measured, mpu6050_data_t imu_data_in, float dt) {
    // Predict
    float rate = imu_data_in.gyro_x - kf.bias; // Remove bias from gyro rate
    kf.angle += dt * rate; // Integrate gyro rate to update angle estimate

    // Update error covariance matrix with process noise
    kf.P[0][0] += dt * (dt*kf.P[1][1] - kf.P[0][1] - kf.P[1][0] + Q_angle);
    kf.P[0][1] -= dt * kf.P[1][1];
    kf.P[1][0] -= dt * kf.P[1][1];
    kf.P[1][1] += Q_bias * dt;

    float S = kf.P[0][0] + R_measure; // Innovation covariance
    float K0 = kf.P[0][0] / S; // Kalman gain for angle
    float K1 = kf.P[1][0] / S; // Kalman gain for bias

    float y = angle_measured - kf.angle; // Angle difference (innovation)
    kf.angle += K0 * y; // Correct angle estimate
    kf.bias  += K1 * y; // Correct bias estimate

    // Update error covariance matrix
    float P00_temp = kf.P[0][0];
    float P01_temp = kf.P[0][1];
    kf.P[0][0] -= K0 * P00_temp;
    kf.P[0][1] -= K0 * P01_temp;
    kf.P[1][0] -= K1 * P00_temp;
    kf.P[1][1] -= K1 * P01_temp;
    ESP_LOGI("Data", "Accel: %.2f %.2f %.2f | Gyro: %.2f %.2f %.2f | Accel_angle: %.2f | Kalman_filter_angle: %.2f |", imu_data_in.accel_x, imu_data_in.accel_y, imu_data_in.accel_z, imu_data_in.gyro_x, imu_data_in.gyro_y, imu_data_in.gyro_z, angle_measured, kf.angle);
}

void kalman_filter_step(float dt) {
    if (KALMAN_FILTER == KALMAN_NOT_CONFIGURED){
        ESP_LOGE(TAG_Kalman, "Not properly set up: %d / 1 configured", KALMAN_FILTER); // Return immedietly if the kalman filter is not fully configured
        vTaskDelete(NULL); // Deletes itself since it wasn't configured correctly
    }

    mpu6050_data_t imu_data_in; // Struct to hold raw IMU data
    if (xQueueReceive(raw_imu_queue, &imu_data_in, portMAX_DELAY) == pdPASS) { // Wait for IMU data
        float acc_angle = atan2f(imu_data_in.accel_z, imu_data_in.accel_y) * RAD_TO_DEG; // Calculate angle from accelerometer
        kalman_filter_update(acc_angle, imu_data_in, dt); //Calculate the new tilt angle
    }
}

void KalmanTask(void* pvParameters) {
    TickType_t previous_ticks = xTaskGetTickCount(); // Initialize previous tick count
    while(1){
        TickType_t current_ticks = xTaskGetTickCount(); // Get current tick since startup (1 tick is 10ms by default)
        float dt_seconds = (float)(current_ticks - previous_ticks) * portTICK_PERIOD_MS / 1000.0f; // Convert Ticks to change in time (should be the same as MPU timer but this makes sure)
        previous_ticks = current_ticks; // Update previous ticks value
        kalman_filter_step(dt_seconds); // Update tilt angle estimate
    }
}