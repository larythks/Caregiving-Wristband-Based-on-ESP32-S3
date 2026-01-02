/**
 * @file mpu6050.h
 * @brief MPU6050 6-axis motion sensor driver for ESP32-S3
 *
 * This driver provides functions to interface with the MPU6050 sensor
 * which combines a 3-axis accelerometer and a 3-axis gyroscope.
 *
 * @author larythks
 * @date 2026-01-02
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MPU6050 I2C address (AD0 pin connected to GND)
 */
#define MPU6050_I2C_ADDR            0x68

/**
 * @brief MPU6050 register addresses
 */
#define MPU6050_REG_SELF_TEST_X     0x0D
#define MPU6050_REG_SELF_TEST_Y     0x0E
#define MPU6050_REG_SELF_TEST_Z     0x0F
#define MPU6050_REG_SELF_TEST_A     0x10
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_FIFO_EN         0x23
#define MPU6050_REG_INT_PIN_CFG     0x37
#define MPU6050_REG_INT_ENABLE      0x38
#define MPU6050_REG_INT_STATUS      0x3A
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_ACCEL_XOUT_L    0x3C
#define MPU6050_REG_ACCEL_YOUT_H    0x3D
#define MPU6050_REG_ACCEL_YOUT_L    0x3E
#define MPU6050_REG_ACCEL_ZOUT_H    0x3F
#define MPU6050_REG_ACCEL_ZOUT_L    0x40
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_TEMP_OUT_L      0x42
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_GYRO_XOUT_L     0x44
#define MPU6050_REG_GYRO_YOUT_H     0x45
#define MPU6050_REG_GYRO_YOUT_L     0x46
#define MPU6050_REG_GYRO_ZOUT_H     0x47
#define MPU6050_REG_GYRO_ZOUT_L     0x48
#define MPU6050_REG_USER_CTRL       0x6A
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_PWR_MGMT_2      0x6C
#define MPU6050_REG_FIFO_COUNTH     0x72
#define MPU6050_REG_FIFO_COUNTL     0x73
#define MPU6050_REG_FIFO_R_W        0x74
#define MPU6050_REG_WHO_AM_I        0x75

/**
 * @brief MPU6050 WHO_AM_I register value
 */
#define MPU6050_WHO_AM_I_VAL        0x68

/**
 * @brief Accelerometer full scale range
 */
typedef enum {
    MPU6050_ACCEL_FS_2G  = 0,  ///< ±2g
    MPU6050_ACCEL_FS_4G  = 1,  ///< ±4g
    MPU6050_ACCEL_FS_8G  = 2,  ///< ±8g
    MPU6050_ACCEL_FS_16G = 3   ///< ±16g
} mpu6050_accel_fs_t;

/**
 * @brief Gyroscope full scale range
 */
typedef enum {
    MPU6050_GYRO_FS_250  = 0,  ///< ±250°/s
    MPU6050_GYRO_FS_500  = 1,  ///< ±500°/s
    MPU6050_GYRO_FS_1000 = 2,  ///< ±1000°/s
    MPU6050_GYRO_FS_2000 = 3   ///< ±2000°/s
} mpu6050_gyro_fs_t;

/**
 * @brief Digital Low Pass Filter (DLPF) bandwidth
 */
typedef enum {
    MPU6050_DLPF_260HZ = 0,  ///< Bandwidth 260Hz
    MPU6050_DLPF_184HZ = 1,  ///< Bandwidth 184Hz
    MPU6050_DLPF_94HZ  = 2,  ///< Bandwidth 94Hz
    MPU6050_DLPF_44HZ  = 3,  ///< Bandwidth 44Hz
    MPU6050_DLPF_21HZ  = 4,  ///< Bandwidth 21Hz
    MPU6050_DLPF_10HZ  = 5,  ///< Bandwidth 10Hz
    MPU6050_DLPF_5HZ   = 6   ///< Bandwidth 5Hz
} mpu6050_dlpf_t;

/**
 * @brief Clock source selection
 */
typedef enum {
    MPU6050_CLOCK_INTERNAL   = 0,  ///< Internal 8MHz oscillator
    MPU6050_CLOCK_PLL_XGYRO  = 1,  ///< PLL with X axis gyroscope reference
    MPU6050_CLOCK_PLL_YGYRO  = 2,  ///< PLL with Y axis gyroscope reference
    MPU6050_CLOCK_PLL_ZGYRO  = 3,  ///< PLL with Z axis gyroscope reference
    MPU6050_CLOCK_PLL_EXT32K = 4,  ///< PLL with external 32.768kHz reference
    MPU6050_CLOCK_PLL_EXT19M = 5,  ///< PLL with external 19.2MHz reference
    MPU6050_CLOCK_KEEP_RESET = 7   ///< Stops the clock and keeps timing generator in reset
} mpu6050_clock_src_t;

/**
 * @brief Raw sensor data structure
 */
typedef struct {
    int16_t accel_x;    ///< Raw accelerometer X-axis data
    int16_t accel_y;    ///< Raw accelerometer Y-axis data
    int16_t accel_z;    ///< Raw accelerometer Z-axis data
    int16_t gyro_x;     ///< Raw gyroscope X-axis data
    int16_t gyro_y;     ///< Raw gyroscope Y-axis data
    int16_t gyro_z;     ///< Raw gyroscope Z-axis data
    int16_t temperature; ///< Raw temperature data
} mpu6050_raw_data_t;

/**
 * @brief Scaled sensor data structure (physical units)
 */
typedef struct {
    float accel_x;      ///< Accelerometer X-axis (g)
    float accel_y;      ///< Accelerometer Y-axis (g)
    float accel_z;      ///< Accelerometer Z-axis (g)
    float gyro_x;       ///< Gyroscope X-axis (°/s)
    float gyro_y;       ///< Gyroscope Y-axis (°/s)
    float gyro_z;       ///< Gyroscope Z-axis (°/s)
    float temperature;  ///< Temperature (°C)
} mpu6050_scaled_data_t;

/**
 * @brief Attitude angles structure (Euler angles)
 */
typedef struct {
    float roll;         ///< Roll angle (°)
    float pitch;        ///< Pitch angle (°)
    float yaw;          ///< Yaw angle (°) - requires magnetometer for absolute yaw
} mpu6050_attitude_t;

/**
 * @brief MPU6050 configuration structure
 */
typedef struct {
    mpu6050_accel_fs_t accel_fs;    ///< Accelerometer full scale range
    mpu6050_gyro_fs_t gyro_fs;      ///< Gyroscope full scale range
    mpu6050_dlpf_t dlpf;            ///< Digital low pass filter bandwidth
    mpu6050_clock_src_t clock_src;  ///< Clock source
    uint8_t sample_rate_div;        ///< Sample rate divider (sample_rate = 1kHz / (1 + div))
} mpu6050_config_t;

/**
 * @brief Initialize MPU6050 with default configuration
 *
 * Default configuration:
 * - Accelerometer: ±2g
 * - Gyroscope: ±250°/s
 * - DLPF: 94Hz bandwidth
 * - Clock: PLL with Z-axis gyroscope reference
 * - Sample rate: 100Hz (div = 9)
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if MPU6050 not detected
 * @return ESP_FAIL on communication error
 */
esp_err_t mpu6050_init(void);

/**
 * @brief Initialize MPU6050 with custom configuration
 *
 * @param config Pointer to configuration structure
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if config is NULL
 * @return ESP_ERR_NOT_FOUND if MPU6050 not detected
 * @return ESP_FAIL on communication error
 */
esp_err_t mpu6050_init_with_config(const mpu6050_config_t *config);

/**
 * @brief Test MPU6050 connection by reading WHO_AM_I register
 *
 * @return true if MPU6050 is detected
 * @return false if MPU6050 is not detected
 */
bool mpu6050_test_connection(void);

/**
 * @brief Wake up MPU6050 from sleep mode
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on communication error
 */
esp_err_t mpu6050_wake_up(void);

/**
 * @brief Put MPU6050 into sleep mode to save power
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on communication error
 */
esp_err_t mpu6050_sleep(void);

/**
 * @brief Reset MPU6050 (all registers reset to default values)
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on communication error
 */
esp_err_t mpu6050_reset(void);

/**
 * @brief Set accelerometer full scale range
 *
 * @param fs Full scale range
 * @return ESP_OK on success
 * @return ESP_FAIL on communication error
 */
esp_err_t mpu6050_set_accel_fs(mpu6050_accel_fs_t fs);

/**
 * @brief Set gyroscope full scale range
 *
 * @param fs Full scale range
 * @return ESP_OK on success
 * @return ESP_FAIL on communication error
 */
esp_err_t mpu6050_set_gyro_fs(mpu6050_gyro_fs_t fs);

/**
 * @brief Set digital low pass filter bandwidth
 *
 * @param dlpf DLPF bandwidth
 * @return ESP_OK on success
 * @return ESP_FAIL on communication error
 */
esp_err_t mpu6050_set_dlpf(mpu6050_dlpf_t dlpf);

/**
 * @brief Set sample rate divider
 *
 * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
 * where Gyroscope Output Rate = 1kHz (when DLPF is enabled)
 *
 * @param div Sample rate divider (0-255)
 * @return ESP_OK on success
 * @return ESP_FAIL on communication error
 */
esp_err_t mpu6050_set_sample_rate_div(uint8_t div);

/**
 * @brief Read raw sensor data (all 6 axes + temperature)
 *
 * @param data Pointer to raw data structure
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if data is NULL
 * @return ESP_FAIL on communication error
 */
esp_err_t mpu6050_read_raw_data(mpu6050_raw_data_t *data);

/**
 * @brief Read raw accelerometer data only
 *
 * @param accel_x Pointer to X-axis data
 * @param accel_y Pointer to Y-axis data
 * @param accel_z Pointer to Z-axis data
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if any pointer is NULL
 * @return ESP_FAIL on communication error
 */
esp_err_t mpu6050_read_accel(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z);

/**
 * @brief Read raw gyroscope data only
 *
 * @param gyro_x Pointer to X-axis data
 * @param gyro_y Pointer to Y-axis data
 * @param gyro_z Pointer to Z-axis data
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if any pointer is NULL
 * @return ESP_FAIL on communication error
 */
esp_err_t mpu6050_read_gyro(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

/**
 * @brief Read raw temperature data
 *
 * @param temp Pointer to temperature data
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if temp is NULL
 * @return ESP_FAIL on communication error
 */
esp_err_t mpu6050_read_temperature(int16_t *temp);

/**
 * @brief Read and convert sensor data to physical units
 *
 * @param data Pointer to scaled data structure
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if data is NULL
 * @return ESP_FAIL on communication error
 */
esp_err_t mpu6050_read_scaled_data(mpu6050_scaled_data_t *data);

/**
 * @brief Calculate attitude angles from accelerometer data
 *
 * Uses accelerometer data to calculate roll and pitch angles.
 * Yaw angle is set to 0 (requires magnetometer for absolute yaw).
 *
 * @param attitude Pointer to attitude structure
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if attitude is NULL
 * @return ESP_FAIL on communication error
 */
esp_err_t mpu6050_get_attitude(mpu6050_attitude_t *attitude);

/**
 * @brief Calculate attitude angles using complementary filter
 *
 * Combines accelerometer and gyroscope data using a complementary filter.
 * Provides more stable attitude estimation than accelerometer-only.
 *
 * @param attitude Pointer to attitude structure
 * @param dt Time interval since last update (seconds)
 * @param alpha Filter coefficient (0.0-1.0), typical value: 0.98
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if attitude is NULL or parameters invalid
 * @return ESP_FAIL on communication error
 */
esp_err_t mpu6050_get_attitude_complementary(mpu6050_attitude_t *attitude, float dt, float alpha);

/**
 * @brief Get current accelerometer sensitivity (LSB/g)
 *
 * @return Sensitivity value based on current full scale range
 */
float mpu6050_get_accel_sensitivity(void);

/**
 * @brief Get current gyroscope sensitivity (LSB/(°/s))
 *
 * @return Sensitivity value based on current full scale range
 */
float mpu6050_get_gyro_sensitivity(void);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_H
