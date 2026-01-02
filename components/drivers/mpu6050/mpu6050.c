/**
 * @file mpu6050.c
 * @brief MPU6050 6-axis motion sensor driver implementation
 *
 * This file implements the MPU6050 driver functions for accelerometer,
 * gyroscope, and attitude calculation.
 *
 * @author larythks
 * @date 2026-01-02
 */

#include "mpu6050.h"
#include "driver/i2c_master.h"
#include "i2c_master.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include <math.h>

static const char *TAG = "MPU6050";

/**
 * @brief Current configuration
 */
static mpu6050_config_t g_mpu_config = {
    .accel_fs = MPU6050_ACCEL_FS_2G,
    .gyro_fs = MPU6050_GYRO_FS_250,
    .dlpf = MPU6050_DLPF_94HZ,
    .clock_src = MPU6050_CLOCK_PLL_ZGYRO,
    .sample_rate_div = 9  // 100Hz sample rate
};

/**
 * @brief Previous attitude angles for complementary filter
 */
static mpu6050_attitude_t g_prev_attitude = {0};
static bool g_filter_initialized = false;

/**
 * @brief Accelerometer sensitivity lookup table (LSB/g)
 */
static const float accel_sensitivity[] = {
    [MPU6050_ACCEL_FS_2G]  = 16384.0f,
    [MPU6050_ACCEL_FS_4G]  = 8192.0f,
    [MPU6050_ACCEL_FS_8G]  = 4096.0f,
    [MPU6050_ACCEL_FS_16G] = 2048.0f
};

/**
 * @brief Gyroscope sensitivity lookup table (LSB/(°/s))
 */
static const float gyro_sensitivity[] = {
    [MPU6050_GYRO_FS_250]  = 131.0f,
    [MPU6050_GYRO_FS_500]  = 65.5f,
    [MPU6050_GYRO_FS_1000] = 32.8f,
    [MPU6050_GYRO_FS_2000] = 16.4f
};

/**
 * @brief Write a byte to MPU6050 register
 *
 * @param reg Register address
 * @param data Data byte to write
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    return i2c_write_byte(MPU6050_I2C_ADDR, reg, data);
}

/**
 * @brief Read a byte from MPU6050 register
 *
 * @param reg Register address
 * @param data Pointer to store read data
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t mpu6050_read_reg(uint8_t reg, uint8_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return i2c_read_byte(MPU6050_I2C_ADDR, reg, data);
}

/**
 * @brief Read multiple bytes from MPU6050 registers
 *
 * @param reg Starting register address
 * @param data Pointer to buffer to store read data
 * @param len Number of bytes to read
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t mpu6050_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    return i2c_read_bytes(MPU6050_I2C_ADDR, reg, data, len);
}

/**
 * @brief Modify bits in a register
 *
 * @param reg Register address
 * @param mask Bit mask
 * @param value New value (will be masked)
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t mpu6050_modify_reg(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t old_val;
    esp_err_t ret = mpu6050_read_reg(reg, &old_val);
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t new_val = (old_val & ~mask) | (value & mask);
    if (new_val != old_val) {
        return mpu6050_write_reg(reg, new_val);
    }

    return ESP_OK;
}

bool mpu6050_test_connection(void)
{
    uint8_t who_am_i = 0;
    esp_err_t ret = mpu6050_read_reg(MPU6050_REG_WHO_AM_I, &who_am_i);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return false;
    }

    if (who_am_i == MPU6050_WHO_AM_I_VAL) {
        ESP_LOGI(TAG, "MPU6050 detected (WHO_AM_I = 0x%02X)", who_am_i);
        return true;
    } else {
        ESP_LOGW(TAG, "Unexpected WHO_AM_I value: 0x%02X (expected 0x%02X)",
                 who_am_i, MPU6050_WHO_AM_I_VAL);
        return false;
    }
}

esp_err_t mpu6050_reset(void)
{
    ESP_LOGI(TAG, "Resetting MPU6050...");

    // Set DEVICE_RESET bit in PWR_MGMT_1
    esp_err_t ret = mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x80);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset MPU6050");
        return ret;
    }

    // Wait for reset to complete
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}

esp_err_t mpu6050_wake_up(void)
{
    ESP_LOGD(TAG, "Waking up MPU6050...");

    // Clear SLEEP bit in PWR_MGMT_1
    return mpu6050_modify_reg(MPU6050_REG_PWR_MGMT_1, 0x40, 0x00);
}

esp_err_t mpu6050_sleep(void)
{
    ESP_LOGD(TAG, "Putting MPU6050 to sleep...");

    // Set SLEEP bit in PWR_MGMT_1
    return mpu6050_modify_reg(MPU6050_REG_PWR_MGMT_1, 0x40, 0x40);
}

esp_err_t mpu6050_set_accel_fs(mpu6050_accel_fs_t fs)
{
    ESP_LOGD(TAG, "Setting accelerometer full scale to %d", fs);

    esp_err_t ret = mpu6050_modify_reg(MPU6050_REG_ACCEL_CONFIG, 0x18, fs << 3);
    if (ret == ESP_OK) {
        g_mpu_config.accel_fs = fs;
    }
    return ret;
}

esp_err_t mpu6050_set_gyro_fs(mpu6050_gyro_fs_t fs)
{
    ESP_LOGD(TAG, "Setting gyroscope full scale to %d", fs);

    esp_err_t ret = mpu6050_modify_reg(MPU6050_REG_GYRO_CONFIG, 0x18, fs << 3);
    if (ret == ESP_OK) {
        g_mpu_config.gyro_fs = fs;
    }
    return ret;
}

esp_err_t mpu6050_set_dlpf(mpu6050_dlpf_t dlpf)
{
    ESP_LOGD(TAG, "Setting DLPF to %d", dlpf);

    esp_err_t ret = mpu6050_modify_reg(MPU6050_REG_CONFIG, 0x07, dlpf);
    if (ret == ESP_OK) {
        g_mpu_config.dlpf = dlpf;
    }
    return ret;
}

esp_err_t mpu6050_set_sample_rate_div(uint8_t div)
{
    ESP_LOGD(TAG, "Setting sample rate divider to %d", div);

    esp_err_t ret = mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV, div);
    if (ret == ESP_OK) {
        g_mpu_config.sample_rate_div = div;
    }
    return ret;
}

esp_err_t mpu6050_init_with_config(const mpu6050_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Initializing MPU6050 with custom config...");

    // Copy configuration
    memcpy(&g_mpu_config, config, sizeof(mpu6050_config_t));

    // Test connection
    if (!mpu6050_test_connection()) {
        ESP_LOGE(TAG, "MPU6050 not found on I2C bus");
        return ESP_ERR_NOT_FOUND;
    }

    // Reset device
    esp_err_t ret = mpu6050_reset();
    if (ret != ESP_OK) {
        return ret;
    }

    // Wake up from sleep
    ret = mpu6050_wake_up();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        return ret;
    }

    // Set clock source
    ret = mpu6050_modify_reg(MPU6050_REG_PWR_MGMT_1, 0x07, config->clock_src);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set clock source");
        return ret;
    }

    // Set gyroscope full scale range
    ret = mpu6050_set_gyro_fs(config->gyro_fs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gyro full scale");
        return ret;
    }

    // Set accelerometer full scale range
    ret = mpu6050_set_accel_fs(config->accel_fs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set accel full scale");
        return ret;
    }

    // Set DLPF bandwidth
    ret = mpu6050_set_dlpf(config->dlpf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set DLPF");
        return ret;
    }

    // Set sample rate divider
    ret = mpu6050_set_sample_rate_div(config->sample_rate_div);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set sample rate divider");
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    ESP_LOGI(TAG, "  Accel FS: ±%dg", 2 << config->accel_fs);
    ESP_LOGI(TAG, "  Gyro FS: ±%d°/s", 250 << config->gyro_fs);
    ESP_LOGI(TAG, "  Sample Rate: %d Hz", 1000 / (1 + config->sample_rate_div));

    return ESP_OK;
}

esp_err_t mpu6050_init(void)
{
    ESP_LOGI(TAG, "Initializing MPU6050 with default config...");
    return mpu6050_init_with_config(&g_mpu_config);
}

esp_err_t mpu6050_read_raw_data(mpu6050_raw_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read all sensor data in one burst (14 bytes)
    // Register 0x3B to 0x48: ACCEL_XOUT_H to GYRO_ZOUT_L
    uint8_t buffer[14];
    esp_err_t ret = mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H, buffer, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return ret;
    }

    // Parse accelerometer data (big-endian)
    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);

    // Parse temperature data
    data->temperature = (int16_t)((buffer[6] << 8) | buffer[7]);

    // Parse gyroscope data (big-endian)
    data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);

    return ESP_OK;
}

esp_err_t mpu6050_read_accel(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z)
{
    if (accel_x == NULL || accel_y == NULL || accel_z == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[6];
    esp_err_t ret = mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H, buffer, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer data");
        return ret;
    }

    *accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    *accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    *accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);

    return ESP_OK;
}

esp_err_t mpu6050_read_gyro(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
{
    if (gyro_x == NULL || gyro_y == NULL || gyro_z == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[6];
    esp_err_t ret = mpu6050_read_regs(MPU6050_REG_GYRO_XOUT_H, buffer, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read gyroscope data");
        return ret;
    }

    *gyro_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    *gyro_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    *gyro_z = (int16_t)((buffer[4] << 8) | buffer[5]);

    return ESP_OK;
}

esp_err_t mpu6050_read_temperature(int16_t *temp)
{
    if (temp == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[2];
    esp_err_t ret = mpu6050_read_regs(MPU6050_REG_TEMP_OUT_H, buffer, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature data");
        return ret;
    }

    *temp = (int16_t)((buffer[0] << 8) | buffer[1]);

    return ESP_OK;
}

esp_err_t mpu6050_read_scaled_data(mpu6050_scaled_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read raw data
    mpu6050_raw_data_t raw_data;
    esp_err_t ret = mpu6050_read_raw_data(&raw_data);
    if (ret != ESP_OK) {
        return ret;
    }

    // Get current sensitivities
    float accel_sens = mpu6050_get_accel_sensitivity();
    float gyro_sens = mpu6050_get_gyro_sensitivity();

    // Convert to physical units
    data->accel_x = (float)raw_data.accel_x / accel_sens;
    data->accel_y = (float)raw_data.accel_y / accel_sens;
    data->accel_z = (float)raw_data.accel_z / accel_sens;

    data->gyro_x = (float)raw_data.gyro_x / gyro_sens;
    data->gyro_y = (float)raw_data.gyro_y / gyro_sens;
    data->gyro_z = (float)raw_data.gyro_z / gyro_sens;

    // Temperature in degrees C = (TEMP_OUT Register Value / 340) + 36.53
    data->temperature = ((float)raw_data.temperature / 340.0f) + 36.53f;

    return ESP_OK;
}

esp_err_t mpu6050_get_attitude(mpu6050_attitude_t *attitude)
{
    if (attitude == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read accelerometer data
    int16_t ax, ay, az;
    esp_err_t ret = mpu6050_read_accel(&ax, &ay, &az);
    if (ret != ESP_OK) {
        return ret;
    }

    // Convert to g
    float accel_sens = mpu6050_get_accel_sensitivity();
    float ax_g = (float)ax / accel_sens;
    float ay_g = (float)ay / accel_sens;
    float az_g = (float)az / accel_sens;

    // Calculate roll and pitch angles using accelerometer
    // Roll (rotation around X-axis): atan2(ay, az)
    // Pitch (rotation around Y-axis): atan2(-ax, sqrt(ay^2 + az^2))
    attitude->roll = atan2f(ay_g, az_g) * 180.0f / M_PI;
    attitude->pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 180.0f / M_PI;
    attitude->yaw = 0.0f;  // Cannot determine yaw from accelerometer alone

    return ESP_OK;
}

esp_err_t mpu6050_get_attitude_complementary(mpu6050_attitude_t *attitude, float dt, float alpha)
{
    if (attitude == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (dt <= 0.0f || alpha < 0.0f || alpha > 1.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read sensor data
    mpu6050_scaled_data_t data;
    esp_err_t ret = mpu6050_read_scaled_data(&data);
    if (ret != ESP_OK) {
        return ret;
    }

    // Calculate angles from accelerometer
    float accel_roll = atan2f(data.accel_y, data.accel_z) * 180.0f / M_PI;
    float accel_pitch = atan2f(-data.accel_x, sqrtf(data.accel_y * data.accel_y +
                                                     data.accel_z * data.accel_z)) * 180.0f / M_PI;

    if (!g_filter_initialized) {
        // First iteration: use accelerometer values
        attitude->roll = accel_roll;
        attitude->pitch = accel_pitch;
        attitude->yaw = 0.0f;
        g_prev_attitude = *attitude;
        g_filter_initialized = true;
    } else {
        // Complementary filter:
        // angle = alpha * (prev_angle + gyro * dt) + (1 - alpha) * accel_angle
        attitude->roll = alpha * (g_prev_attitude.roll + data.gyro_x * dt) +
                        (1.0f - alpha) * accel_roll;
        attitude->pitch = alpha * (g_prev_attitude.pitch + data.gyro_y * dt) +
                         (1.0f - alpha) * accel_pitch;
        attitude->yaw = g_prev_attitude.yaw + data.gyro_z * dt;  // Integrate yaw

        // Update previous values
        g_prev_attitude = *attitude;
    }

    return ESP_OK;
}

float mpu6050_get_accel_sensitivity(void)
{
    return accel_sensitivity[g_mpu_config.accel_fs];
}

float mpu6050_get_gyro_sensitivity(void)
{
    return gyro_sensitivity[g_mpu_config.gyro_fs];
}
