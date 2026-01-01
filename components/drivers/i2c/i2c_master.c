/**
 * @file i2c_master.c
 * @brief I2C Master Driver Implementation for ESP32-S3 Wristband
 * @author larythks
 * @date 2025-12-30
 */

#include "i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "I2C";

/* Mutex for thread-safe I2C bus access */
static SemaphoreHandle_t i2c_mutex = NULL;
static bool i2c_initialized = false;

/**
 * @brief Initialize I2C master bus
 */
esp_err_t i2c_master_init(void)
{
    if (i2c_initialized) {
        ESP_LOGW(TAG, "I2C master already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing I2C master...");
    ESP_LOGI(TAG, "  SDA: GPIO%d, SCL: GPIO%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    ESP_LOGI(TAG, "  Frequency: %d Hz", I2C_MASTER_FREQ_HZ);

    /* Configure I2C parameters */
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    /* Configure I2C driver */
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Install I2C driver */
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Create mutex for thread-safe access */
    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        i2c_driver_delete(I2C_MASTER_NUM);
        return ESP_FAIL;
    }

    i2c_initialized = true;
    ESP_LOGI(TAG, "I2C master initialized successfully");

    return ESP_OK;
}

/**
 * @brief Deinitialize I2C master bus
 */
esp_err_t i2c_master_deinit(void)
{
    if (!i2c_initialized) {
        return ESP_OK;
    }

    if (i2c_mutex != NULL) {
        vSemaphoreDelete(i2c_mutex);
        i2c_mutex = NULL;
    }

    esp_err_t ret = i2c_driver_delete(I2C_MASTER_NUM);
    if (ret == ESP_OK) {
        i2c_initialized = false;
        ESP_LOGI(TAG, "I2C master deinitialized");
    }

    return ret;
}

/**
 * @brief Check if a device exists at given I2C address
 */
bool i2c_device_exists(uint8_t device_addr)
{
    if (!i2c_initialized) {
        return false;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK);
}

/**
 * @brief Scan I2C bus for connected devices
 */
esp_err_t i2c_scan_devices(uint8_t *device_count)
{
    if (!i2c_initialized) {
        ESP_LOGE(TAG, "I2C not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Scanning I2C bus...");
    ESP_LOGI(TAG, "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");

    uint8_t count = 0;
    for (uint8_t addr = 0; addr < 0x80; addr++) {
        if (addr % 16 == 0) {
            printf("%02x: ", addr);
        }

        if (i2c_device_exists(addr)) {
            printf("%02x ", addr);
            count++;
        } else {
            printf("-- ");
        }

        if ((addr + 1) % 16 == 0) {
            printf("\n");
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to avoid bus congestion
    }

    ESP_LOGI(TAG, "Found %d device(s) on I2C bus", count);

    if (device_count != NULL) {
        *device_count = count;
    }

    return ESP_OK;
}

/**
 * @brief Write a single byte to I2C device register
 */
esp_err_t i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data)
{
    return i2c_write_bytes(device_addr, reg_addr, &data, 1);
}

/**
 * @brief Write multiple bytes to I2C device
 */
esp_err_t i2c_write_bytes(uint8_t device_addr, uint8_t reg_addr, const uint8_t *data, size_t len)
{
    if (!i2c_initialized) {
        ESP_LOGE(TAG, "I2C not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Take mutex to ensure thread-safe access */
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take I2C mutex");
        return ESP_ERR_TIMEOUT;
    }

    /* Build I2C command */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    /* Execute I2C transaction */
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    /* Release mutex */
    xSemaphoreGive(i2c_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed (addr=0x%02X, reg=0x%02X): %s",
                 device_addr, reg_addr, esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief Read a single byte from I2C device register
 */
esp_err_t i2c_read_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t *data)
{
    return i2c_read_bytes(device_addr, reg_addr, data, 1);
}

/**
 * @brief Read multiple bytes from I2C device
 */
esp_err_t i2c_read_bytes(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    if (!i2c_initialized) {
        ESP_LOGE(TAG, "I2C not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Take mutex to ensure thread-safe access */
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take I2C mutex");
        return ESP_ERR_TIMEOUT;
    }

    /* Build I2C command */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    /* Write register address */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    /* Read data */
    i2c_master_start(cmd); // Repeated start
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    /* Execute I2C transaction */
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    /* Release mutex */
    xSemaphoreGive(i2c_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed (addr=0x%02X, reg=0x%02X): %s",
                 device_addr, reg_addr, esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief Write data to I2C device without register address
 */
esp_err_t i2c_write_data(uint8_t device_addr, const uint8_t *data, size_t len)
{
    if (!i2c_initialized) {
        ESP_LOGE(TAG, "I2C not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Take mutex */
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take I2C mutex");
        return ESP_ERR_TIMEOUT;
    }

    /* Build I2C command */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    /* Execute I2C transaction */
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    /* Release mutex */
    xSemaphoreGive(i2c_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write_data failed (addr=0x%02X): %s",
                 device_addr, esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief Read data from I2C device without register address
 */
esp_err_t i2c_read_data(uint8_t device_addr, uint8_t *data, size_t len)
{
    if (!i2c_initialized) {
        ESP_LOGE(TAG, "I2C not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Take mutex */
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take I2C mutex");
        return ESP_ERR_TIMEOUT;
    }

    /* Build I2C command */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    /* Execute I2C transaction */
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    /* Release mutex */
    xSemaphoreGive(i2c_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read_data failed (addr=0x%02X): %s",
                 device_addr, esp_err_to_name(ret));
    }

    return ret;
}
