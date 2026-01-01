/**
 * @file main.c
 * @brief ESP32-S3 Wristband Main Entry Point
 * @author larythks
 * @date 2025-12-30
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "app_config.h"
#include "i2c_master.h"
#include "oled.h"

static const char *TAG = TAG_MAIN;

/**
 * @brief Main application entry point
 */
void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ESP32-S3 Smart Wristband Starting...");
    ESP_LOGI(TAG, "========================================");

    /* Initialize NVS (Non-Volatile Storage) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS flash...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized successfully");

    /* Print system information */
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());

    /* Initialize I2C master bus */
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Initializing Hardware Drivers (Day 1-2)");
    ESP_LOGI(TAG, "========================================");

    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C master!");
        return;
    }

    /* Scan I2C bus for connected devices */
    ESP_LOGI(TAG, "Scanning I2C bus for devices...");
    uint8_t device_count = 0;
    i2c_scan_devices(&device_count);

    /* Expected devices:
     * 0x3C - OLED Display (SSD1306)
     * 0x57 - MAX30102 Heart Rate Sensor
     * 0x68 - MPU6050 Motion Sensor
     */
    if (device_count > 0) {
        ESP_LOGI(TAG, "I2C device detection:");
        ESP_LOGI(TAG, "  OLED (0x3C):     %s", i2c_device_exists(0x3C) ? "FOUND" : "NOT FOUND");
        ESP_LOGI(TAG, "  MAX30102 (0x57): %s", i2c_device_exists(0x57) ? "FOUND" : "NOT FOUND");
        ESP_LOGI(TAG, "  MPU6050 (0x68):  %s", i2c_device_exists(0x68) ? "FOUND" : "NOT FOUND");
    } else {
        ESP_LOGW(TAG, "No I2C devices found! Check hardware connections.");
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Hardware initialization complete!");
    ESP_LOGI(TAG, "========================================");

    /* Initialize OLED Display (Day 3) */
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Initializing OLED Display (Day 3)");
    ESP_LOGI(TAG, "========================================");

    if (i2c_device_exists(OLED_I2C_ADDRESS)) {
        ret = oled_init();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "OLED display initialized successfully");

            // Display welcome message
            oled_clear();
            oled_show_string(0, 0, "ESP32-S3");
            oled_show_string(0, 1, "Wristband");
            oled_show_string(0, 3, "Hardware OK!");
            oled_show_string(0, 5, "I2C Devices:");

            char device_info[32];
            snprintf(device_info, sizeof(device_info), "Found: %d", device_count);
            oled_show_string(0, 6, device_info);

            oled_refresh();
            ESP_LOGI(TAG, "Welcome message displayed on OLED");
        } else {
            ESP_LOGE(TAG, "Failed to initialize OLED display!");
        }
    } else {
        ESP_LOGW(TAG, "OLED not detected, skipping initialization");
    }

    /* Main loop */
    ESP_LOGI(TAG, "Entering main loop...");
    uint32_t loop_count = 0;
    while (1) {
        ESP_LOGI(TAG, "Loop #%lu - Free heap: %lu bytes", ++loop_count, esp_get_free_heap_size());

        // Update OLED with real-time counter
        if (i2c_device_exists(OLED_I2C_ADDRESS)) {
            char counter_str[32];
            snprintf(counter_str, sizeof(counter_str), "Count: %lu", loop_count);
            oled_show_string(0, 7, counter_str);
            oled_refresh();
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Update every 1 second
    }
}
