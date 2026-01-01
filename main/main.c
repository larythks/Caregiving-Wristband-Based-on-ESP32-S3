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
#include "onewire.h"
#include "ds18b20.h"

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

    /* Initialize DS18B20 Temperature Sensor (Day 4) */
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Initializing DS18B20 Sensor (Day 4)");
    ESP_LOGI(TAG, "========================================");

    onewire_bus_t onewire_bus;
    ds18b20_device_t ds18b20_device;
    bool ds18b20_available = false;

    ret = onewire_init(&onewire_bus, DS18B20_GPIO);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "1-Wire bus initialized on GPIO%d", DS18B20_GPIO);

        // Initialize DS18B20 (skip ROM for single device)
        ret = ds18b20_init(&ds18b20_device, &onewire_bus, NULL);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "DS18B20 temperature sensor initialized successfully");
            ds18b20_available = true;

            // Set resolution to 12-bit for maximum accuracy
            ret = ds18b20_set_resolution(&ds18b20_device, DS18B20_RESOLUTION_12BIT);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "DS18B20 resolution set to 12-bit (0.0625°C)");
            }
        } else {
            ESP_LOGW(TAG, "DS18B20 not detected on GPIO%d", DS18B20_GPIO);
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialize 1-Wire bus");
    }

    /* Main loop */
    ESP_LOGI(TAG, "Entering main loop...");
    uint32_t loop_count = 0;
    while (1) {
        ESP_LOGI(TAG, "Loop #%lu - Free heap: %lu bytes", ++loop_count, esp_get_free_heap_size());

        // Read temperature from DS18B20
        if (ds18b20_available) {
            float temperature = 0.0f;
            ret = ds18b20_read_temperature(&ds18b20_device, &temperature);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Temperature: %.2f°C", temperature);

                // Update OLED with temperature display
                if (i2c_device_exists(OLED_I2C_ADDRESS)) {
                    char temp_str[32];
                    snprintf(temp_str, sizeof(temp_str), "Temp: %.2fC", temperature);
                    oled_show_string(0, 4, temp_str);
                }
            } else {
                ESP_LOGW(TAG, "Failed to read temperature");
            }
        }

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
