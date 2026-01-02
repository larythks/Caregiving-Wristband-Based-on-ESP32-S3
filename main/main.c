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
#include "mpu6050.h"

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
     * 0x3C - OLED Display (SH1106)
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

    ESP_LOGI(TAG, "Configuring 1-Wire bus on GPIO%d...", DS18B20_GPIO);
    ret = onewire_init(&onewire_bus, DS18B20_GPIO);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "1-Wire bus initialized successfully");

        // Test GPIO and pull-up resistor
        ESP_LOGI(TAG, "Testing GPIO configuration...");
        ESP_LOGI(TAG, "GPIO idle state (should be HIGH): %d", gpio_get_level(DS18B20_GPIO));

        // Pull low and check
        gpio_set_level(DS18B20_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
        int low_level = gpio_get_level(DS18B20_GPIO);
        ESP_LOGI(TAG, "GPIO when pulled LOW: %d", low_level);

        // Release and check if pull-up brings it high
        gpio_set_level(DS18B20_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
        int high_level = gpio_get_level(DS18B20_GPIO);
        ESP_LOGI(TAG, "GPIO when released (should be HIGH): %d", high_level);

        if (high_level == 0) {
            ESP_LOGE(TAG, "============================================");
            ESP_LOGE(TAG, "CRITICAL: Pull-up resistor NOT working!");
            ESP_LOGE(TAG, "GPIO stays LOW when released.");
            ESP_LOGE(TAG, "");
            ESP_LOGE(TAG, "Possible causes:");
            ESP_LOGE(TAG, "  1. NO 4.7K pull-up resistor installed");
            ESP_LOGE(TAG, "  2. Pull-up resistor not connected properly");
            ESP_LOGE(TAG, "  3. DQ line shorted to GND");
            ESP_LOGE(TAG, "");
            ESP_LOGE(TAG, "Fix: Install 4.7K resistor between GPIO%d and 3.3V", DS18B20_GPIO);
            ESP_LOGE(TAG, "============================================");
        } else {
            ESP_LOGI(TAG, "Pull-up resistor working correctly");
        }

        // Test for device presence
        ESP_LOGI(TAG, "Testing for DS18B20 presence...");
        if (onewire_reset(&onewire_bus)) {
            ESP_LOGI(TAG, "Device presence detected!");

            // Initialize DS18B20 (skip ROM for single device)
            ret = ds18b20_init(&ds18b20_device, &onewire_bus, NULL);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "DS18B20 initialized successfully");
                ds18b20_available = true;

                // Read ROM code for diagnostics
                uint8_t rom[8];
                if (onewire_read_rom(&onewire_bus, rom)) {
                    ESP_LOGI(TAG, "DS18B20 ROM Code: %02X %02X %02X %02X %02X %02X %02X %02X",
                             rom[0], rom[1], rom[2], rom[3], rom[4], rom[5], rom[6], rom[7]);
                    ESP_LOGI(TAG, "  Family Code: 0x%02X (should be 0x28 for DS18B20)", rom[0]);
                    ESP_LOGI(TAG, "  Serial Number: %02X%02X%02X%02X%02X%02X",
                             rom[6], rom[5], rom[4], rom[3], rom[2], rom[1]);
                    ESP_LOGI(TAG, "  CRC: 0x%02X", rom[7]);
                } else {
                    ESP_LOGW(TAG, "Failed to read ROM code");
                }

                // Set resolution to 12-bit for maximum accuracy
                ret = ds18b20_set_resolution(&ds18b20_device, DS18B20_RESOLUTION_12BIT);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Resolution set to 12-bit (0.0625°C precision)");
                } else {
                    ESP_LOGW(TAG, "Failed to set resolution: %s", esp_err_to_name(ret));
                }

                // Check power mode
                bool parasitic = false;
                if (ds18b20_is_parasitic_power(&ds18b20_device, &parasitic) == ESP_OK) {
                    ESP_LOGI(TAG, "Power mode: %s", parasitic ? "Parasitic" : "External");

                    if (parasitic) {
                        ESP_LOGW(TAG, "============================================");
                        ESP_LOGW(TAG, "WARNING: DS18B20 detected in PARASITIC mode!");
                        ESP_LOGW(TAG, "");
                        ESP_LOGW(TAG, "For development/debugging, you should:");
                        ESP_LOGW(TAG, "  1. Connect DS18B20 VDD pin to 3.3V");
                        ESP_LOGW(TAG, "  2. Keep GND connected to GND");
                        ESP_LOGW(TAG, "  3. Keep DQ connected to GPIO%d with 4.7K pull-up", DS18B20_GPIO);
                        ESP_LOGW(TAG, "");
                        ESP_LOGW(TAG, "If VDD is already connected to 3.3V:");
                        ESP_LOGW(TAG, "  - This may be a clone/fake DS18B20 with buggy power detection");
                        ESP_LOGW(TAG, "  - The sensor should still work for temperature readings");
                        ESP_LOGW(TAG, "  - Try reading temperature to verify functionality");
                        ESP_LOGW(TAG, "============================================");
                    } else {
                        ESP_LOGI(TAG, "External power mode - optimal for debugging");
                    }
                }

                // Perform a test temperature reading to verify sensor is working
                ESP_LOGI(TAG, "Performing initial temperature test...");
                float test_temp = 0.0f;
                ret = ds18b20_read_temperature(&ds18b20_device, &test_temp);
                if (ret == ESP_OK && test_temp != 0.0f) {
                    ESP_LOGI(TAG, "Initial temperature reading: %.2f°C - Sensor is working!", test_temp);
                } else {
                    ESP_LOGW(TAG, "Initial temperature test failed or returned 0°C");
                }
            } else {
                ESP_LOGE(TAG, "DS18B20 initialization failed: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGW(TAG, "No device detected on 1-Wire bus!");
            ESP_LOGW(TAG, "Please check:");
            ESP_LOGW(TAG, "  1. DS18B20 is connected to GPIO%d", DS18B20_GPIO);
            ESP_LOGW(TAG, "  2. 4.7k pull-up resistor is installed");
            ESP_LOGW(TAG, "  3. Power supply is connected");
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialize 1-Wire bus: %s", esp_err_to_name(ret));
    }

    /* Initialize MPU6050 Motion Sensor (Day 5-6) */
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Initializing MPU6050 Sensor (Day 5-6)");
    ESP_LOGI(TAG, "========================================");

    bool mpu6050_available = false;
    mpu6050_scaled_data_t mpu_data = {0};
    mpu6050_attitude_t attitude = {0};

    if (i2c_device_exists(MPU6050_I2C_ADDR)) {
        ret = mpu6050_init();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "MPU6050 initialized successfully");
            mpu6050_available = true;

            // Perform initial reading
            ESP_LOGI(TAG, "Performing initial sensor test...");
            ret = mpu6050_read_scaled_data(&mpu_data);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Accelerometer (g): X=%.3f, Y=%.3f, Z=%.3f",
                         mpu_data.accel_x, mpu_data.accel_y, mpu_data.accel_z);
                ESP_LOGI(TAG, "Gyroscope (°/s): X=%.3f, Y=%.3f, Z=%.3f",
                         mpu_data.gyro_x, mpu_data.gyro_y, mpu_data.gyro_z);
                ESP_LOGI(TAG, "MPU Temperature: %.2f°C", mpu_data.temperature);

                // Calculate initial attitude
                ret = mpu6050_get_attitude(&attitude);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Initial Attitude: Roll=%.1f°, Pitch=%.1f°",
                             attitude.roll, attitude.pitch);
                }
            } else {
                ESP_LOGW(TAG, "Failed to read sensor data: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGE(TAG, "Failed to initialize MPU6050: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGW(TAG, "MPU6050 not detected at address 0x68, skipping initialization");
    }

    /* Main loop */
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Entering main loop...");
    ESP_LOGI(TAG, "========================================");
    uint32_t loop_count = 0;
    while (1) {
        loop_count++;

        // Read temperature from DS18B20
        if (ds18b20_available) {
            float temperature = 0.0f;
            ret = ds18b20_read_temperature(&ds18b20_device, &temperature);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "DS18B20 Temperature: %.2f°C", temperature);

                // Update OLED with temperature display
                if (i2c_device_exists(OLED_I2C_ADDRESS)) {
                    char temp_str[32];
                    snprintf(temp_str, sizeof(temp_str), "Temp:%.2fC", temperature);
                    oled_show_string(0, 2, temp_str);
                }
            } else {
                ESP_LOGE(TAG, "Failed to read DS18B20: %s", esp_err_to_name(ret));

                // Display error on OLED
                if (i2c_device_exists(OLED_I2C_ADDRESS)) {
                    oled_show_string(0, 2, "Temp: Error!");
                }
            }
        }

        // Read motion data from MPU6050
        if (mpu6050_available) {
            ret = mpu6050_read_scaled_data(&mpu_data);
            if (ret == ESP_OK) {
                // Log accelerometer data
                ESP_LOGI(TAG, "Accel(g): X=%.2f Y=%.2f Z=%.2f",
                         mpu_data.accel_x, mpu_data.accel_y, mpu_data.accel_z);

                // Log gyroscope data (log less frequently to reduce spam)
                if (loop_count % 2 == 0) {
                    ESP_LOGI(TAG, "Gyro(°/s): X=%.1f Y=%.1f Z=%.1f",
                             mpu_data.gyro_x, mpu_data.gyro_y, mpu_data.gyro_z);
                }

                // Calculate attitude angles
                ret = mpu6050_get_attitude(&attitude);
                if (ret == ESP_OK && loop_count % 2 == 0) {
                    ESP_LOGI(TAG, "Attitude: Roll=%.1f° Pitch=%.1f°",
                             attitude.roll, attitude.pitch);
                }

                // Update OLED with motion data
                if (i2c_device_exists(OLED_I2C_ADDRESS)) {
                    char accel_str[32];
                    snprintf(accel_str, sizeof(accel_str), "Ax:%.2fg", mpu_data.accel_x);
                    oled_show_string(0, 3, accel_str);

                    snprintf(accel_str, sizeof(accel_str), "Ay:%.2fg", mpu_data.accel_y);
                    oled_show_string(0, 4, accel_str);

                    snprintf(accel_str, sizeof(accel_str), "Az:%.2fg", mpu_data.accel_z);
                    oled_show_string(0, 5, accel_str);

                    // Display attitude on OLED
                    char attitude_str[32];
                    snprintf(attitude_str, sizeof(attitude_str), "R:%.0f P:%.0f",
                             attitude.roll, attitude.pitch);
                    oled_show_string(0, 6, attitude_str);
                }
            } else {
                ESP_LOGE(TAG, "Failed to read MPU6050: %s", esp_err_to_name(ret));

                // Display error on OLED
                if (i2c_device_exists(OLED_I2C_ADDRESS)) {
                    oled_show_string(0, 3, "MPU: Error!");
                }
            }
        }

        // Update OLED with real-time counter
        if (i2c_device_exists(OLED_I2C_ADDRESS)) {
            char counter_str[32];
            snprintf(counter_str, sizeof(counter_str), "Count: %lu", loop_count);
            oled_show_string(0, 7, counter_str);
            oled_refresh();
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Update every 1 seconds to reduce log spam
    }
}
