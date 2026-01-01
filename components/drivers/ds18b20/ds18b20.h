/**
 * @file ds18b20.h
 * @brief DS18B20 Temperature Sensor Driver for ESP32-S3
 *
 * This driver provides functions to read temperature from DS18B20 sensors
 * using the 1-Wire protocol.
 *
 * @author larythks
 * @date 2026-01-02
 */

#ifndef __DS18B20_H__
#define __DS18B20_H__

#include <stdint.h>
#include <stdbool.h>
#include "onewire.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief DS18B20 function commands
 */
#define DS18B20_CMD_CONVERT_T           0x44    // Start temperature conversion
#define DS18B20_CMD_READ_SCRATCHPAD     0xBE    // Read scratchpad memory
#define DS18B20_CMD_WRITE_SCRATCHPAD    0x4E    // Write scratchpad memory
#define DS18B20_CMD_COPY_SCRATCHPAD     0x48    // Copy scratchpad to EEPROM
#define DS18B20_CMD_RECALL_E2           0xB8    // Recall EEPROM to scratchpad
#define DS18B20_CMD_READ_POWER_SUPPLY   0xB4    // Read power supply mode

/**
 * @brief DS18B20 resolution settings
 */
typedef enum {
    DS18B20_RESOLUTION_9BIT  = 0,   // 9-bit resolution (0.5°C, 93.75ms)
    DS18B20_RESOLUTION_10BIT = 1,   // 10-bit resolution (0.25°C, 187.5ms)
    DS18B20_RESOLUTION_11BIT = 2,   // 11-bit resolution (0.125°C, 375ms)
    DS18B20_RESOLUTION_12BIT = 3    // 12-bit resolution (0.0625°C, 750ms)
} ds18b20_resolution_t;

/**
 * @brief DS18B20 conversion time for different resolutions (in milliseconds)
 */
#define DS18B20_CONVERSION_TIME_9BIT    94
#define DS18B20_CONVERSION_TIME_10BIT   188
#define DS18B20_CONVERSION_TIME_11BIT   375
#define DS18B20_CONVERSION_TIME_12BIT   750

/**
 * @brief DS18B20 device handle structure
 */
typedef struct {
    onewire_bus_t *bus;             // Pointer to 1-Wire bus handle
    uint8_t rom[8];                 // ROM code of the device
    ds18b20_resolution_t resolution; // Temperature resolution
    bool use_rom;                   // True: use specific ROM, False: skip ROM (single device)
    bool initialized;               // Initialization status
} ds18b20_device_t;

/**
 * @brief Initialize DS18B20 sensor
 *
 * @param device Pointer to DS18B20 device handle
 * @param bus Pointer to initialized 1-Wire bus handle
 * @param rom Pointer to 8-byte ROM code (NULL to use Skip ROM for single device)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ds18b20_init(ds18b20_device_t *device, onewire_bus_t *bus, const uint8_t *rom);

/**
 * @brief Set temperature resolution
 *
 * @param device Pointer to DS18B20 device handle
 * @param resolution Resolution setting (9-12 bits)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ds18b20_set_resolution(ds18b20_device_t *device, ds18b20_resolution_t resolution);

/**
 * @brief Start temperature conversion
 *
 * @param device Pointer to DS18B20 device handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ds18b20_start_conversion(ds18b20_device_t *device);

/**
 * @brief Read temperature from sensor (blocking)
 *
 * This function starts conversion and waits for completion before reading.
 *
 * @param device Pointer to DS18B20 device handle
 * @param temperature Pointer to store temperature value in degrees Celsius
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ds18b20_read_temperature(ds18b20_device_t *device, float *temperature);

/**
 * @brief Read temperature from sensor (non-blocking)
 *
 * This function reads the last converted temperature without starting a new conversion.
 * You must call ds18b20_start_conversion() and wait for conversion time before calling this.
 *
 * @param device Pointer to DS18B20 device handle
 * @param temperature Pointer to store temperature value in degrees Celsius
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ds18b20_read_temperature_async(ds18b20_device_t *device, float *temperature);

/**
 * @brief Read raw scratchpad data
 *
 * @param device Pointer to DS18B20 device handle
 * @param scratchpad Pointer to 9-byte buffer for scratchpad data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ds18b20_read_scratchpad(ds18b20_device_t *device, uint8_t *scratchpad);

/**
 * @brief Check if DS18B20 is using parasitic power
 *
 * @param device Pointer to DS18B20 device handle
 * @param parasitic Pointer to store result (true if parasitic power)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ds18b20_is_parasitic_power(ds18b20_device_t *device, bool *parasitic);

/**
 * @brief Get conversion time for current resolution
 *
 * @param device Pointer to DS18B20 device handle
 * @return uint16_t Conversion time in milliseconds
 */
uint16_t ds18b20_get_conversion_time(ds18b20_device_t *device);

/**
 * @brief Search and initialize all DS18B20 devices on the bus
 *
 * @param bus Pointer to initialized 1-Wire bus handle
 * @param devices Pointer to array of DS18B20 device handles
 * @param max_devices Maximum number of devices to search for
 * @return int Number of devices found and initialized
 */
int ds18b20_search_devices(onewire_bus_t *bus, ds18b20_device_t *devices, int max_devices);

#ifdef __cplusplus
}
#endif

#endif // __DS18B20_H__
