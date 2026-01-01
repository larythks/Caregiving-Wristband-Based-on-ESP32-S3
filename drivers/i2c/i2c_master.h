/**
 * @file i2c_master.h
 * @brief I2C Master Driver for ESP32-S3 Wristband
 * @author Claude AI Assistant
 * @date 2025-12-30
 *
 * This driver provides I2C master functionality for communicating with:
 * - OLED Display (0x3C)
 * - MAX30102 Heart Rate Sensor (0x57)
 * - MPU6050 Motion Sensor (0x68)
 */

#ifndef I2C_MASTER_H
#define I2C_MASTER_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* I2C Configuration */
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_SDA_IO       8           // GPIO8
#define I2C_MASTER_SCL_IO       9           // GPIO9
#define I2C_MASTER_FREQ_HZ      400000      // 400kHz
#define I2C_MASTER_TIMEOUT_MS   1000

#define I2C_MASTER_TX_BUF_DISABLE   0       // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0

/* ACK values */
#define I2C_MASTER_ACK              0
#define I2C_MASTER_NACK             1

/**
 * @brief Initialize I2C master bus
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_master_init(void);

/**
 * @brief Deinitialize I2C master bus
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_master_deinit(void);

/**
 * @brief Scan I2C bus for connected devices
 *
 * Scans the entire I2C address range (0x00-0x7F) and prints found devices
 *
 * @param[out] device_count Number of devices found (can be NULL)
 * @return ESP_OK on success
 */
esp_err_t i2c_scan_devices(uint8_t *device_count);

/**
 * @brief Check if a device exists at given I2C address
 *
 * @param device_addr 7-bit I2C device address
 * @return true if device responds, false otherwise
 */
bool i2c_device_exists(uint8_t device_addr);

/**
 * @brief Write a single byte to I2C device register
 *
 * @param device_addr 7-bit I2C device address
 * @param reg_addr Register address
 * @param data Data byte to write
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data);

/**
 * @brief Write multiple bytes to I2C device
 *
 * @param device_addr 7-bit I2C device address
 * @param reg_addr Starting register address
 * @param data Pointer to data buffer
 * @param len Number of bytes to write
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_write_bytes(uint8_t device_addr, uint8_t reg_addr, const uint8_t *data, size_t len);

/**
 * @brief Read a single byte from I2C device register
 *
 * @param device_addr 7-bit I2C device address
 * @param reg_addr Register address to read from
 * @param data Pointer to store read byte
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_read_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t *data);

/**
 * @brief Read multiple bytes from I2C device
 *
 * @param device_addr 7-bit I2C device address
 * @param reg_addr Starting register address
 * @param data Pointer to buffer to store read data
 * @param len Number of bytes to read
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_read_bytes(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Write data to I2C device without register address
 *
 * Useful for devices that don't use register-based addressing (like some OLEDs)
 *
 * @param device_addr 7-bit I2C device address
 * @param data Pointer to data buffer
 * @param len Number of bytes to write
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_write_data(uint8_t device_addr, const uint8_t *data, size_t len);

/**
 * @brief Read data from I2C device without register address
 *
 * @param device_addr 7-bit I2C device address
 * @param data Pointer to buffer to store read data
 * @param len Number of bytes to read
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_read_data(uint8_t device_addr, uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // I2C_MASTER_H
