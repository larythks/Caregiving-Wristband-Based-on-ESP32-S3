/**
 * @file onewire.h
 * @brief 1-Wire Protocol Driver for ESP32-S3
 *
 * This driver implements the Dallas 1-Wire protocol for communication
 * with 1-Wire devices like DS18B20 temperature sensors.
 *
 * @author larythks
 * @date 2026-01-02
 */

#ifndef __ONEWIRE_H__
#define __ONEWIRE_H__

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 1-Wire timing constants (in microseconds)
 * Based on Dallas 1-Wire specification
 * Adjusted for ESP32-S3 GPIO timing characteristics
 */
#define ONEWIRE_DELAY_RESET_LOW     480    // Reset pulse duration (min 480us)
#define ONEWIRE_DELAY_RESET_WAIT    70     // Wait before reading presence pulse
#define ONEWIRE_DELAY_RESET_READ    410    // Total reset cycle time
#define ONEWIRE_DELAY_WRITE_LOW     1      // Write 1 low duration (min 1us, keep short)
#define ONEWIRE_DELAY_WRITE_SLOT    60     // Write slot duration (min 60us)
#define ONEWIRE_DELAY_READ_LOW      1      // Read slot initiation (min 1us, keep very short)
#define ONEWIRE_DELAY_READ_WAIT     14     // Wait before sampling (must be < 15us, increased for weak pull-up)
#define ONEWIRE_DELAY_READ_SLOT     50     // Read slot recovery (total slot ~65us)

/**
 * @brief 1-Wire ROM commands
 */
#define ONEWIRE_CMD_SEARCH_ROM      0xF0
#define ONEWIRE_CMD_READ_ROM        0x33
#define ONEWIRE_CMD_MATCH_ROM       0x55
#define ONEWIRE_CMD_SKIP_ROM        0xCC
#define ONEWIRE_CMD_ALARM_SEARCH    0xEC

/**
 * @brief 1-Wire device handle structure
 */
typedef struct {
    gpio_num_t pin;                 // GPIO pin number for 1-Wire bus
    bool initialized;               // Initialization status
} onewire_bus_t;

/**
 * @brief Initialize 1-Wire bus
 *
 * @param bus Pointer to 1-Wire bus handle
 * @param pin GPIO pin number for 1-Wire data line
 * @return esp_err_t ESP_OK on success
 */
esp_err_t onewire_init(onewire_bus_t *bus, gpio_num_t pin);

/**
 * @brief Send reset pulse and check for presence pulse
 *
 * @param bus Pointer to 1-Wire bus handle
 * @return true if device presence detected, false otherwise
 */
bool onewire_reset(onewire_bus_t *bus);

/**
 * @brief Write a single bit on 1-Wire bus
 *
 * @param bus Pointer to 1-Wire bus handle
 * @param bit Bit value (0 or 1)
 */
void onewire_write_bit(onewire_bus_t *bus, uint8_t bit);

/**
 * @brief Read a single bit from 1-Wire bus
 *
 * @param bus Pointer to 1-Wire bus handle
 * @return uint8_t Bit value (0 or 1)
 */
uint8_t onewire_read_bit(onewire_bus_t *bus);

/**
 * @brief Write a byte on 1-Wire bus
 *
 * @param bus Pointer to 1-Wire bus handle
 * @param data Byte to write
 */
void onewire_write_byte(onewire_bus_t *bus, uint8_t data);

/**
 * @brief Read a byte from 1-Wire bus
 *
 * @param bus Pointer to 1-Wire bus handle
 * @return uint8_t Byte read from bus
 */
uint8_t onewire_read_byte(onewire_bus_t *bus);

/**
 * @brief Write multiple bytes on 1-Wire bus
 *
 * @param bus Pointer to 1-Wire bus handle
 * @param data Pointer to data buffer
 * @param len Number of bytes to write
 */
void onewire_write_bytes(onewire_bus_t *bus, const uint8_t *data, size_t len);

/**
 * @brief Read multiple bytes from 1-Wire bus
 *
 * @param bus Pointer to 1-Wire bus handle
 * @param data Pointer to buffer for received data
 * @param len Number of bytes to read
 */
void onewire_read_bytes(onewire_bus_t *bus, uint8_t *data, size_t len);

/**
 * @brief Calculate CRC8 checksum (Dallas/Maxim polynomial)
 *
 * Used for verifying data integrity in 1-Wire communication.
 * Polynomial: x^8 + x^5 + x^4 + 1
 *
 * @param data Pointer to data buffer
 * @param len Length of data
 * @return uint8_t CRC8 checksum
 */
uint8_t onewire_crc8(const uint8_t *data, size_t len);

/**
 * @brief Send Skip ROM command (0xCC)
 *
 * Allows addressing all devices on the bus simultaneously.
 * Use when there is only one device on the bus.
 *
 * @param bus Pointer to 1-Wire bus handle
 */
void onewire_skip_rom(onewire_bus_t *bus);

/**
 * @brief Send Match ROM command (0x55)
 *
 * Allows addressing a specific device by its 64-bit ROM code.
 *
 * @param bus Pointer to 1-Wire bus handle
 * @param rom Pointer to 8-byte ROM code
 */
void onewire_match_rom(onewire_bus_t *bus, const uint8_t *rom);

/**
 * @brief Read ROM code from device
 *
 * Can only be used when there is a single device on the bus.
 *
 * @param bus Pointer to 1-Wire bus handle
 * @param rom Pointer to 8-byte buffer for ROM code
 * @return true if ROM read successfully, false otherwise
 */
bool onewire_read_rom(onewire_bus_t *bus, uint8_t *rom);

/**
 * @brief Search for devices on 1-Wire bus
 *
 * This function implements the 1-Wire search algorithm to find
 * all devices connected to the bus.
 *
 * @param bus Pointer to 1-Wire bus handle
 * @param rom_codes Pointer to array for storing ROM codes (8 bytes each)
 * @param max_devices Maximum number of devices to search for
 * @return int Number of devices found
 */
int onewire_search_devices(onewire_bus_t *bus, uint8_t (*rom_codes)[8], int max_devices);

#ifdef __cplusplus
}
#endif

#endif // __ONEWIRE_H__
