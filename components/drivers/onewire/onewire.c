/**
 * @file onewire.c
 * @brief 1-Wire Protocol Driver Implementation
 *
 * @author larythks
 * @date 2026-01-02
 */

#include "onewire.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "ONEWIRE";

/**
 * @brief Set GPIO output level to low
 * In Open-Drain mode, writing 0 pulls the line low
 */
static inline void onewire_set_low(onewire_bus_t *bus)
{
    gpio_set_level(bus->pin, 0);
}

/**
 * @brief Set GPIO output level to high (release the line)
 * In Open-Drain mode, writing 1 releases the line (pulled high by external resistor)
 */
static inline void onewire_set_high(onewire_bus_t *bus)
{
    gpio_set_level(bus->pin, 1);
}

/**
 * @brief Read GPIO input level
 * In Open-Drain mode, we can read the line state at any time
 */
static inline int onewire_read_level(onewire_bus_t *bus)
{
    return gpio_get_level(bus->pin);
}

/**
 * @brief Microsecond delay
 */
static inline void onewire_delay_us(uint32_t us)
{
    ets_delay_us(us);
}

/**
 * @brief Initialize 1-Wire bus
 */
esp_err_t onewire_init(onewire_bus_t *bus, gpio_num_t pin)
{
    if (bus == NULL) {
        ESP_LOGE(TAG, "Bus handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    bus->pin = pin;

    // Configure GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT_OD,  // Open-drain mode
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO configuration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set bus to idle state (high)
    gpio_set_level(pin, 1);

    bus->initialized = true;
    ESP_LOGI(TAG, "1-Wire bus initialized on GPIO%d", pin);

    return ESP_OK;
}

/**
 * @brief Send reset pulse and check for presence pulse
 */
bool onewire_reset(onewire_bus_t *bus)
{
    if (!bus->initialized) {
        ESP_LOGE(TAG, "Bus not initialized");
        return false;
    }

    int presence;

    // Disable interrupts for precise timing
    portDISABLE_INTERRUPTS();

    // Send reset pulse (pull bus low for 480us)
    onewire_set_low(bus);
    onewire_delay_us(ONEWIRE_DELAY_RESET_LOW);

    // Release bus and wait for presence pulse
    onewire_set_high(bus);
    onewire_delay_us(ONEWIRE_DELAY_RESET_WAIT);

    // Read presence pulse (device pulls bus low)
    presence = !onewire_read_level(bus);

    // Wait for presence pulse to complete
    onewire_delay_us(ONEWIRE_DELAY_RESET_READ);

    portENABLE_INTERRUPTS();

    return presence;
}

/**
 * @brief Write a single bit on 1-Wire bus
 */
void onewire_write_bit(onewire_bus_t *bus, uint8_t bit)
{
    portDISABLE_INTERRUPTS();

    if (bit & 1) {
        // Write 1: short low pulse (< 15us), then release for rest of time slot
        onewire_set_low(bus);
        onewire_delay_us(ONEWIRE_DELAY_WRITE_LOW);  // 1us
        onewire_set_high(bus);
        onewire_delay_us(ONEWIRE_DELAY_WRITE_SLOT); // 60us recovery
    } else {
        // Write 0: long low pulse (> 60us)
        onewire_set_low(bus);
        onewire_delay_us(ONEWIRE_DELAY_WRITE_SLOT); // 60us
        onewire_set_high(bus);
        onewire_delay_us(ONEWIRE_DELAY_WRITE_LOW);  // 1us recovery
    }

    portENABLE_INTERRUPTS();
}

/**
 * @brief Read a single bit from 1-Wire bus
 */
uint8_t onewire_read_bit(onewire_bus_t *bus)
{
    uint8_t bit;

    portDISABLE_INTERRUPTS();

    // Initiate read slot by pulling bus low
    onewire_set_low(bus);
    onewire_delay_us(ONEWIRE_DELAY_READ_LOW);

    // Release bus and wait for device to pull down (if sending 0)
    onewire_set_high(bus);
    onewire_delay_us(ONEWIRE_DELAY_READ_WAIT);

    // Read bit value - sample the bus state
    bit = onewire_read_level(bus);

    // Complete read slot (wait for full time slot)
    onewire_delay_us(ONEWIRE_DELAY_READ_SLOT);

    portENABLE_INTERRUPTS();

    return bit ? 1 : 0;  // Ensure we return 0 or 1
}

/**
 * @brief Write a byte on 1-Wire bus
 */
void onewire_write_byte(onewire_bus_t *bus, uint8_t data)
{
    for (int i = 0; i < 8; i++) {
        onewire_write_bit(bus, data & 0x01);
        data >>= 1;
    }
}

/**
 * @brief Read a byte from 1-Wire bus
 */
uint8_t onewire_read_byte(onewire_bus_t *bus)
{
    uint8_t data = 0;

    for (int i = 0; i < 8; i++) {
        data >>= 1;
        if (onewire_read_bit(bus)) {
            data |= 0x80;
        }
    }

    return data;
}

/**
 * @brief Write multiple bytes on 1-Wire bus
 */
void onewire_write_bytes(onewire_bus_t *bus, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        onewire_write_byte(bus, data[i]);
    }
}

/**
 * @brief Read multiple bytes from 1-Wire bus
 */
void onewire_read_bytes(onewire_bus_t *bus, uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        data[i] = onewire_read_byte(bus);
    }
}

/**
 * @brief Calculate CRC8 checksum (Dallas/Maxim polynomial)
 */
uint8_t onewire_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;

    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];

        for (int j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ byte) & 0x01;
            crc >>= 1;
            if (mix) {
                crc ^= 0x8C;  // Polynomial: x^8 + x^5 + x^4 + 1
            }
            byte >>= 1;
        }
    }

    return crc;
}

/**
 * @brief Send Skip ROM command (0xCC)
 */
void onewire_skip_rom(onewire_bus_t *bus)
{
    onewire_write_byte(bus, ONEWIRE_CMD_SKIP_ROM);
}

/**
 * @brief Send Match ROM command (0x55)
 */
void onewire_match_rom(onewire_bus_t *bus, const uint8_t *rom)
{
    onewire_write_byte(bus, ONEWIRE_CMD_MATCH_ROM);
    onewire_write_bytes(bus, rom, 8);
}

/**
 * @brief Read ROM code from device
 */
bool onewire_read_rom(onewire_bus_t *bus, uint8_t *rom)
{
    if (!onewire_reset(bus)) {
        ESP_LOGE(TAG, "No device detected on bus");
        return false;
    }

    onewire_write_byte(bus, ONEWIRE_CMD_READ_ROM);
    onewire_read_bytes(bus, rom, 8);

    // Verify CRC
    if (onewire_crc8(rom, 7) != rom[7]) {
        ESP_LOGE(TAG, "ROM CRC mismatch");
        return false;
    }

    return true;
}

/**
 * @brief Search for devices on 1-Wire bus
 *
 * This implements the 1-Wire search algorithm to find all devices.
 * Note: This is a simplified implementation for single device scenarios.
 */
int onewire_search_devices(onewire_bus_t *bus, uint8_t (*rom_codes)[8], int max_devices)
{
    int device_count = 0;
    uint8_t last_discrepancy = 0;
    bool last_device_flag = false;
    uint8_t rom[8] = {0};

    while (!last_device_flag && device_count < max_devices) {
        if (!onewire_reset(bus)) {
            ESP_LOGW(TAG, "No devices found on bus");
            break;
        }

        onewire_write_byte(bus, ONEWIRE_CMD_SEARCH_ROM);

        uint8_t id_bit_number = 1;
        uint8_t last_zero = 0;
        uint8_t rom_byte_number = 0;
        uint8_t rom_byte_mask = 1;
        bool search_direction;

        while (rom_byte_number < 8) {
            // Read bit and complement bit
            uint8_t id_bit = onewire_read_bit(bus);
            uint8_t cmp_id_bit = onewire_read_bit(bus);

            if (id_bit && cmp_id_bit) {
                // No devices responded
                break;
            } else if (id_bit != cmp_id_bit) {
                // All devices have same bit value
                search_direction = id_bit;
            } else {
                // Discrepancy: devices have different bits
                if (id_bit_number < last_discrepancy) {
                    search_direction = (rom[rom_byte_number] & rom_byte_mask) > 0;
                } else {
                    search_direction = (id_bit_number == last_discrepancy);
                }

                if (!search_direction) {
                    last_zero = id_bit_number;
                }
            }

            if (search_direction) {
                rom[rom_byte_number] |= rom_byte_mask;
            } else {
                rom[rom_byte_number] &= ~rom_byte_mask;
            }

            onewire_write_bit(bus, search_direction);

            id_bit_number++;
            rom_byte_mask <<= 1;

            if (rom_byte_mask == 0) {
                rom_byte_number++;
                rom_byte_mask = 1;
            }
        }

        if (id_bit_number >= 65) {
            // Verify CRC
            if (onewire_crc8(rom, 7) == rom[7]) {
                memcpy(rom_codes[device_count], rom, 8);
                device_count++;

                last_discrepancy = last_zero;

                if (last_discrepancy == 0) {
                    last_device_flag = true;
                }
            }
        }
    }

    ESP_LOGI(TAG, "Found %d device(s) on bus", device_count);
    return device_count;
}
