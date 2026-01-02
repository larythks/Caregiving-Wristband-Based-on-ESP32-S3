/**
 * @file ds18b20.c
 * @brief DS18B20 Temperature Sensor Driver Implementation
 *
 * @author larythks
 * @date 2026-01-02
 */

#include "ds18b20.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include <string.h>

static const char *TAG = "DS18B20";

/**
 * @brief Get conversion time for specific resolution
 */
static uint16_t get_conversion_time_ms(ds18b20_resolution_t resolution)
{
    switch (resolution) {
        case DS18B20_RESOLUTION_9BIT:
            return DS18B20_CONVERSION_TIME_9BIT;
        case DS18B20_RESOLUTION_10BIT:
            return DS18B20_CONVERSION_TIME_10BIT;
        case DS18B20_RESOLUTION_11BIT:
            return DS18B20_CONVERSION_TIME_11BIT;
        case DS18B20_RESOLUTION_12BIT:
        default:
            return DS18B20_CONVERSION_TIME_12BIT;
    }
}

/**
 * @brief Initialize DS18B20 sensor
 */
esp_err_t ds18b20_init(ds18b20_device_t *device, onewire_bus_t *bus, const uint8_t *rom)
{
    if (device == NULL || bus == NULL) {
        ESP_LOGE(TAG, "Invalid device or bus handle");
        return ESP_ERR_INVALID_ARG;
    }

    if (!bus->initialized) {
        ESP_LOGE(TAG, "1-Wire bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    device->bus = bus;
    device->resolution = DS18B20_RESOLUTION_12BIT;  // Default to highest resolution
    device->initialized = false;

    if (rom != NULL) {
        // Use specific ROM code
        memcpy(device->rom, rom, 8);
        device->use_rom = true;
    } else {
        // Skip ROM (single device on bus)
        memset(device->rom, 0, 8);
        device->use_rom = false;
    }

    // Test device presence
    if (!onewire_reset(device->bus)) {
        ESP_LOGE(TAG, "No DS18B20 device found on bus");
        return ESP_ERR_NOT_FOUND;
    }

    device->initialized = true;

    // Set default resolution to 12-bit
    esp_err_t ret = ds18b20_set_resolution(device, DS18B20_RESOLUTION_12BIT);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set resolution, continuing anyway");
    }

    ESP_LOGI(TAG, "DS18B20 initialized successfully");
    return ESP_OK;
}

/**
 * @brief Set temperature resolution
 */
esp_err_t ds18b20_set_resolution(ds18b20_device_t *device, ds18b20_resolution_t resolution)
{
    if (!device->initialized) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (resolution > DS18B20_RESOLUTION_12BIT) {
        ESP_LOGE(TAG, "Invalid resolution");
        return ESP_ERR_INVALID_ARG;
    }

    // Reset and select device
    if (!onewire_reset(device->bus)) {
        ESP_LOGE(TAG, "Device not responding");
        return ESP_ERR_NOT_FOUND;
    }

    if (device->use_rom) {
        onewire_match_rom(device->bus, device->rom);
    } else {
        onewire_skip_rom(device->bus);
    }

    // Write scratchpad command
    onewire_write_byte(device->bus, DS18B20_CMD_WRITE_SCRATCHPAD);

    // Write TH, TL, and configuration registers
    onewire_write_byte(device->bus, 0x00);  // TH register (not used)
    onewire_write_byte(device->bus, 0x00);  // TL register (not used)
    onewire_write_byte(device->bus, (resolution << 5) | 0x1F);  // Configuration

    // Copy scratchpad to EEPROM
    if (!onewire_reset(device->bus)) {
        ESP_LOGE(TAG, "Device not responding after write");
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (device->use_rom) {
        onewire_match_rom(device->bus, device->rom);
    } else {
        onewire_skip_rom(device->bus);
    }

    onewire_write_byte(device->bus, DS18B20_CMD_COPY_SCRATCHPAD);

    // Wait for copy to complete (typical 10ms)
    vTaskDelay(pdMS_TO_TICKS(10));

    device->resolution = resolution;
    ESP_LOGI(TAG, "Resolution set to %d bits", 9 + resolution);

    return ESP_OK;
}

/**
 * @brief Start temperature conversion
 */
esp_err_t ds18b20_start_conversion(ds18b20_device_t *device)
{
    if (!device->initialized) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Reset and select device
    if (!onewire_reset(device->bus)) {
        ESP_LOGE(TAG, "Device not responding during conversion start");
        return ESP_ERR_NOT_FOUND;
    }

    if (device->use_rom) {
        onewire_match_rom(device->bus, device->rom);
    } else {
        onewire_skip_rom(device->bus);
    }

    // Start conversion
    onewire_write_byte(device->bus, DS18B20_CMD_CONVERT_T);

    ESP_LOGI(TAG, "Temperature conversion started (will take ~%d ms)",
             get_conversion_time_ms(device->resolution));

    return ESP_OK;
}

/**
 * @brief Read scratchpad data
 */
esp_err_t ds18b20_read_scratchpad(ds18b20_device_t *device, uint8_t *scratchpad)
{
    if (!device->initialized) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (scratchpad == NULL) {
        ESP_LOGE(TAG, "Scratchpad buffer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Reset and select device
    if (!onewire_reset(device->bus)) {
        ESP_LOGE(TAG, "Device not responding during scratchpad read");
        return ESP_ERR_NOT_FOUND;
    }

    if (device->use_rom) {
        onewire_match_rom(device->bus, device->rom);
    } else {
        onewire_skip_rom(device->bus);
    }

    // Read scratchpad
    onewire_write_byte(device->bus, DS18B20_CMD_READ_SCRATCHPAD);

    // Use microsecond delay instead of vTaskDelay to avoid task switching
    // which could cause timing issues
    ets_delay_us(50);

    onewire_read_bytes(device->bus, scratchpad, 9);

    // Log scratchpad data for debugging (use INFO level for visibility)
    ESP_LOGI(TAG, "Scratchpad: %02X %02X %02X %02X %02X %02X %02X %02X [CRC: %02X]",
             scratchpad[0], scratchpad[1], scratchpad[2], scratchpad[3],
             scratchpad[4], scratchpad[5], scratchpad[6], scratchpad[7],
             scratchpad[8]);

    // Verify CRC
    uint8_t calculated_crc = onewire_crc8(scratchpad, 8);
    if (calculated_crc != scratchpad[8]) {
        ESP_LOGE(TAG, "Scratchpad CRC mismatch! Expected: 0x%02X, Got: 0x%02X",
                 calculated_crc, scratchpad[8]);
        return ESP_ERR_INVALID_CRC;
    }

    return ESP_OK;
}

/**
 * @brief Read temperature from sensor (non-blocking)
 */
esp_err_t ds18b20_read_temperature_async(ds18b20_device_t *device, float *temperature)
{
    if (!device->initialized) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (temperature == NULL) {
        ESP_LOGE(TAG, "Temperature pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t scratchpad[9];
    esp_err_t ret = ds18b20_read_scratchpad(device, scratchpad);
    if (ret != ESP_OK) {
        return ret;
    }

    // Extract temperature from scratchpad (bytes 0 and 1)
    int16_t raw_temp = (scratchpad[1] << 8) | scratchpad[0];

    ESP_LOGI(TAG, "Raw temperature: 0x%04X (%d decimal)", (uint16_t)raw_temp, raw_temp);

    // Convert to Celsius based on resolution
    switch (device->resolution) {
        case DS18B20_RESOLUTION_9BIT:
            raw_temp &= 0xFFF8;  // Mask lower 3 bits
            break;
        case DS18B20_RESOLUTION_10BIT:
            raw_temp &= 0xFFFC;  // Mask lower 2 bits
            break;
        case DS18B20_RESOLUTION_11BIT:
            raw_temp &= 0xFFFE;  // Mask lower 1 bit
            break;
        case DS18B20_RESOLUTION_12BIT:
        default:
            // Use all bits (no masking needed)
            break;
    }

    // Convert to float (LSB = 0.0625°C)
    *temperature = (float)raw_temp * 0.0625f;

    ESP_LOGI(TAG, "Converted temperature: %.4f°C (raw: 0x%04X)", *temperature, (uint16_t)raw_temp);

    return ESP_OK;
}

/**
 * @brief Read temperature from sensor (blocking)
 */
esp_err_t ds18b20_read_temperature(ds18b20_device_t *device, float *temperature)
{
    if (!device->initialized) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (temperature == NULL) {
        ESP_LOGE(TAG, "Temperature pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Start conversion
    esp_err_t ret = ds18b20_start_conversion(device);
    if (ret != ESP_OK) {
        return ret;
    }

    // Wait for conversion to complete
    // For external power mode, simple delay is sufficient
    // For parasitic mode, the pull-up resistor keeps the bus high during conversion
    uint16_t conversion_time = get_conversion_time_ms(device->resolution);
    vTaskDelay(pdMS_TO_TICKS(conversion_time));

    // Give the bus a moment to stabilize before reading
    vTaskDelay(pdMS_TO_TICKS(10));

    // Read temperature
    return ds18b20_read_temperature_async(device, temperature);
}

/**
 * @brief Check if DS18B20 is using parasitic power
 */
esp_err_t ds18b20_is_parasitic_power(ds18b20_device_t *device, bool *parasitic)
{
    if (!device->initialized) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (parasitic == NULL) {
        ESP_LOGE(TAG, "Parasitic pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Reset and select device
    if (!onewire_reset(device->bus)) {
        ESP_LOGE(TAG, "Device not responding");
        return ESP_ERR_NOT_FOUND;
    }

    if (device->use_rom) {
        onewire_match_rom(device->bus, device->rom);
    } else {
        onewire_skip_rom(device->bus);
    }

    // Read power supply command
    onewire_write_byte(device->bus, DS18B20_CMD_READ_POWER_SUPPLY);

    // CRITICAL: After sending READ POWER SUPPLY command, the DS18B20 will
    // actively pull the line LOW if in parasitic mode, or allow it to be
    // pulled HIGH (by pull-up resistor) if in external power mode.
    //
    // We need to:
    // 1. Initiate a read time slot (pull low briefly, then release)
    // 2. Wait for the device to respond
    // 3. Sample the bus level
    //
    // This is exactly what onewire_read_bit() does, but we need to ensure
    // proper timing. Let's add a small delay first for the device to prepare.

    // Wait a bit for device to be ready (DS18B20 datasheet says ~10us)
    ets_delay_us(20);

    // Now read the power mode bit using standard read slot timing
    // The DS18B20 will actively drive the line during the read slot
    uint8_t power_mode = onewire_read_bit(device->bus);

    ESP_LOGI(TAG, "Power supply read bit: %d (0=parasitic, 1=external)", power_mode);

    // Read it a second time to verify (sometimes first read is unreliable)
    ets_delay_us(10);
    uint8_t power_mode_verify = onewire_read_bit(device->bus);

    ESP_LOGI(TAG, "Power supply verify bit: %d", power_mode_verify);

    // Use the second reading (more reliable)
    *parasitic = (power_mode_verify == 0);

    ESP_LOGI(TAG, "Power mode: %s", *parasitic ? "Parasitic" : "External");

    // If parasitic mode is detected during development, log a warning
    if (*parasitic) {
        ESP_LOGW(TAG, "============================================");
        ESP_LOGW(TAG, "Parasitic mode detected!");
        ESP_LOGW(TAG, "If using external power (VDD to 3.3V):");
        ESP_LOGW(TAG, "  - Check VDD pin connection");
        ESP_LOGW(TAG, "  - Ensure VDD is connected to 3.3V, not GND");
        ESP_LOGW(TAG, "  - Verify no short to GND on VDD pin");
        ESP_LOGW(TAG, "============================================");
    }

    return ESP_OK;
}

/**
 * @brief Get conversion time for current resolution
 */
uint16_t ds18b20_get_conversion_time(ds18b20_device_t *device)
{
    if (!device->initialized) {
        return DS18B20_CONVERSION_TIME_12BIT;
    }

    return get_conversion_time_ms(device->resolution);
}

/**
 * @brief Search and initialize all DS18B20 devices on the bus
 */
int ds18b20_search_devices(onewire_bus_t *bus, ds18b20_device_t *devices, int max_devices)
{
    if (bus == NULL || devices == NULL || max_devices <= 0) {
        ESP_LOGE(TAG, "Invalid arguments");
        return 0;
    }

    if (!bus->initialized) {
        ESP_LOGE(TAG, "1-Wire bus not initialized");
        return 0;
    }

    // Allocate ROM codes array
    uint8_t rom_codes[max_devices][8];

    // Search for devices on the bus
    int device_count = onewire_search_devices(bus, rom_codes, max_devices);

    if (device_count == 0) {
        ESP_LOGW(TAG, "No DS18B20 devices found");
        return 0;
    }

    // Initialize each found device
    int initialized_count = 0;
    for (int i = 0; i < device_count; i++) {
        // Check if this is a DS18B20 (family code 0x28)
        if (rom_codes[i][0] != 0x28) {
            ESP_LOGW(TAG, "Device %d is not a DS18B20 (family code: 0x%02X)",
                     i, rom_codes[i][0]);
            continue;
        }

        // Initialize device
        esp_err_t ret = ds18b20_init(&devices[initialized_count], bus, rom_codes[i]);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Device %d initialized: ROM=%02X%02X%02X%02X%02X%02X%02X%02X",
                     initialized_count,
                     rom_codes[i][0], rom_codes[i][1], rom_codes[i][2], rom_codes[i][3],
                     rom_codes[i][4], rom_codes[i][5], rom_codes[i][6], rom_codes[i][7]);
            initialized_count++;
        }
    }

    ESP_LOGI(TAG, "Total DS18B20 devices initialized: %d", initialized_count);
    return initialized_count;
}
