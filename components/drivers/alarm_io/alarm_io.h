/**
 * @file alarm_io.h
 * @brief LED and Alarm I/O driver for ESP32-S3 Wristband
 *
 * This driver provides control for:
 * - LED1 (Power indicator) - GPIO6
 * - LED2 (Battery full indicator) - GPIO2
 * - LED3 (Charging indicator) - GPIO1
 * - Alarm mode with LED flashing and speaker alert
 *
 * @author larythks
 * @date 2026-01-02
 */

#ifndef ALARM_IO_H
#define ALARM_IO_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LED identifiers
 */
typedef enum {
    LED_POWER = 0,      ///< LED1 - Power indicator (GPIO6)
    LED_FULL,           ///< LED2 - Battery full indicator (GPIO2)
    LED_CHARGING,       ///< LED3 - Charging indicator (GPIO1)
    LED_MAX
} led_id_t;

/**
 * @brief LED mode
 */
typedef enum {
    LED_MODE_OFF = 0,       ///< LED off
    LED_MODE_ON,            ///< LED on (steady)
    LED_MODE_BLINK,         ///< LED blinking
    LED_MODE_BREATHE,       ///< LED breathing (PWM)
    LED_MODE_ALARM          ///< LED alarm mode (rapid flash)
} led_mode_t;

/**
 * @brief Initialize LED and alarm I/O
 *
 * Configures GPIO pins for all LEDs and initializes PWM if needed.
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on error
 */
esp_err_t alarm_io_init(void);

/**
 * @brief Deinitialize LED and alarm I/O
 *
 * @return ESP_OK on success
 */
esp_err_t alarm_io_deinit(void);

/**
 * @brief Set LED state (on/off)
 *
 * @param led_id LED identifier
 * @param state true=on, false=off
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if led_id is invalid
 */
esp_err_t led_set(led_id_t led_id, bool state);

/**
 * @brief Toggle LED state
 *
 * @param led_id LED identifier
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if led_id is invalid
 */
esp_err_t led_toggle(led_id_t led_id);

/**
 * @brief Get current LED state
 *
 * @param led_id LED identifier
 * @return true if LED is on, false if off or invalid
 */
bool led_get_state(led_id_t led_id);

/**
 * @brief Set LED mode
 *
 * @param led_id LED identifier
 * @param mode LED mode
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parameters are invalid
 */
esp_err_t led_set_mode(led_id_t led_id, led_mode_t mode);

/**
 * @brief Set LED brightness (PWM duty cycle)
 *
 * @param led_id LED identifier
 * @param brightness Brightness level (0-100%)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parameters are invalid
 * @return ESP_ERR_NOT_SUPPORTED if PWM is not initialized
 */
esp_err_t led_set_brightness(led_id_t led_id, uint8_t brightness);

/**
 * @brief Set LED blink parameters
 *
 * @param led_id LED identifier
 * @param on_time_ms On duration in milliseconds
 * @param off_time_ms Off duration in milliseconds
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parameters are invalid
 */
esp_err_t led_set_blink(led_id_t led_id, uint32_t on_time_ms, uint32_t off_time_ms);

/**
 * @brief Start alarm mode
 *
 * Activates alarm with:
 * - LED1 rapid flashing (10Hz)
 * - Speaker alert tone (if enabled)
 * - Auto-stop after timeout or manual stop
 *
 * @return ESP_OK on success
 * @return ESP_FAIL if alarm is already active
 */
esp_err_t alarm_start(void);

/**
 * @brief Stop alarm mode
 *
 * @return ESP_OK on success
 */
esp_err_t alarm_stop(void);

/**
 * @brief Check if alarm is currently active
 *
 * @return true if alarm is active, false otherwise
 */
bool alarm_is_active(void);

/**
 * @brief Get alarm duration (in seconds)
 *
 * @return Alarm duration in seconds, 0 if not active
 */
uint32_t alarm_get_duration(void);

/**
 * @brief Set alarm auto-stop timeout
 *
 * @param timeout_sec Timeout in seconds (0 = no auto-stop)
 * @return ESP_OK on success
 */
esp_err_t alarm_set_timeout(uint32_t timeout_sec);

#ifdef __cplusplus
}
#endif

#endif // ALARM_IO_H
