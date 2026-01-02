/**
 * @file button.h
 * @brief Button driver for ESP32-S3 Wristband
 *
 * This driver provides debounced button input with support for:
 * - Short press detection
 * - Long press detection
 * - Double click detection
 * - Multi-button support
 *
 * @author larythks
 * @date 2026-01-02
 */

#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Button identifiers
 */
typedef enum {
    BUTTON_BOOT = 0,    ///< BOOT button (GPIO0) - system use
    BUTTON_SW1,         ///< SW1 - Alarm trigger button
    BUTTON_SW2,         ///< SW2 - Power switch button
    BUTTON_SW3,         ///< SW3 - Reset/Menu button
    BUTTON_MAX          ///< Maximum button count
} button_id_t;

/**
 * @brief Button event types
 */
typedef enum {
    BUTTON_EVENT_NONE = 0,      ///< No event
    BUTTON_EVENT_PRESSED,       ///< Button pressed (down edge)
    BUTTON_EVENT_RELEASED,      ///< Button released (up edge)
    BUTTON_EVENT_SHORT_PRESS,   ///< Short press completed
    BUTTON_EVENT_LONG_PRESS,    ///< Long press detected
    BUTTON_EVENT_DOUBLE_CLICK,  ///< Double click detected
    BUTTON_EVENT_MAX
} button_event_t;

/**
 * @brief Button state
 */
typedef enum {
    BUTTON_STATE_IDLE = 0,      ///< Button idle (not pressed)
    BUTTON_STATE_PRESSED,       ///< Button currently pressed
    BUTTON_STATE_RELEASED,      ///< Button just released
    BUTTON_STATE_LONG_PRESS,    ///< Long press in progress
    BUTTON_STATE_MAX
} button_state_t;

/**
 * @brief Button callback function type
 *
 * @param button_id Button identifier
 * @param event Event type
 * @param user_data User data passed during registration
 */
typedef void (*button_callback_t)(button_id_t button_id, button_event_t event, void *user_data);

/**
 * @brief Button configuration structure
 */
typedef struct {
    gpio_num_t gpio;                ///< GPIO number
    bool active_low;                ///< True if button is active low
    uint32_t debounce_ms;           ///< Debounce time in milliseconds
    uint32_t long_press_ms;         ///< Long press threshold in milliseconds
    uint32_t double_click_ms;       ///< Double click max interval in milliseconds
    button_callback_t callback;     ///< Event callback function
    void *user_data;                ///< User data for callback
} button_config_t;

/**
 * @brief Default button configuration values
 */
#define BUTTON_DEFAULT_DEBOUNCE_MS      50      ///< 50ms debounce time
#define BUTTON_DEFAULT_LONG_PRESS_MS    1000    ///< 1 second for long press
#define BUTTON_DEFAULT_DOUBLE_CLICK_MS  500     ///< 500ms max interval for double click

/**
 * @brief Initialize button driver
 *
 * Initializes GPIO for all configured buttons and starts the button
 * scanning task.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 * @return ESP_FAIL on other errors
 */
esp_err_t button_init(void);

/**
 * @brief Deinitialize button driver
 *
 * Stops the button scanning task and releases resources.
 *
 * @return ESP_OK on success
 */
esp_err_t button_deinit(void);

/**
 * @brief Register a button with custom configuration
 *
 * @param button_id Button identifier
 * @param config Pointer to configuration structure
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parameters are invalid
 * @return ESP_ERR_INVALID_STATE if button already registered
 */
esp_err_t button_register(button_id_t button_id, const button_config_t *config);

/**
 * @brief Unregister a button
 *
 * @param button_id Button identifier
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if button_id is invalid
 * @return ESP_ERR_NOT_FOUND if button not registered
 */
esp_err_t button_unregister(button_id_t button_id);

/**
 * @brief Set callback for button events
 *
 * @param button_id Button identifier
 * @param callback Callback function
 * @param user_data User data to pass to callback
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if button_id is invalid
 * @return ESP_ERR_NOT_FOUND if button not registered
 */
esp_err_t button_set_callback(button_id_t button_id, button_callback_t callback, void *user_data);

/**
 * @brief Get current button state
 *
 * @param button_id Button identifier
 * @param state Pointer to store current state
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parameters are invalid
 * @return ESP_ERR_NOT_FOUND if button not registered
 */
esp_err_t button_get_state(button_id_t button_id, button_state_t *state);

/**
 * @brief Check if button is currently pressed
 *
 * @param button_id Button identifier
 * @return true if button is pressed
 * @return false if button is not pressed or invalid
 */
bool button_is_pressed(button_id_t button_id);

/**
 * @brief Get button press duration (in milliseconds)
 *
 * Returns the duration the button has been held down.
 *
 * @param button_id Button identifier
 * @return Press duration in milliseconds, 0 if not pressed or invalid
 */
uint32_t button_get_press_duration(button_id_t button_id);

/**
 * @brief Enable/disable button
 *
 * Disabled buttons will not generate events.
 *
 * @param button_id Button identifier
 * @param enable true to enable, false to disable
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if button_id is invalid
 * @return ESP_ERR_NOT_FOUND if button not registered
 */
esp_err_t button_set_enable(button_id_t button_id, bool enable);

/**
 * @brief Get button name string
 *
 * @param button_id Button identifier
 * @return Button name string, "UNKNOWN" if invalid
 */
const char* button_get_name(button_id_t button_id);

#ifdef __cplusplus
}
#endif

#endif // BUTTON_H
