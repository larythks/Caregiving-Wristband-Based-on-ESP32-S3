/**
 * @file button.c
 * @brief Button driver implementation for ESP32-S3 Wristband
 *
 * This driver implements debounced button input with event detection
 * using FreeRTOS tasks for periodic button scanning.
 *
 * @author larythks
 * @date 2026-01-02
 */

#include "button.h"
#include "app_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include <string.h>

static const char *TAG = TAG_BUTTON;

/**
 * @brief Button scan interval in milliseconds
 */
#define BUTTON_SCAN_INTERVAL_MS 10

/**
 * @brief Button state machine structure
 */
typedef struct {
    button_config_t config;         ///< Button configuration
    button_state_t state;           ///< Current state
    bool is_registered;             ///< Registration status
    bool is_enabled;                ///< Enable/disable status

    // Timing variables
    uint32_t press_start_time;      ///< Time when button was first pressed
    uint32_t last_release_time;     ///< Time of last button release
    uint32_t stable_time;           ///< Time when state became stable (after debounce)

    // State tracking
    bool raw_state;                 ///< Current raw GPIO state
    bool debounced_state;           ///< Debounced state
    bool prev_debounced_state;      ///< Previous debounced state
    bool long_press_sent;           ///< Long press event already sent
    uint8_t click_count;            ///< Click counter for double-click detection
} button_handle_t;

/**
 * @brief Button driver context
 */
typedef struct {
    button_handle_t buttons[BUTTON_MAX];    ///< Button handles
    TaskHandle_t scan_task_handle;          ///< Scan task handle
    SemaphoreHandle_t mutex;                ///< Mutex for thread safety
    bool is_initialized;                    ///< Initialization status
} button_driver_t;

static button_driver_t g_button_driver = {0};

/**
 * @brief Button name strings
 */
static const char *button_names[BUTTON_MAX] = {
    [BUTTON_BOOT] = "BOOT",
    [BUTTON_SW1]  = "SW1",
    [BUTTON_SW2]  = "SW2",
    [BUTTON_SW3]  = "SW3"
};

/**
 * @brief Get current time in milliseconds
 */
static inline uint32_t button_get_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/**
 * @brief Read raw button state from GPIO
 */
static bool button_read_raw_state(button_handle_t *btn)
{
    int level = gpio_get_level(btn->config.gpio);

    // Invert if button is active low
    if (btn->config.active_low) {
        return (level == 0);
    } else {
        return (level == 1);
    }
}

/**
 * @brief Process button events and call callback
 */
static void button_send_event(button_id_t button_id, button_event_t event)
{
    button_handle_t *btn = &g_button_driver.buttons[button_id];

    if (btn->config.callback) {
        btn->config.callback(button_id, event, btn->config.user_data);
    }

    ESP_LOGD(TAG, "Button %s: Event %d", button_names[button_id], event);
}

/**
 * @brief Update button state machine
 */
static void button_update_state(button_id_t button_id)
{
    button_handle_t *btn = &g_button_driver.buttons[button_id];
    uint32_t current_time = button_get_time_ms();

    if (!btn->is_registered || !btn->is_enabled) {
        return;
    }

    // Read current raw state
    btn->raw_state = button_read_raw_state(btn);

    // Debounce logic
    uint32_t time_since_stable = current_time - btn->stable_time;

    if (btn->raw_state != btn->debounced_state) {
        // State changed, check if debounce period has elapsed
        if (time_since_stable >= btn->config.debounce_ms) {
            // Update debounced state
            btn->prev_debounced_state = btn->debounced_state;
            btn->debounced_state = btn->raw_state;
            btn->stable_time = current_time;

            ESP_LOGD(TAG, "Button %s: Debounced state changed to %d",
                     button_names[button_id], btn->debounced_state);
        }
    } else {
        // State is stable, update stable time
        btn->stable_time = current_time;
    }

    // Detect state transitions
    bool rising_edge = btn->debounced_state && !btn->prev_debounced_state;
    bool falling_edge = !btn->debounced_state && btn->prev_debounced_state;

    // Button pressed (rising edge)
    if (rising_edge) {
        btn->press_start_time = current_time;
        btn->state = BUTTON_STATE_PRESSED;
        btn->long_press_sent = false;
        button_send_event(button_id, BUTTON_EVENT_PRESSED);
    }

    // Button released (falling edge)
    else if (falling_edge) {
        uint32_t press_duration = current_time - btn->press_start_time;
        btn->state = BUTTON_STATE_RELEASED;

        button_send_event(button_id, BUTTON_EVENT_RELEASED);

        // Check if it was a long press
        if (btn->long_press_sent) {
            // Long press already sent, just reset
            btn->long_press_sent = false;
        } else if (press_duration >= btn->config.long_press_ms) {
            // Long press completed
            button_send_event(button_id, BUTTON_EVENT_LONG_PRESS);
        } else {
            // Short press
            // Check for double-click
            uint32_t time_since_last_click = current_time - btn->last_release_time;

            if (time_since_last_click < btn->config.double_click_ms && btn->click_count == 1) {
                // Double click detected
                button_send_event(button_id, BUTTON_EVENT_DOUBLE_CLICK);
                btn->click_count = 0;
            } else {
                // First click or single click
                btn->click_count++;

                // Delay to check for second click
                // For now, just send short press immediately
                button_send_event(button_id, BUTTON_EVENT_SHORT_PRESS);
                btn->click_count = 1;
            }
        }

        btn->last_release_time = current_time;
        btn->state = BUTTON_STATE_IDLE;
    }

    // Check for long press while button is held
    else if (btn->debounced_state && !btn->long_press_sent) {
        uint32_t press_duration = current_time - btn->press_start_time;

        if (press_duration >= btn->config.long_press_ms) {
            btn->state = BUTTON_STATE_LONG_PRESS;
            btn->long_press_sent = true;
            button_send_event(button_id, BUTTON_EVENT_LONG_PRESS);
        }
    }

    // Reset click count if timeout
    if (btn->click_count > 0) {
        uint32_t time_since_last_click = current_time - btn->last_release_time;
        if (time_since_last_click >= btn->config.double_click_ms) {
            btn->click_count = 0;
        }
    }
}

/**
 * @brief Button scanning task
 */
static void button_scan_task(void *arg)
{
    ESP_LOGI(TAG, "Button scan task started");

    while (1) {
        // Take mutex
        if (xSemaphoreTake(g_button_driver.mutex, portMAX_DELAY) == pdTRUE) {
            // Scan all registered buttons
            for (button_id_t i = 0; i < BUTTON_MAX; i++) {
                if (g_button_driver.buttons[i].is_registered) {
                    button_update_state(i);
                }
            }

            // Release mutex
            xSemaphoreGive(g_button_driver.mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(BUTTON_SCAN_INTERVAL_MS));
    }
}

esp_err_t button_init(void)
{
    if (g_button_driver.is_initialized) {
        ESP_LOGW(TAG, "Button driver already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing button driver...");

    // Clear driver structure
    memset(&g_button_driver, 0, sizeof(button_driver_t));

    // Create mutex
    g_button_driver.mutex = xSemaphoreCreateMutex();
    if (g_button_driver.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize all buttons as unregistered
    for (int i = 0; i < BUTTON_MAX; i++) {
        g_button_driver.buttons[i].is_registered = false;
        g_button_driver.buttons[i].is_enabled = false;
    }

    // Create button scan task
    BaseType_t ret = xTaskCreate(
        button_scan_task,
        "button_scan",
        TASK_STACK_SIZE_SMALL,
        NULL,
        TASK_PRIORITY_HIGH,
        &g_button_driver.scan_task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button scan task");
        vSemaphoreDelete(g_button_driver.mutex);
        return ESP_ERR_NO_MEM;
    }

    g_button_driver.is_initialized = true;
    ESP_LOGI(TAG, "Button driver initialized successfully");

    return ESP_OK;
}

esp_err_t button_deinit(void)
{
    if (!g_button_driver.is_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing button driver...");

    // Delete scan task
    if (g_button_driver.scan_task_handle != NULL) {
        vTaskDelete(g_button_driver.scan_task_handle);
        g_button_driver.scan_task_handle = NULL;
    }

    // Unregister all buttons
    for (button_id_t i = 0; i < BUTTON_MAX; i++) {
        if (g_button_driver.buttons[i].is_registered) {
            button_unregister(i);
        }
    }

    // Delete mutex
    if (g_button_driver.mutex != NULL) {
        vSemaphoreDelete(g_button_driver.mutex);
        g_button_driver.mutex = NULL;
    }

    g_button_driver.is_initialized = false;
    ESP_LOGI(TAG, "Button driver deinitialized");

    return ESP_OK;
}

esp_err_t button_register(button_id_t button_id, const button_config_t *config)
{
    if (button_id >= BUTTON_MAX || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_button_driver.is_initialized) {
        ESP_LOGE(TAG, "Button driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_button_driver.mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    button_handle_t *btn = &g_button_driver.buttons[button_id];

    if (btn->is_registered) {
        xSemaphoreGive(g_button_driver.mutex);
        ESP_LOGW(TAG, "Button %s already registered", button_names[button_id]);
        return ESP_ERR_INVALID_STATE;
    }

    // Copy configuration
    memcpy(&btn->config, config, sizeof(button_config_t));

    // Set default values if not specified
    if (btn->config.debounce_ms == 0) {
        btn->config.debounce_ms = BUTTON_DEFAULT_DEBOUNCE_MS;
    }
    if (btn->config.long_press_ms == 0) {
        btn->config.long_press_ms = BUTTON_DEFAULT_LONG_PRESS_MS;
    }
    if (btn->config.double_click_ms == 0) {
        btn->config.double_click_ms = BUTTON_DEFAULT_DOUBLE_CLICK_MS;
    }

    // Configure GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = config->active_low ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = config->active_low ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        xSemaphoreGive(g_button_driver.mutex);
        ESP_LOGE(TAG, "Failed to configure GPIO %d for button %s",
                 config->gpio, button_names[button_id]);
        return ret;
    }

    // Initialize button state
    btn->state = BUTTON_STATE_IDLE;
    btn->is_registered = true;
    btn->is_enabled = true;
    btn->press_start_time = 0;
    btn->last_release_time = 0;
    btn->stable_time = button_get_time_ms();
    btn->raw_state = button_read_raw_state(btn);
    btn->debounced_state = btn->raw_state;
    btn->prev_debounced_state = btn->raw_state;
    btn->long_press_sent = false;
    btn->click_count = 0;

    xSemaphoreGive(g_button_driver.mutex);

    ESP_LOGI(TAG, "Button %s registered on GPIO %d (active %s)",
             button_names[button_id], config->gpio,
             config->active_low ? "LOW" : "HIGH");

    return ESP_OK;
}

esp_err_t button_unregister(button_id_t button_id)
{
    if (button_id >= BUTTON_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_button_driver.mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    button_handle_t *btn = &g_button_driver.buttons[button_id];

    if (!btn->is_registered) {
        xSemaphoreGive(g_button_driver.mutex);
        return ESP_ERR_NOT_FOUND;
    }

    // Reset GPIO
    gpio_reset_pin(btn->config.gpio);

    // Clear button handle
    memset(btn, 0, sizeof(button_handle_t));
    btn->is_registered = false;

    xSemaphoreGive(g_button_driver.mutex);

    ESP_LOGI(TAG, "Button %s unregistered", button_names[button_id]);

    return ESP_OK;
}

esp_err_t button_set_callback(button_id_t button_id, button_callback_t callback, void *user_data)
{
    if (button_id >= BUTTON_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_button_driver.mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    button_handle_t *btn = &g_button_driver.buttons[button_id];

    if (!btn->is_registered) {
        xSemaphoreGive(g_button_driver.mutex);
        return ESP_ERR_NOT_FOUND;
    }

    btn->config.callback = callback;
    btn->config.user_data = user_data;

    xSemaphoreGive(g_button_driver.mutex);

    return ESP_OK;
}

esp_err_t button_get_state(button_id_t button_id, button_state_t *state)
{
    if (button_id >= BUTTON_MAX || state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_button_driver.mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    button_handle_t *btn = &g_button_driver.buttons[button_id];

    if (!btn->is_registered) {
        xSemaphoreGive(g_button_driver.mutex);
        return ESP_ERR_NOT_FOUND;
    }

    *state = btn->state;

    xSemaphoreGive(g_button_driver.mutex);

    return ESP_OK;
}

bool button_is_pressed(button_id_t button_id)
{
    if (button_id >= BUTTON_MAX) {
        return false;
    }

    button_handle_t *btn = &g_button_driver.buttons[button_id];

    if (!btn->is_registered) {
        return false;
    }

    return btn->debounced_state;
}

uint32_t button_get_press_duration(button_id_t button_id)
{
    if (button_id >= BUTTON_MAX) {
        return 0;
    }

    button_handle_t *btn = &g_button_driver.buttons[button_id];

    if (!btn->is_registered || !btn->debounced_state) {
        return 0;
    }

    return button_get_time_ms() - btn->press_start_time;
}

esp_err_t button_set_enable(button_id_t button_id, bool enable)
{
    if (button_id >= BUTTON_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(g_button_driver.mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    button_handle_t *btn = &g_button_driver.buttons[button_id];

    if (!btn->is_registered) {
        xSemaphoreGive(g_button_driver.mutex);
        return ESP_ERR_NOT_FOUND;
    }

    btn->is_enabled = enable;

    xSemaphoreGive(g_button_driver.mutex);

    ESP_LOGI(TAG, "Button %s %s", button_names[button_id],
             enable ? "enabled" : "disabled");

    return ESP_OK;
}

const char* button_get_name(button_id_t button_id)
{
    if (button_id >= BUTTON_MAX) {
        return "UNKNOWN";
    }

    return button_names[button_id];
}
