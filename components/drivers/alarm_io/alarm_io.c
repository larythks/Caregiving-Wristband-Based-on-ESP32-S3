/**
 * @file alarm_io.c
 * @brief LED and Alarm I/O driver implementation for ESP32-S3 Wristband
 *
 * This driver implements control for LEDs and alarm functionality
 * using GPIO and PWM (LEDC) for advanced LED modes.
 *
 * @author larythks
 * @date 2026-01-02
 */

#include "alarm_io.h"
#include "app_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = TAG_LED;

/**
 * @brief LEDC configuration for PWM control
 */
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT  // 13-bit resolution (0-8191)
#define LEDC_FREQUENCY          5000               // 5kHz PWM frequency
#define LEDC_MAX_DUTY           ((1 << LEDC_DUTY_RES) - 1)

/**
 * @brief LED channel assignments for LEDC
 */
#define LEDC_CHANNEL_LED1       LEDC_CHANNEL_0
#define LEDC_CHANNEL_LED2       LEDC_CHANNEL_1
#define LEDC_CHANNEL_LED3       LEDC_CHANNEL_2

/**
 * @brief LED state structure
 */
typedef struct {
    gpio_num_t gpio;            ///< GPIO number
    ledc_channel_t channel;     ///< LEDC channel for PWM
    led_mode_t mode;            ///< Current mode
    bool state;                 ///< Current state (on/off)
    uint8_t brightness;         ///< Brightness level (0-100%)
    uint32_t on_time_ms;        ///< Blink on time
    uint32_t off_time_ms;       ///< Blink off time
    uint32_t last_toggle_time;  ///< Last toggle timestamp
    bool pwm_enabled;           ///< PWM channel configured
} led_state_t;

/**
 * @brief Alarm state structure
 */
typedef struct {
    bool active;                ///< Alarm active flag
    uint32_t start_time;        ///< Alarm start timestamp (seconds)
    uint32_t timeout_sec;       ///< Auto-stop timeout (0 = no timeout)
    TaskHandle_t task_handle;   ///< Alarm task handle
} alarm_state_t;

/**
 * @brief Driver context
 */
typedef struct {
    led_state_t leds[LED_MAX];      ///< LED states
    alarm_state_t alarm;            ///< Alarm state
    SemaphoreHandle_t mutex;        ///< Mutex for thread safety
    bool initialized;               ///< Initialization flag
} alarm_io_driver_t;

static alarm_io_driver_t g_driver = {0};

/**
 * @brief LED GPIO mapping
 */
static const gpio_num_t led_gpio_map[LED_MAX] = {
    [LED_POWER]    = LED1_POWER_GPIO,
    [LED_FULL]     = LED2_FULL_GPIO,
    [LED_CHARGING] = LED3_CHRG_GPIO
};

/**
 * @brief LED LEDC channel mapping
 */
static const ledc_channel_t led_channel_map[LED_MAX] = {
    [LED_POWER]    = LEDC_CHANNEL_LED1,
    [LED_FULL]     = LEDC_CHANNEL_LED2,
    [LED_CHARGING] = LEDC_CHANNEL_LED3
};

/**
 * @brief Get current time in seconds
 */
static inline uint32_t get_time_sec(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000ULL);
}

/**
 * @brief Get current time in milliseconds
 */
static inline uint32_t get_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/**
 * @brief Configure PWM for LED
 */
static esp_err_t led_setup_pwm(led_id_t led_id)
{
    if (led_id >= LED_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    led_state_t *led = &g_driver.leds[led_id];

    if (led->pwm_enabled) {
        return ESP_OK;  // Already configured
    }

    // Configure LEDC channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num   = led->gpio,
        .speed_mode = LEDC_MODE,
        .channel    = led->channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };

    esp_err_t ret = ledc_channel_config(&ledc_channel);
    if (ret == ESP_OK) {
        led->pwm_enabled = true;
        ESP_LOGD(TAG, "PWM configured for LED%d on GPIO%d", led_id, led->gpio);
    } else {
        ESP_LOGE(TAG, "Failed to configure PWM for LED%d: %s", led_id, esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief Set LED output using GPIO or PWM
 */
static esp_err_t led_set_output(led_id_t led_id, bool state, uint8_t brightness)
{
    if (led_id >= LED_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    led_state_t *led = &g_driver.leds[led_id];

    if (brightness < 100 && brightness > 0) {
        // Use PWM for brightness control
        if (!led->pwm_enabled) {
            esp_err_t ret = led_setup_pwm(led_id);
            if (ret != ESP_OK) {
                return ret;
            }
        }

        uint32_t duty = state ? (LEDC_MAX_DUTY * brightness / 100) : 0;
        return ledc_set_duty(LEDC_MODE, led->channel, duty) == ESP_OK ?
               ledc_update_duty(LEDC_MODE, led->channel) : ESP_FAIL;
    } else {
        // Use simple GPIO for on/off
        if (led->pwm_enabled) {
            // Stop PWM and reconfigure as GPIO
            ledc_stop(LEDC_MODE, led->channel, 0);
            led->pwm_enabled = false;

            gpio_config_t io_conf = {
                .pin_bit_mask = (1ULL << led->gpio),
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE
            };
            gpio_config(&io_conf);
        }

        return gpio_set_level(led->gpio, state ? 1 : 0);
    }
}

/**
 * @brief LED blink task
 */
static void led_blink_task(void *arg)
{
    while (1) {
        if (xSemaphoreTake(g_driver.mutex, portMAX_DELAY) == pdTRUE) {
            uint32_t current_time = get_time_ms();

            for (led_id_t i = 0; i < LED_MAX; i++) {
                led_state_t *led = &g_driver.leds[i];

                if (led->mode == LED_MODE_BLINK || led->mode == LED_MODE_ALARM) {
                    uint32_t elapsed = current_time - led->last_toggle_time;
                    uint32_t threshold = led->state ? led->on_time_ms : led->off_time_ms;

                    if (elapsed >= threshold) {
                        led->state = !led->state;
                        led->last_toggle_time = current_time;
                        led_set_output(i, led->state, led->brightness);
                    }
                }
            }

            xSemaphoreGive(g_driver.mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms update interval
    }
}

/**
 * @brief Alarm task
 */
static void alarm_task(void *arg)
{
    ESP_LOGI(TAG, "Alarm task started");

    while (g_driver.alarm.active) {
        // Check for timeout
        if (g_driver.alarm.timeout_sec > 0) {
            uint32_t duration = get_time_sec() - g_driver.alarm.start_time;
            if (duration >= g_driver.alarm.timeout_sec) {
                ESP_LOGI(TAG, "Alarm timeout reached (%lu sec), stopping", g_driver.alarm.timeout_sec);
                alarm_stop();
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "Alarm task exiting");
    g_driver.alarm.task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t alarm_io_init(void)
{
    if (g_driver.initialized) {
        ESP_LOGW(TAG, "Alarm I/O already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing alarm I/O driver...");

    memset(&g_driver, 0, sizeof(alarm_io_driver_t));

    // Create mutex
    g_driver.mutex = xSemaphoreCreateMutex();
    if (g_driver.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = LEDC_FREQUENCY,
        .clk_cfg         = LEDC_AUTO_CLK
    };

    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer");
        vSemaphoreDelete(g_driver.mutex);
        return ret;
    }

    // Initialize LED states
    for (led_id_t i = 0; i < LED_MAX; i++) {
        g_driver.leds[i].gpio = led_gpio_map[i];
        g_driver.leds[i].channel = led_channel_map[i];
        g_driver.leds[i].mode = LED_MODE_OFF;
        g_driver.leds[i].state = false;
        g_driver.leds[i].brightness = 100;
        g_driver.leds[i].on_time_ms = 500;
        g_driver.leds[i].off_time_ms = 500;
        g_driver.leds[i].last_toggle_time = 0;
        g_driver.leds[i].pwm_enabled = false;

        // Configure GPIO as output
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << g_driver.leds[i].gpio),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };

        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure GPIO%d for LED%d", g_driver.leds[i].gpio, i);
            vSemaphoreDelete(g_driver.mutex);
            return ret;
        }

        // Turn off LED initially
        gpio_set_level(g_driver.leds[i].gpio, 0);
    }

    // Initialize alarm state
    g_driver.alarm.active = false;
    g_driver.alarm.start_time = 0;
    g_driver.alarm.timeout_sec = ALARM_AUTO_STOP_SEC;
    g_driver.alarm.task_handle = NULL;

    g_driver.initialized = true;

    ESP_LOGI(TAG, "Alarm I/O driver initialized successfully");
    return ESP_OK;
}

esp_err_t alarm_io_deinit(void)
{
    if (!g_driver.initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing alarm I/O driver...");

    // Stop alarm if active
    if (g_driver.alarm.active) {
        alarm_stop();
    }

    // Turn off all LEDs and release resources
    for (led_id_t i = 0; i < LED_MAX; i++) {
        led_set(i, false);

        if (g_driver.leds[i].pwm_enabled) {
            ledc_stop(LEDC_MODE, g_driver.leds[i].channel, 0);
        }

        gpio_reset_pin(g_driver.leds[i].gpio);
    }

    // Delete mutex
    if (g_driver.mutex != NULL) {
        vSemaphoreDelete(g_driver.mutex);
        g_driver.mutex = NULL;
    }

    g_driver.initialized = false;

    ESP_LOGI(TAG, "Alarm I/O driver deinitialized");
    return ESP_OK;
}

esp_err_t led_set(led_id_t led_id, bool state)
{
    if (led_id >= LED_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_driver.initialized) {
        ESP_LOGE(TAG, "Driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_driver.mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    led_state_t *led = &g_driver.leds[led_id];
    led->state = state;
    led->mode = state ? LED_MODE_ON : LED_MODE_OFF;

    esp_err_t ret = led_set_output(led_id, state, led->brightness);

    xSemaphoreGive(g_driver.mutex);

    ESP_LOGD(TAG, "LED%d set to %s", led_id, state ? "ON" : "OFF");
    return ret;
}

esp_err_t led_toggle(led_id_t led_id)
{
    if (led_id >= LED_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_driver.initialized) {
        ESP_LOGE(TAG, "Driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    bool current_state = g_driver.leds[led_id].state;
    return led_set(led_id, !current_state);
}

bool led_get_state(led_id_t led_id)
{
    if (led_id >= LED_MAX || !g_driver.initialized) {
        return false;
    }

    return g_driver.leds[led_id].state;
}

esp_err_t led_set_mode(led_id_t led_id, led_mode_t mode)
{
    if (led_id >= LED_MAX || mode >= LED_MODE_ALARM + 1) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_driver.initialized) {
        ESP_LOGE(TAG, "Driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_driver.mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    led_state_t *led = &g_driver.leds[led_id];
    led->mode = mode;
    led->last_toggle_time = get_time_ms();

    switch (mode) {
        case LED_MODE_OFF:
            led->state = false;
            led_set_output(led_id, false, led->brightness);
            break;

        case LED_MODE_ON:
            led->state = true;
            led_set_output(led_id, true, led->brightness);
            break;

        case LED_MODE_BLINK:
            // Blink task will handle this
            break;

        case LED_MODE_BREATHE:
            // TODO: Implement breathing effect using PWM fade
            ESP_LOGW(TAG, "Breathing mode not yet implemented");
            break;

        case LED_MODE_ALARM:
            // Set rapid flash parameters for alarm
            led->on_time_ms = 1000 / (ALARM_LED_FLASH_FREQ_HZ * 2);
            led->off_time_ms = led->on_time_ms;
            break;

        default:
            xSemaphoreGive(g_driver.mutex);
            return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreGive(g_driver.mutex);

    ESP_LOGI(TAG, "LED%d mode set to %d", led_id, mode);
    return ESP_OK;
}

esp_err_t led_set_brightness(led_id_t led_id, uint8_t brightness)
{
    if (led_id >= LED_MAX || brightness > 100) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_driver.initialized) {
        ESP_LOGE(TAG, "Driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_driver.mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    g_driver.leds[led_id].brightness = brightness;

    // Update output if LED is currently on
    esp_err_t ret = ESP_OK;
    if (g_driver.leds[led_id].state) {
        ret = led_set_output(led_id, true, brightness);
    }

    xSemaphoreGive(g_driver.mutex);

    ESP_LOGD(TAG, "LED%d brightness set to %d%%", led_id, brightness);
    return ret;
}

esp_err_t led_set_blink(led_id_t led_id, uint32_t on_time_ms, uint32_t off_time_ms)
{
    if (led_id >= LED_MAX || on_time_ms == 0 || off_time_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_driver.initialized) {
        ESP_LOGE(TAG, "Driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_driver.mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    g_driver.leds[led_id].on_time_ms = on_time_ms;
    g_driver.leds[led_id].off_time_ms = off_time_ms;

    xSemaphoreGive(g_driver.mutex);

    ESP_LOGI(TAG, "LED%d blink set to ON:%lums OFF:%lums", led_id, on_time_ms, off_time_ms);
    return ESP_OK;
}

esp_err_t alarm_start(void)
{
    if (!g_driver.initialized) {
        ESP_LOGE(TAG, "Driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (g_driver.alarm.active) {
        ESP_LOGW(TAG, "Alarm already active");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Starting alarm...");

    g_driver.alarm.active = true;
    g_driver.alarm.start_time = get_time_sec();

    // Set LED1 to alarm mode (rapid flash)
    led_set_mode(LED_POWER, LED_MODE_ALARM);

    // Create alarm task to monitor timeout
    BaseType_t ret = xTaskCreate(
        alarm_task,
        "alarm_task",
        TASK_STACK_SIZE_SMALL,
        NULL,
        TASK_PRIORITY_HIGH,
        &g_driver.alarm.task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create alarm task");
        g_driver.alarm.active = false;
        led_set_mode(LED_POWER, LED_MODE_OFF);
        return ESP_ERR_NO_MEM;
    }

    // Create LED blink task if not already running
    static TaskHandle_t blink_task_handle = NULL;
    if (blink_task_handle == NULL) {
        ret = xTaskCreate(
            led_blink_task,
            "led_blink",
            TASK_STACK_SIZE_SMALL,
            NULL,
            TASK_PRIORITY_MEDIUM,
            &blink_task_handle
        );

        if (ret != pdPASS) {
            ESP_LOGE(TAG, "Failed to create LED blink task");
        }
    }

    ESP_LOGI(TAG, "Alarm started successfully");
    return ESP_OK;
}

esp_err_t alarm_stop(void)
{
    if (!g_driver.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!g_driver.alarm.active) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping alarm...");

    g_driver.alarm.active = false;

    // Stop LED alarm mode
    led_set_mode(LED_POWER, LED_MODE_OFF);

    // Alarm task will exit automatically
    // Note: We don't delete the task here as it will delete itself

    ESP_LOGI(TAG, "Alarm stopped");
    return ESP_OK;
}

bool alarm_is_active(void)
{
    if (!g_driver.initialized) {
        return false;
    }

    return g_driver.alarm.active;
}

uint32_t alarm_get_duration(void)
{
    if (!g_driver.initialized || !g_driver.alarm.active) {
        return 0;
    }

    return get_time_sec() - g_driver.alarm.start_time;
}

esp_err_t alarm_set_timeout(uint32_t timeout_sec)
{
    if (!g_driver.initialized) {
        ESP_LOGE(TAG, "Driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    g_driver.alarm.timeout_sec = timeout_sec;

    ESP_LOGI(TAG, "Alarm timeout set to %lu seconds", timeout_sec);
    return ESP_OK;
}
