/**
 * @file app_config.h
 * @brief ESP32-S3 Wristband Global Configuration
 * @author Claude AI Assistant
 * @date 2025-12-30
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== Hardware Pin Definitions ==================== */

/* I2C Configuration */
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_SDA_IO       8           // GPIO8
#define I2C_MASTER_SCL_IO       9           // GPIO9
#define I2C_MASTER_FREQ_HZ      400000      // 400kHz
#define I2C_MASTER_TIMEOUT_MS   1000

/* I2C Device Addresses */
#define OLED_I2C_ADDRESS        0x3C        // SSH1106 OLED
#define MAX30102_I2C_ADDRESS    0x57        // MAX30102 Heart Rate Sensor
#define MPU6050_I2C_ADDRESS     0x68        // MPU6050 IMU (AD0=GND)

/* DS18B20 Temperature Sensor (1-Wire) */
#define DS18B20_GPIO            4           // GPIO4

/* MPU6050 Interrupt */
#define MPU6050_INT_GPIO        46          // GPIO46

/* MAX30102 Interrupt */
#define MAX30102_INT_GPIO       5           // GPIO5

/* I2S Microphone (INMP441) */
#define I2S_MIC_SCK_IO          15          // GPIO15
#define I2S_MIC_WS_IO           17          // GPIO17
#define I2S_MIC_SD_IO           16          // GPIO16

/* I2S Speaker (MAX98357A) */
#define I2S_SPK_BCLK_IO         16          // GPIO16 (shared with mic)
#define I2S_SPK_LRCLK_IO        17          // GPIO17 (shared with mic)
#define I2S_SPK_DIN_IO          18          // GPIO18

/* Button GPIOs */
#define BUTTON_BOOT_GPIO        0           // GPIO0 (BOOT button - system use)
#define BUTTON_SW1_GPIO         -1          // TODO: Define GPIO - Alarm button (LED1 flash + speaker alert)
#define BUTTON_SW2_GPIO         -1          // TODO: Define GPIO - Power switch (toggle ON/OFF)
#define BUTTON_SW3_GPIO         -1          // TODO: Define GPIO - Reset button

/* LED GPIOs */
#define LED1_POWER_GPIO         6           // GPIO6 - Power indicator (ON when device powered)
#define LED2_FULL_GPIO          2           // GPIO2 - Battery full indicator (TP4056 STDBY)
#define LED3_CHRG_GPIO          1           // GPIO1 - Charging indicator (TP4056 CHRG)

/* Battery Management */
#define BATTERY_ADC_CHANNEL     ADC_CHANNEL_3  // GPIO38 (ADC1_CH3)
#define BATTERY_ADC_GPIO        38          // GPIO38
#define BATTERY_CHRG_GPIO       LED3_CHRG_GPIO  // GPIO1 (TP4056 CHRG# output - active low)
#define BATTERY_STDBY_GPIO      LED2_FULL_GPIO  // GPIO2 (TP4056 STDBY# output - active low)

/* Voltage divider for battery (R15=100k, R16=100k) */
#define BATTERY_VOLTAGE_DIVIDER 2.0f        // Divider ratio
#define BATTERY_MIN_VOLTAGE     3.0f        // Minimum battery voltage (V)
#define BATTERY_MAX_VOLTAGE     4.2f        // Maximum battery voltage (V)

/* ==================== System Configuration ==================== */

/* FreeRTOS Task Priorities */
#define TASK_PRIORITY_HIGH      5           // Button, urgent events
#define TASK_PRIORITY_MEDIUM    3           // Sensor data collection
#define TASK_PRIORITY_LOW       1           // Background tasks

/* Task Stack Sizes */
#define TASK_STACK_SIZE_LARGE   4096
#define TASK_STACK_SIZE_MEDIUM  2048
#define TASK_STACK_SIZE_SMALL   1024

/* ==================== Feature Flags ==================== */

#define ENABLE_OLED_DISPLAY     1           // Enable OLED display
#define ENABLE_DS18B20          1           // Enable temperature sensor
#define ENABLE_MAX30102         1           // Enable heart rate sensor
#define ENABLE_MPU6050          1           // Enable motion sensor
#define ENABLE_I2S_MIC          1           // Enable microphone
#define ENABLE_I2S_SPEAKER      1           // Enable speaker (for TTS and alarm)
#define ENABLE_BLE              1           // Enable BLE (for WeChat Mini Program)
#define ENABLE_WECHAT_MINIPROGRAM 1         // Enable WeChat Mini Program integration
#define ENABLE_VOICE_RECOGNITION  1         // Enable voice recognition
#define ENABLE_VOICE_TTS        1           // Enable voice playback/TTS
#define ENABLE_WIFI             0           // WiFi disabled for now

/* ==================== Voice Recognition Configuration ==================== */

/* Wake-up Keywords */
#define WAKEUP_KEYWORD_1        "小手环"    // Wake-up keyword 1
#define WAKEUP_KEYWORD_2        "你好手环"  // Wake-up keyword 2

/* Voice Commands */
#define VOICE_CMD_QUERY_HR      "查询心率"  // Query heart rate
#define VOICE_CMD_QUERY_STEPS   "查询步数"  // Query step count
#define VOICE_CMD_CALL_FAMILY   "呼叫家人"  // Emergency call

/* Voice Recognition Parameters */
#define VOICE_SAMPLE_RATE       16000       // 16kHz sample rate
#define VOICE_FRAME_SIZE        512         // Frame size for processing
#define VOICE_DETECTION_THRESHOLD 1000      // Energy threshold for voice detection
#define VOICE_TIMEOUT_MS        5000        // Command timeout (5 seconds)

/* ==================== WeChat Mini Program Configuration ==================== */

/* BLE Service UUID (Custom Health Monitoring Service) */
#define WECHAT_BLE_SERVICE_UUID     "0000FFE0-0000-1000-8000-00805F9B34FB"

/* BLE Characteristic UUIDs */
#define WECHAT_CHAR_HEART_RATE_UUID "0000FFE1-0000-1000-8000-00805F9B34FB"  // Heart rate
#define WECHAT_CHAR_SPO2_UUID       "0000FFE2-0000-1000-8000-00805F9B34FB"  // SpO2
#define WECHAT_CHAR_TEMP_UUID       "0000FFE3-0000-1000-8000-00805F9B34FB"  // Temperature
#define WECHAT_CHAR_STEPS_UUID      "0000FFE4-0000-1000-8000-00805F9B34FB"  // Step count
#define WECHAT_CHAR_ALARM_UUID      "0000FFE5-0000-1000-8000-00805F9B34FB"  // Alarm/Emergency

/* Data Upload Intervals */
#define WECHAT_UPLOAD_INTERVAL_HR_MS    5000    // Heart rate upload every 5 seconds
#define WECHAT_UPLOAD_INTERVAL_STEPS_MS 60000   // Steps upload every 1 minute
#define WECHAT_UPLOAD_INTERVAL_TEMP_MS  10000   // Temperature upload every 10 seconds

/* Emergency Call Configuration */
#define EMERGENCY_CALL_TIMEOUT_MS   30000   // Emergency call notification timeout (30s)
#define EMERGENCY_CALL_RETRY_COUNT  3       // Retry count for emergency notification

/* ==================== Alarm System Configuration ==================== */

/* Alarm LED Flash Configuration */
#define ALARM_LED_FLASH_FREQ_HZ     10      // LED flash frequency (10Hz = rapid flashing)
#define ALARM_LED_DUTY_CYCLE        50      // PWM duty cycle percentage (50% = equal on/off)

/* Alarm Speaker Configuration */
#define ALARM_TONE_FREQ_HZ          2000    // Alert tone frequency (2kHz)
#define ALARM_TONE_DURATION_MS      200     // Single beep duration
#define ALARM_TONE_INTERVAL_MS      100     // Interval between beeps
#define ALARM_AUTO_STOP_SEC         30      // Auto-stop alarm after 30 seconds

/* ==================== LED Behavior Configuration ==================== */

/* LED1 (Power Indicator) */
#define LED1_ALWAYS_ON              true    // LED1 stays on when device is powered

/* LED2 (Battery Full) - Controlled by TP4056 STDBY pin */
#define LED2_ACTIVE_LOW             true    // Active low (LOW = battery full)

/* LED3 (Charging) - Controlled by TP4056 CHRG pin */
#define LED3_ACTIVE_LOW             true    // Active low (LOW = charging)

/* ==================== Debug Configuration ==================== */

#define DEBUG_ENABLE_LOGS       1           // Enable debug logs
#define DEBUG_I2C_SCAN          1           // Enable I2C device scanning at startup

/* Log Tags */
#define TAG_MAIN                "MAIN"
#define TAG_I2C                 "I2C"
#define TAG_OLED                "OLED"
#define TAG_DS18B20             "DS18B20"
#define TAG_MAX30102            "MAX30102"
#define TAG_MPU6050             "MPU6050"
#define TAG_BUTTON              "BUTTON"
#define TAG_LED                 "LED"
#define TAG_BATTERY             "BATTERY"
#define TAG_SENSOR_MGR          "SENSOR_MGR"

#endif // APP_CONFIG_H
