/**
 * @file oled.c
 * @brief SH1106 OLED Display Driver Implementation
 * @author Claude AI Assistant
 * @date 2026-01-01
 *
 * This driver supports 1.3" SH1106 OLED display (128x64 pixels) via I2C
 */

#include "oled.h"
#include "i2c_master.h"
#include "app_config.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ==================== Private Variables ==================== */
static const char *TAG = TAG_OLED;
static uint8_t oled_buffer[OLED_WIDTH * OLED_PAGES];  // Display buffer

/* ==================== 6x8 ASCII Font ==================== */
// Simple 6x8 font for ASCII characters 32-127
static const uint8_t font_6x8[][6] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // Space (32)
    {0x00, 0x00, 0x5F, 0x00, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62, 0x00}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50, 0x00}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14, 0x00}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08, 0x00}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08, 0x00}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02, 0x00}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46, 0x00}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31, 0x00}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10, 0x00}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39, 0x00}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03, 0x00}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36, 0x00}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E, 0x00}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14, 0x00}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08, 0x00}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06, 0x00}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E, 0x00}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36, 0x00}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22, 0x00}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41, 0x00}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01, 0x00}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A, 0x00}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01, 0x00}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41, 0x00}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40, 0x00}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06, 0x00}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46, 0x00}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31, 0x00}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01, 0x00}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63, 0x00}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07, 0x00}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43, 0x00}, // Z
    {0x00, 0x7F, 0x41, 0x41, 0x00, 0x00}, // [
    {0x02, 0x04, 0x08, 0x10, 0x20, 0x00}, // backslash
    {0x00, 0x41, 0x41, 0x7F, 0x00, 0x00}, // ]
    {0x04, 0x02, 0x01, 0x02, 0x04, 0x00}, // ^
    {0x40, 0x40, 0x40, 0x40, 0x40, 0x00}, // _
    {0x00, 0x01, 0x02, 0x04, 0x00, 0x00}, // `
    {0x20, 0x54, 0x54, 0x54, 0x78, 0x00}, // a
    {0x7F, 0x48, 0x44, 0x44, 0x38, 0x00}, // b
    {0x38, 0x44, 0x44, 0x44, 0x20, 0x00}, // c
    {0x38, 0x44, 0x44, 0x48, 0x7F, 0x00}, // d
    {0x38, 0x54, 0x54, 0x54, 0x18, 0x00}, // e
    {0x08, 0x7E, 0x09, 0x01, 0x02, 0x00}, // f
    {0x0C, 0x52, 0x52, 0x52, 0x3E, 0x00}, // g
    {0x7F, 0x08, 0x04, 0x04, 0x78, 0x00}, // h
    {0x00, 0x44, 0x7D, 0x40, 0x00, 0x00}, // i
    {0x20, 0x40, 0x44, 0x3D, 0x00, 0x00}, // j
    {0x7F, 0x10, 0x28, 0x44, 0x00, 0x00}, // k
    {0x00, 0x41, 0x7F, 0x40, 0x00, 0x00}, // l
    {0x7C, 0x04, 0x18, 0x04, 0x78, 0x00}, // m
    {0x7C, 0x08, 0x04, 0x04, 0x78, 0x00}, // n
    {0x38, 0x44, 0x44, 0x44, 0x38, 0x00}, // o
    {0x7C, 0x14, 0x14, 0x14, 0x08, 0x00}, // p
    {0x08, 0x14, 0x14, 0x18, 0x7C, 0x00}, // q
    {0x7C, 0x08, 0x04, 0x04, 0x08, 0x00}, // r
    {0x48, 0x54, 0x54, 0x54, 0x20, 0x00}, // s
    {0x04, 0x3F, 0x44, 0x40, 0x20, 0x00}, // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00}, // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00}, // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00}, // w
    {0x44, 0x28, 0x10, 0x28, 0x44, 0x00}, // x
    {0x0C, 0x50, 0x50, 0x50, 0x3C, 0x00}, // y
    {0x44, 0x64, 0x54, 0x4C, 0x44, 0x00}, // z
    {0x00, 0x08, 0x36, 0x41, 0x00, 0x00}, // {
    {0x00, 0x00, 0x7F, 0x00, 0x00, 0x00}, // |
    {0x00, 0x41, 0x36, 0x08, 0x00, 0x00}, // }
    {0x08, 0x04, 0x08, 0x10, 0x08, 0x00}, // ~
};

/* ==================== Private Function Prototypes ==================== */
static esp_err_t oled_write_command(uint8_t cmd);
static esp_err_t oled_write_data(uint8_t data);
static void oled_set_position(uint8_t x, uint8_t page);

/* ==================== Private Functions ==================== */

/**
 * @brief Write a command to OLED
 */
static esp_err_t oled_write_command(uint8_t cmd)
{
    uint8_t buffer[2] = {0x00, cmd};  // 0x00 = command mode
    return i2c_write_data(OLED_I2C_ADDRESS, buffer, 2);
}

/**
 * @brief Write data to OLED
 */
static esp_err_t oled_write_data(uint8_t data)
{
    uint8_t buffer[2] = {0x40, data};  // 0x40 = data mode
    return i2c_write_data(OLED_I2C_ADDRESS, buffer, 2);
}

/**
 * @brief Set cursor position (for SH1106)
 */
static void oled_set_position(uint8_t x, uint8_t page)
{
    // SH1106 has 2-pixel column offset
    x += OLED_COLUMN_OFFSET;

    oled_write_command(OLED_CMD_SET_PAGE_ADDR | page);           // Set page address
    oled_write_command(OLED_CMD_SET_LOW_COLUMN | (x & 0x0F));   // Set lower column address
    oled_write_command(OLED_CMD_SET_HIGH_COLUMN | (x >> 4));    // Set higher column address
}

/* ==================== Public Functions ==================== */

/**
 * @brief Initialize OLED display
 */
esp_err_t oled_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing SH1106 OLED display...");

    // Wait for display to power up
    vTaskDelay(pdMS_TO_TICKS(100));

    // Turn off display
    ret = oled_write_command(OLED_CMD_DISPLAY_OFF);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to turn off display");
        return ret;
    }

    // Set display clock divide ratio/oscillator frequency
    oled_write_command(OLED_CMD_SET_DISPLAY_CLOCK_DIV);
    oled_write_command(0x80);  // Default ratio

    // Set multiplex ratio (1 to 64)
    oled_write_command(OLED_CMD_SET_MULTIPLEX);
    oled_write_command(0x3F);  // 1/64 duty

    // Set display offset
    oled_write_command(OLED_CMD_SET_DISPLAY_OFFSET);
    oled_write_command(0x00);  // No offset

    // Set start line address
    oled_write_command(OLED_CMD_SET_START_LINE | 0x00);  // Start line = 0

    // Enable charge pump regulator (required for SH1106)
    oled_write_command(OLED_CMD_CHARGE_PUMP);
    oled_write_command(0x14);  // Enable

    // Set segment re-map (flip horizontally)
    oled_write_command(OLED_CMD_SEG_REMAP);  // Column address 127 mapped to SEG0

    // Set COM output scan direction (flip vertically)
    oled_write_command(OLED_CMD_COM_SCAN_DEC);  // Remapped mode. Scan from COM[N-1] to COM0

    // Set COM pins hardware configuration
    oled_write_command(OLED_CMD_SET_COM_PINS);
    oled_write_command(0x12);  // Alternative COM pin configuration

    // Set contrast control
    oled_write_command(OLED_CMD_SET_CONTRAST);
    oled_write_command(0x7F);  // Medium brightness (0-255)

    // Set pre-charge period
    oled_write_command(OLED_CMD_SET_PRECHARGE);
    oled_write_command(0xF1);  // Phase 1: 15 clocks, Phase 2: 1 clock

    // Set VCOMH deselect level
    oled_write_command(OLED_CMD_SET_VCOMH_DESELECT);
    oled_write_command(0x40);  // ~0.77 x Vcc

    // Entire display ON (resume to RAM content display)
    oled_write_command(0xA4);

    // Set normal display (not inverted)
    oled_write_command(OLED_CMD_NORMAL_DISPLAY);

    // Clear display buffer
    memset(oled_buffer, 0, sizeof(oled_buffer));
    oled_refresh();

    // Turn on display
    ret = oled_write_command(OLED_CMD_DISPLAY_ON);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to turn on display");
        return ret;
    }

    ESP_LOGI(TAG, "OLED initialized successfully");
    return ESP_OK;
}

/**
 * @brief Clear the entire display
 */
void oled_clear(void)
{
    memset(oled_buffer, 0, sizeof(oled_buffer));
    oled_refresh();
}

/**
 * @brief Refresh the display (write buffer to OLED)
 */
void oled_refresh(void)
{
    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        oled_set_position(0, page);

        // Write one page of data
        for (uint8_t col = 0; col < OLED_WIDTH; col++) {
            oled_write_data(oled_buffer[page * OLED_WIDTH + col]);
        }
    }
}

/**
 * @brief Display a single character
 */
void oled_show_char(uint8_t x, uint8_t y, char chr)
{
    if (chr < 32 || chr > 126) {
        chr = '?';  // Unknown character
    }

    uint8_t char_index = chr - 32;

    // Write character to buffer
    for (uint8_t i = 0; i < FONT_WIDTH; i++) {
        if (x + i < OLED_WIDTH) {
            oled_buffer[y * OLED_WIDTH + x + i] = font_6x8[char_index][i];
        }
    }
}

/**
 * @brief Display a string
 */
void oled_show_string(uint8_t x, uint8_t y, const char *text)
{
    uint8_t pos_x = x;

    while (*text != '\0') {
        if (pos_x + FONT_WIDTH > OLED_WIDTH) {
            break;  // String too long for display
        }

        oled_show_char(pos_x, y, *text);
        pos_x += FONT_WIDTH;
        text++;
    }
}

/**
 * @brief Display a number
 */
void oled_show_number(uint8_t x, uint8_t y, uint32_t num, uint8_t len)
{
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%0*lu", len, (unsigned long)num);
    oled_show_string(x, y, buffer);
}

/**
 * @brief Display a floating-point number
 */
void oled_show_float(uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t frac_len)
{
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%*.*f", int_len + frac_len + 1, frac_len, num);
    oled_show_string(x, y, buffer);
}

/**
 * @brief Set a single pixel
 */
void oled_draw_pixel(uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) {
        return;  // Out of bounds
    }

    uint8_t page = y / 8;
    uint8_t bit = y % 8;

    if (color) {
        oled_buffer[page * OLED_WIDTH + x] |= (1 << bit);
    } else {
        oled_buffer[page * OLED_WIDTH + x] &= ~(1 << bit);
    }
}

/**
 * @brief Draw a horizontal line
 */
void oled_draw_hline(uint8_t x, uint8_t y, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        oled_draw_pixel(x + i, y, 1);
    }
}

/**
 * @brief Draw a vertical line
 */
void oled_draw_vline(uint8_t x, uint8_t y, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        oled_draw_pixel(x, y + i, 1);
    }
}

/**
 * @brief Draw a rectangle outline
 */
void oled_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    oled_draw_hline(x, y, w);
    oled_draw_hline(x, y + h - 1, w);
    oled_draw_vline(x, y, h);
    oled_draw_vline(x + w - 1, y, h);
}

/**
 * @brief Draw a filled rectangle
 */
void oled_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    for (uint8_t i = 0; i < h; i++) {
        oled_draw_hline(x, y + i, w);
    }
}

/**
 * @brief Set display brightness
 */
void oled_set_brightness(uint8_t brightness)
{
    oled_write_command(OLED_CMD_SET_CONTRAST);
    oled_write_command(brightness);
}

/**
 * @brief Turn display on/off
 */
void oled_set_display(bool on)
{
    if (on) {
        oled_write_command(OLED_CMD_DISPLAY_ON);
    } else {
        oled_write_command(OLED_CMD_DISPLAY_OFF);
    }
}

/**
 * @brief Invert display colors
 */
void oled_set_invert(bool invert)
{
    if (invert) {
        oled_write_command(OLED_CMD_INVERSE_DISPLAY);
    } else {
        oled_write_command(OLED_CMD_NORMAL_DISPLAY);
    }
}
