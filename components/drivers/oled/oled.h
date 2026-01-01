/**
 * @file oled.h
 * @brief SH1106 OLED Display Driver Header
 * @author larythks
 * @date 2026-01-01
 *
 * This driver supports 1.3" SH1106 OLED display (128x64 pixels) via I2C
 */

#ifndef OLED_H
#define OLED_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ==================== Display Specifications ==================== */
#define OLED_WIDTH              128         // Display width in pixels
#define OLED_HEIGHT             64          // Display height in pixels
#define OLED_PAGES              8           // Number of pages (64/8=8)
#define OLED_COLUMN_OFFSET      2           // SH1106 has 2-pixel offset

/* ==================== SH1106 Commands ==================== */
#define OLED_CMD_DISPLAY_OFF            0xAE
#define OLED_CMD_DISPLAY_ON             0xAF
#define OLED_CMD_SET_CONTRAST           0x81
#define OLED_CMD_NORMAL_DISPLAY         0xA6
#define OLED_CMD_INVERSE_DISPLAY        0xA7
#define OLED_CMD_SET_DISPLAY_OFFSET     0xD3
#define OLED_CMD_SET_COM_PINS           0xDA
#define OLED_CMD_SET_VCOMH_DESELECT     0xDB
#define OLED_CMD_SET_DISPLAY_CLOCK_DIV  0xD5
#define OLED_CMD_SET_PRECHARGE          0xD9
#define OLED_CMD_SET_MULTIPLEX          0xA8
#define OLED_CMD_SET_LOW_COLUMN         0x00
#define OLED_CMD_SET_HIGH_COLUMN        0x10
#define OLED_CMD_SET_START_LINE         0x40
#define OLED_CMD_MEMORY_MODE            0x20
#define OLED_CMD_COLUMN_ADDR            0x21
#define OLED_CMD_PAGE_ADDR              0x22
#define OLED_CMD_COM_SCAN_DEC           0xC8
#define OLED_CMD_SEG_REMAP              0xA1
#define OLED_CMD_CHARGE_PUMP            0x8D
#define OLED_CMD_ACTIVATE_SCROLL        0x2F
#define OLED_CMD_DEACTIVATE_SCROLL      0x2E
#define OLED_CMD_SET_PAGE_ADDR          0xB0

/* ==================== Font Configuration ==================== */
#define FONT_WIDTH              6           // 6x8 font width
#define FONT_HEIGHT             8           // 6x8 font height

/* ==================== Public API ==================== */

/**
 * @brief Initialize OLED display
 *
 * This function initializes the SH1106 OLED display via I2C.
 * It performs the following steps:
 * 1. Turn off display
 * 2. Configure display settings (contrast, multiplex, etc.)
 * 3. Enable charge pump
 * 4. Clear display buffer
 * 5. Turn on display
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_FAIL: Initialization failed
 */
esp_err_t oled_init(void);

/**
 * @brief Clear the entire display
 *
 * Fills the display buffer with zeros (all pixels off) and updates the screen.
 */
void oled_clear(void);

/**
 * @brief Display a string at specified position
 *
 * @param x X-coordinate (0-127)
 * @param y Y-coordinate (0-7, page number)
 * @param text Null-terminated string to display
 */
void oled_show_string(uint8_t x, uint8_t y, const char *text);

/**
 * @brief Display a number at specified position
 *
 * @param x X-coordinate (0-127)
 * @param y Y-coordinate (0-7, page number)
 * @param num Number to display
 * @param len Number of digits to display (pad with zeros if needed)
 */
void oled_show_number(uint8_t x, uint8_t y, uint32_t num, uint8_t len);

/**
 * @brief Display a floating-point number at specified position
 *
 * @param x X-coordinate (0-127)
 * @param y Y-coordinate (0-7, page number)
 * @param num Floating-point number to display
 * @param int_len Number of integer digits
 * @param frac_len Number of fractional digits
 */
void oled_show_float(uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t frac_len);

/**
 * @brief Display a single character at specified position
 *
 * @param x X-coordinate (0-127)
 * @param y Y-coordinate (0-7, page number)
 * @param chr Character to display (ASCII 32-127)
 */
void oled_show_char(uint8_t x, uint8_t y, char chr);

/**
 * @brief Set a single pixel
 *
 * @param x X-coordinate (0-127)
 * @param y Y-coordinate (0-63)
 * @param color 1=on, 0=off
 */
void oled_draw_pixel(uint8_t x, uint8_t y, uint8_t color);

/**
 * @brief Draw a horizontal line
 *
 * @param x X-coordinate start
 * @param y Y-coordinate
 * @param len Length of the line
 */
void oled_draw_hline(uint8_t x, uint8_t y, uint8_t len);

/**
 * @brief Draw a vertical line
 *
 * @param x X-coordinate
 * @param y Y-coordinate start
 * @param len Length of the line
 */
void oled_draw_vline(uint8_t x, uint8_t y, uint8_t len);

/**
 * @brief Draw a rectangle outline
 *
 * @param x X-coordinate (top-left corner)
 * @param y Y-coordinate (top-left corner)
 * @param w Width
 * @param h Height
 */
void oled_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

/**
 * @brief Draw a filled rectangle
 *
 * @param x X-coordinate (top-left corner)
 * @param y Y-coordinate (top-left corner)
 * @param w Width
 * @param h Height
 */
void oled_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

/**
 * @brief Refresh the display
 *
 * Transfers the display buffer to the OLED hardware.
 * Call this after drawing operations to update the screen.
 */
void oled_refresh(void);

/**
 * @brief Set display brightness
 *
 * @param brightness Brightness level (0-255)
 */
void oled_set_brightness(uint8_t brightness);

/**
 * @brief Turn display on/off
 *
 * @param on true=on, false=off
 */
void oled_set_display(bool on);

/**
 * @brief Invert display colors
 *
 * @param invert true=inverted, false=normal
 */
void oled_set_invert(bool invert);

#endif // OLED_H
