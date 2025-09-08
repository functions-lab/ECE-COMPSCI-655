#ifndef ESP32S3_BOX_LCD_CONFIG_H
#define ESP32S3_BOX_LCD_CONFIG_H

// Use ILI9341 LCD controller
#include "esp_lcd_ili9341.h"

// Using SPI2 in the example
#define LCD_HOST SPI2_HOST

// Please update the following configuration according to your LCD spec
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL 1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_SCLK 7
#define EXAMPLE_PIN_NUM_MOSI 6
#define EXAMPLE_PIN_NUM_MISO -1
#define EXAMPLE_PIN_NUM_LCD_DC 4
#define EXAMPLE_PIN_NUM_LCD_RST 48
#define EXAMPLE_PIN_NUM_LCD_CS 5
#define EXAMPLE_PIN_NUM_BK_LIGHT 47
#define EXAMPLE_PIN_NUM_TOUCH_CS 3

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES 320
#define EXAMPLE_LCD_V_RES 240
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS 8
#define EXAMPLE_LCD_PARAM_BITS 8

#define EXAMPLE_LVGL_DRAW_BUF_LINES                                            \
  20 // number of display lines in each draw buffer
#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1000 / CONFIG_FREERTOS_HZ
#define EXAMPLE_LVGL_TASK_STACK_SIZE (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY 2

#endif // ESP32S3_BOX_LCD_CONFIG_H
