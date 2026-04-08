#pragma once

#include "driver/gpio.h"
#include "esp_heap_caps.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_LCD_H_RES                   240
#define APP_LCD_V_RES                   240
#define APP_RGB565_BITS_PER_PIXEL       16

#define EXAMPLE_LCD_SPI_NUM             SPI3_HOST
#define EXAMPLE_LCD_CMD_BITS            8
#define EXAMPLE_LCD_PARAM_BITS          8

#define EXAMPLE_LCD_SPI_MOSI            GPIO_NUM_47
#define EXAMPLE_LCD_SPI_CLK             GPIO_NUM_21
#define EXAMPLE_LCD_SPI_CS              GPIO_NUM_44
#define EXAMPLE_LCD_DC                  GPIO_NUM_43
#define EXAMPLE_LCD_RST                 GPIO_NUM_NC
#define EXAMPLE_LCD_BACKLIGHT           GPIO_NUM_48
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ      (20 * 1000 * 1000)

#define EXAMPLE_SENSOR_I2C_SDA_IO       4
#define EXAMPLE_SENSOR_I2C_SCL_IO       5

#define EXAMPLE_BUTTON_GPIO             GPIO_NUM_0
#define EXAMPLE_BUTTON_ACTIVE_LEVEL     0

#if CONFIG_SPIRAM
#define APP_FRAMEBUF_ALLOC_CAPS         (MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA)
#else
#define APP_FRAMEBUF_ALLOC_CAPS         (MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA)
#endif

#ifdef __cplusplus
}
#endif
