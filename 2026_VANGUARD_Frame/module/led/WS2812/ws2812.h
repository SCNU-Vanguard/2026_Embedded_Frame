/**
* @file ws2812.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _WS2812_H__
#define _WS2812_H__

#include <stdint.h>
#include "bsp_spi.h"

typedef struct
{
    SPI_t *ws2812_spi;
}ws2812_instance_t;

typedef struct
{
    spi_init_config_t ws2812_spi_config;
}ws2812_init_config_t;

extern ws2812_init_config_t *ws2812_config;
extern ws2812_instance_t *ws2812_instance;

ws2812_instance_t *WS2812_Init(ws2812_init_config_t *config);
void WS2812_Control(ws2812_instance_t *ws2812, uint8_t r, uint8_t g, uint8_t b);

#endif /* WS2812_H__ */
