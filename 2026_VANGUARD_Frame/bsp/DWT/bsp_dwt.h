/**
 ******************************************************************************
 * @file	bsp_dwt.h
 * @author  Wang Hongxi
 * @version V1.1.0
 * @date    2022/3/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef _BSP_DWT_H
#define _BSP_DWT_H

#include <stdint.h>
#include "main.h"

typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_clock_t;

extern void DWT_Init(uint32_t CPU_Freq_mHz);

extern float DWT_GetDeltaT(uint32_t *cnt_last);
extern double DWT_GetDeltaT64(uint32_t *cnt_last);
extern float DWT_GetTimeline_s(void);
extern float DWT_GetTimeline_ms(void);
extern uint64_t DWT_GetTimeline_us(void);

extern void DWT_Delay(float Delay);

extern DWT_clock_t system_time;

#endif /* BSP_DWT_H_ */
