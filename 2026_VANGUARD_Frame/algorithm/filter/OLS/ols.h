/**
* @file ols.h
 * @author guatai (2508588132@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __OLS_H__
#define __OLS_H__

#include <stdint.h>
#include <stdbool.h>

#include "user_lib.h"

typedef struct
{
	uint16_t Order;
	uint32_t Count;

	float *x;
	float *y;

	float k;
	float b;

	float StandardDeviation;

	float t[4];
} __attribute__((__packed__)) Ordinary_Least_Squares_t;

void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order);

void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y);

float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y);

float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y);

float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS);

float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS);

#endif /* __OLS_H__ */