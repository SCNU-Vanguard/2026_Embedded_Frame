/**
******************************************************************************
 * @file    INS.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h>

#include "INS.h" 

#include "bmi088.h"

static bmi088_data_t imu_data;

void INS_Data_Update(void)
{
 BMI088_Acquire(bmi088_h7, &imu_data);   
}