/**
******************************************************************************
 * @file    robot_frame_init.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include "robot_frame_init.h"

#include "robot_frame_config.h"

#include "bsp_dwt.h"

static void Frame_MCU_Init(void)
{
    DWT_Delay(480);
}

static void Frame_Device_Init(void)
{
    ;
}

static void Frame_Task_Init(void)
{
    ;
}

void Robot_Frame_Init(void)
{
    Frame_MCU_Init();

    Frame_Device_Init();

    Frame_Task_Init();
}