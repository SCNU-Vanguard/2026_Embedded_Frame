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
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"

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
    Chassis_Task_Init();

    Gimbal_Task_Init();
    
    Shoot_Task_Init();
}

void Robot_Frame_Init(void)
{
    Frame_MCU_Init();

    Frame_Device_Init();

    Frame_Task_Init();
}