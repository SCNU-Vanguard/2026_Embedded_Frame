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
#include "daemon_task.h"
#include "INS_task.h"

#include "bmi088.h"
#include "ws2812.h"
#include "buzzer.h"

#include "bsp_dwt.h"

float init_time;

static void Frame_MCU_Init(void)
{
	DWT_Delay(480);
}

static void Frame_Device_Init(void)
{
	Buzzer_Register( );
	ws2812_instance = WS2812_Init(&ws2812_config);

	bmi088_h7 = BMI088_Register(&bmi088_init_h7);
}

static void Frame_Task_Init(void)
{
	TIME_ELAPSE(init_time, Buzzer_Task_Init( );
	Chassis_Task_Init( );
	Gimbal_Task_Init( );
	Shoot_Task_Init( );
    INS_Task_Init( );
	)
	;

	//    Buzzer_Task_Init( );

	//    Chassis_Task_Init( );

	//    Gimbal_Task_Init( );

	//    Shoot_Task_Init( );
}

void Robot_Frame_Init(void)
{
	Frame_MCU_Init( );

	Frame_Device_Init( );

	Frame_Task_Init( );
}
