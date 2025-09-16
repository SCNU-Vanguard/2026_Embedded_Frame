/**
******************************************************************************
 * @file    chassis_task.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h> 

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#include "chassis_task.h" 
#include "chassis.h"

#include "message_center.h"

#include "DM_motor.h"

#define CHASSIS_TASK_PERIOD 1 // ms

osThreadId_t robot_cmd_task_handel;

static publisher_t *chassis_publisher;
static subscriber_t *chassis_subscriber;

static void Chassis_Task( void *argument );

void Chassis_Task_Init( void )
{
	const osThreadAttr_t attr = {
			.name = "Chassis_Task",
			.stack_size = 128 * 8,
			.priority = ( osPriority_t )osPriorityRealtime4,
	};
	robot_cmd_task_handel = osThreadNew( Chassis_Task, NULL, &attr );

	chassis_publisher = Publisher_Register("chassis_transmit_feed", sizeof(chassis_behaviour_t));
	chassis_subscriber = Subscriber_Register("chassis_receive_cmd", sizeof(chassis_cmd_t));
	
}

uint32_t chassis_task_diff;

static void Chassis_Task( void *argument )
{
	uint32_t time = osKernelGetTickCount( );

	for( ; ; )
	{
		
		DM_Motor_DISABLE(NULL);
		
		chassis_task_diff = osKernelGetTickCount( ) - time;
		time = osKernelGetTickCount( );
		osDelayUntil( time + CHASSIS_TASK_PERIOD );
	}
}