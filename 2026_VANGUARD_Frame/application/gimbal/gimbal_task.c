/**
******************************************************************************
 * @file    gimbal_task.c
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

#include "gimbal_task.h" 
#include "gimbal.h"

#include "message_center.h"

#define GIMBAL_TASK_PERIOD 1 // ms

osThreadId_t gimbal_task_handel;

static publisher_t *gimbal_publisher;
static subscriber_t *gimbal_subscriber;

static void Gimbal_Task( void *argument );

void Gimbal_Task_Init( void )
{
    const osThreadAttr_t attr = {
            .name = "Gimbal_Task",
            .stack_size = 128 * 8,
            .priority = ( osPriority_t )osPriorityRealtime4,
    };
    gimbal_task_handel = osThreadNew( Gimbal_Task, NULL, &attr );

    gimbal_publisher = Publisher_Register("gimbal_transmit_feed", sizeof(gimbal_behaviour_t));
    gimbal_subscriber = Subscriber_Register("gimbal_receive_cmd", sizeof(gimbal_cmd_t));
}

uint32_t gimbal_task_diff;

static void Gimbal_Task( void *argument )
{
    uint32_t time = osKernelGetTickCount( );

    for( ; ; )
    {

        gimbal_task_diff = osKernelGetTickCount( ) - time;
        time = osKernelGetTickCount( );
        osDelayUntil( time + GIMBAL_TASK_PERIOD );
    }
}