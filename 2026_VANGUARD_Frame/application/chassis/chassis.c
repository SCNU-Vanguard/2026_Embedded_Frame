/**
******************************************************************************
 * @file    chassis.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h>

#include "chassis.h"

#include "DM_motor.h"
#include "DJI_motor.h"

motor_init_config_t DM_right_back_motor_init = {
	.controller_param_init_config = {
		.angle_PID = NULL,
		.speed_PID = NULL,
		.current_PID = NULL,
		.torque_PID = NULL,

		.other_angle_feedback_ptr = NULL,
		.other_speed_feedback_ptr = NULL,

		.angle_feedforward_ptr = NULL,
		.speed_feedforward_ptr = NULL,
		.current_feedforward_ptr = NULL,
		.torque_feedforward_ptr = NULL,

		.pid_ref = 0.0f,
	},
	.controller_setting_init_config = {
		.close_loop_type = TORQUE_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
	},

	.motor_type = DM8009P,

	.can_init_config = {
		.can_handle = &hfdcan1,
		.tx_id = 0x00,
		.rx_id = 0x10,
	},

	.motor_control_type = TORQUE_LOOP_CONTRO,
};

motor_init_config_t DM_right_front_motor_init = {
	.controller_param_init_config = {
		.angle_PID = NULL,
		.speed_PID = NULL,
		.current_PID = NULL,
		.torque_PID = NULL,

		.other_angle_feedback_ptr = NULL,
		.other_speed_feedback_ptr = NULL,

		.angle_feedforward_ptr = NULL,
		.speed_feedforward_ptr = NULL,
		.current_feedforward_ptr = NULL,
		.torque_feedforward_ptr = NULL,

		.pid_ref = 0.0f,
	},
	.controller_setting_init_config = {
		.close_loop_type = TORQUE_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
	},

	.motor_type = DM8009P,

	.can_init_config = {
		.can_handle = &hfdcan1,
		.tx_id = 0x01,
		.rx_id = 0x11,
	},

	.motor_control_type = TORQUE_LOOP_CONTRO,
};

motor_init_config_t DM_left_front_motor_init = {
	.controller_param_init_config = {
		.angle_PID = NULL,
		.speed_PID = NULL,
		.current_PID = NULL,
		.torque_PID = NULL,

		.other_angle_feedback_ptr = NULL,
		.other_speed_feedback_ptr = NULL,

		.angle_feedforward_ptr = NULL,
		.speed_feedforward_ptr = NULL,
		.current_feedforward_ptr = NULL,
		.torque_feedforward_ptr = NULL,

		.pid_ref = 0.0f,
	},
	.controller_setting_init_config = {
		.close_loop_type = TORQUE_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
	},

	.motor_type = DM8009P,

	.can_init_config = {
		.can_handle = &hfdcan1,
		.tx_id = 0x02,
		.rx_id = 0x12,
	},

	.motor_control_type = TORQUE_LOOP_CONTRO,
};

motor_init_config_t DM_left_back_motor_init = {
	.controller_param_init_config = {
		.angle_PID = NULL,
		.speed_PID = NULL,
		.current_PID = NULL,
		.torque_PID = NULL,

		.other_angle_feedback_ptr = NULL,
		.other_speed_feedback_ptr = NULL,

		.angle_feedforward_ptr = NULL,
		.speed_feedforward_ptr = NULL,
		.current_feedforward_ptr = NULL,
		.torque_feedforward_ptr = NULL,

		.pid_ref = 0.0f,
	},
	.controller_setting_init_config = {
		.close_loop_type = TORQUE_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
	},

	.motor_type = DM8009P,

	.can_init_config = {
		.can_handle = &hfdcan1,
		.tx_id = 0x03,
		.rx_id = 0x13,
	},

	.motor_control_type = TORQUE_LOOP_CONTRO,
};

motor_init_config_t DJI_right_motor_init = {
	.controller_param_init_config = {
		.angle_PID = NULL,
		.speed_PID = NULL,
		.current_PID = NULL,
		.torque_PID = NULL,

		.other_angle_feedback_ptr = NULL,
		.other_speed_feedback_ptr = NULL,

		.angle_feedforward_ptr = NULL,
		.speed_feedforward_ptr = NULL,
		.current_feedforward_ptr = NULL,
		.torque_feedforward_ptr = NULL,

		.pid_ref = 0.0f,
	},
	.controller_setting_init_config = {
		.close_loop_type = TORQUE_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
	},

	.motor_type = M3508,

	.can_init_config = {
		.can_handle = &hfdcan3,
		.tx_id = 0x01,
		.rx_id = 0x00,
	},

	.motor_control_type = TORQUE_LOOP_CONTRO,
};

motor_init_config_t DJI_left_motor_init = {
	.controller_param_init_config = {
		.angle_PID = NULL,
		.speed_PID = NULL,
		.current_PID = NULL,
		.torque_PID = NULL,

		.other_angle_feedback_ptr = NULL,
		.other_speed_feedback_ptr = NULL,

		.angle_feedforward_ptr = NULL,
		.speed_feedforward_ptr = NULL,
		.current_feedforward_ptr = NULL,
		.torque_feedforward_ptr = NULL,

		.pid_ref = 0.0f,
	},
	.controller_setting_init_config = {
		.close_loop_type = TORQUE_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
	},

	.motor_type = M3508,

	.can_init_config = {
		.can_handle = &hfdcan3,
		.tx_id = 0x02,
		.rx_id = 0x00,
	},

	.motor_control_type = TORQUE_LOOP_CONTRO,
};

DM_motor_instance_t *DM_right_front_motor_instance;
DM_motor_instance_t *DM_right_back_motor_instance;
DM_motor_instance_t *DM_left_front_motor_instance;
DM_motor_instance_t *DM_left_back_motor_instance;
DJI_motor_instance_t *DJI_right_motor_instance;
DJI_motor_instance_t *DJI_left_motor_instance;

void Chassis_Init(void)
{
	DM_right_front_motor_instance = DM_Motor_Init(&DM_right_front_motor_init);
	DM_right_back_motor_instance  = DM_Motor_Init(&DM_right_back_motor_init);
	DM_left_front_motor_instance  = DM_Motor_Init(&DM_left_front_motor_init);
	DM_left_back_motor_instance   = DM_Motor_Init(&DM_left_back_motor_init);
	DJI_right_motor_instance      = DJI_Motor_Init(&DJI_right_motor_init);
	DJI_left_motor_instance       = DJI_Motor_Init(&DJI_left_motor_init);
}

/******************************DJI电机底盘初始化实例*****************************/

// static DJI_motor_instance_t *chassis_motor[4];

// motor_init_config_t chassis_motor_config = {
//      .controller_param_init_config = {
// 		.angle_PID = NULL,
// 		.speed_PID = NULL,
// 		.current_PID = NULL,
// 		.torque_PID = NULL,

// 		.other_angle_feedback_ptr = NULL,
// 		.other_speed_feedback_ptr = NULL,

// 		.angle_feedforward_ptr = NULL,
// 		.speed_feedforward_ptr = NULL,
// 		.current_feedforward_ptr = NULL,
// 		.torque_feedforward_ptr = NULL,

// 		.pid_ref = 0.0f,
// 	},
// 	.controller_setting_init_config = {
// 		.close_loop_type = TORQUE_LOOP,

// 		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
// 		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

// 		.angle_feedback_source = MOTOR_FEED,
// 		.speed_feedback_source = MOTOR_FEED,

// 		.feedforward_flag = FEEDFORWARD_NONE,
// 	},

// 	.motor_type = M3508,

// 	.can_init_config = {
// 		.can_handle = &hfdcan3,
// 		.tx_id = 0x00,
// 		.rx_id = 0x00,
// 	},

// 	.motor_control_type = TORQUE_LOOP_CONTRO,
// };

//   for (uint8_t i = 0; i < 4; i++)
//   {
//     chassis_motor_config.can_init_config.tx_id = i + 1;
//     chassis_motor[i] = DJI_Motor_Init(&chassis_config);
//   }

/******************************DJI电机底盘初始化实例*****************************/
