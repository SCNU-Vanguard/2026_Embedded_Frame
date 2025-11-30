/**
******************************************************************************
 * @file    procotol.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h>

/******************************procotol处为联系各处的纽带，包含各种需要的头文件*****************************/

#include "procotol.h"
#include "INS.h"
#include "shoot.h"
#include "gimbal.h"
#include "chassis.h"

#include "vofa.h"
#include "bmi088.h"
#include "wfly_control.h"

#include "balance_chassis.h"

extern bmi088_data_t imu_data;
extern wfly_t *rc_data;
extern INS_behaviour_t INS;
extern float test_data_acc[3];
extern float test_data_gyro[3];

void VOFA_Display_IMU(void)
{
	// vofa_data_view[0] = imu_data.acc[0];
	// vofa_data_view[1] = imu_data.acc[1];
	// vofa_data_view[2] = imu_data.acc[2];

	// vofa_data_view[3] = test_data_acc[0];
	// vofa_data_view[4] = test_data_acc[1];
	// vofa_data_view[5] = test_data_acc[2];

	// vofa_data_view[6] = imu_data.gyro[0];
	// vofa_data_view[7] = imu_data.gyro[1];
	// vofa_data_view[8] = imu_data.gyro[2];

	// vofa_data_view[9]  = test_data_gyro[0];
	// vofa_data_view[10] = test_data_gyro[1];
	// vofa_data_view[11] = test_data_gyro[2];
	
	// vofa_data_view[0] = balance_chassis.wheel_motor[0]->motor_controller.pid_ref;
	// vofa_data_view[1] = balance_chassis.wheel_motor[1]->motor_controller.pid_ref;
	// vofa_data_view[2] = balance_chassis.wheel_motor[0]->receive_data.real_current;
	// vofa_data_view[3] = balance_chassis.wheel_motor[1]->receive_data.real_current;

	// vofa_data_view[0] = balance_chassis.v_filter;
	// vofa_data_view[1] = balance_chassis.v_set;
	// vofa_data_view[2] = balance_chassis.x_filter;
	// vofa_data_view[3] = balance_chassis.x_set;

	vofa_data_view[0] = balance_chassis.v_filter;
	vofa_data_view[1] = balance_chassis.wheel_motor[1]->receive_data.speed_rps * 2.0f * PI * WHEEL_RADIUS;
	vofa_data_view[2] = -INS.MotionAccel_b[1];
	vofa_data_view[3] = chassis_err_left_test[0];

	vofa_data_view[4] = chassis_err_right_test[0];
	vofa_data_view[5] = chassis_err_left_test[1];
	vofa_data_view[6] = chassis_err_right_test[1];
	vofa_data_view[7] = chassis_err_left_test[2];
	vofa_data_view[8] = chassis_err_right_test[2];
	vofa_data_view[9] = chassis_err_left_test[3];
	vofa_data_view[10] = chassis_err_right_test[3];
	vofa_data_view[11] = chassis_err_left_test[4];
	vofa_data_view[12] = chassis_err_right_test[4];
	vofa_data_view[13] = chassis_err_left_test[5];
	vofa_data_view[14] = chassis_err_right_test[5];

	// vofa_data_view[4] = chassis_right_leg.wheel_torque;
	// vofa_data_view[5] = chassis_left_leg.wheel_torque;

	// vofa_data_view[6] = chassis_left_leg.F_N;
	// vofa_data_view[7] = chassis_right_leg.F_N;

	// vofa_data_view[4] = INS.Accel[2];
	// vofa_data_view[5] = INS.Accel[1];
	// vofa_data_view[6] = INS.MotionAccel_b[1];
	// vofa_data_view[7] = INS.MotionAccel_b[2];

	// vofa_data_view[8]  = chassis_left_leg.p;
	// vofa_data_view[9]  = chassis_right_leg.p;
	// vofa_data_view[10] = chassis_left_leg.dd_z_wheel;
	// vofa_data_view[11] = chassis_right_leg.dd_z_wheel;

	// vofa_data_view[8]  = chassis_left_leg.back_joint_torque;
	// vofa_data_view[9]  = chassis_right_leg.back_joint_torque;
	// vofa_data_view[10] = chassis_left_leg.front_joint_torque;
	// vofa_data_view[11] = chassis_right_leg.front_joint_torque;

	// vofa_data_view[8] = balance_chassis.turn_T;
	// vofa_data_view[9] = balance_chassis.total_yaw;
	// vofa_data_view[10] = balance_chassis.turn_set;
	// vofa_data_view[11] = INS.Gyro[2];

	// vofa_data_view[11] = bmi088_h7->heat_pwm->dutyratio;
	// vofa_data_view[10] = INS.x_n;
	// vofa_data_view[11] = INS.v_n;

	// vofa_data_view[12] = INS.Pitch;
	// vofa_data_view[13] = INS.Roll;
	// vofa_data_view[14] = INS.Yaw;
	
	VOFA_Send_Data(vofa_data_view, 15);
	// VOFA_JustFloat(vofa_data_view, 7);
}

void RC_Receive_Control(void)
{
	
}
