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

#include "procotol.h"

#include "vofa.h"
#include "bmi088.h"
#include "wfly_control.h"

extern bmi088_data_t imu_data;
extern wfly_t *rc_data;

void VOFA_Display_IMU(void)
{
	vofa_data_view[0] = imu_data.acc[0];
	vofa_data_view[1] = imu_data.acc[1];
	vofa_data_view[2] = imu_data.acc[2];

	vofa_data_view[3] = imu_data.gyro[0];
	vofa_data_view[4] = imu_data.gyro[1];
	vofa_data_view[5] = imu_data.gyro[2];

	vofa_data_view[6] = imu_data.temperature;

	//VOFA_Send_Data(vofa_data_view, 7);
	VOFA_JustFloat(vofa_data_view, 7);
}

void RC_Receive_Control(void)
{
	// switch (rc_data->toggle.SA)
	// {
	// 	case RC_SW_UP:
	// 		if (rc_data->rc.ch[RC_CH_RY])
	// 		{
	// 			chassis_move.leg_set += (float) rc_data->rc.ch[RC_CH_RY] * 0.000005f; // 调腿长
	// 			chassis_move.leg_length_change_flag = 1;
	// 		}
	// 		else
	// 		{
	// 			chassis_move.leg_length_change_flag = 0;
	// 		}

	// 		if (chassis_move.leg_set > leg_max_set)
	// 		{
	// 			chassis_move.leg_set = leg_max_set;
	// 		}
	// 		else if (chassis_move.leg_set < leg_min_set)
	// 		{
	// 			chassis_move.leg_set = leg_min_set;
	// 		}

	// 		if (rc_data->toggle.SC == RC_SW_MID)
	// 		{
	// 			chassis_move.leg_set = leg_init_set;
	// 		}
	// 		break;
	// 	case RC_SW_MID:
	// 		chassis_move.v_set = ramp_calc(chassis_wheel_ramp_speed,
	// 		                               ((float) rc_data->rc.ch[RC_CH_RY] *
	// 		                                0.0025f));

	// 		chassis_move.x_set = 0.0f;

	// 		if (user_abs(chassis_move.wz_set) < 0.025f)
	// 		{
	// 			chassis_move.wz_set = 0.0f;
	// 		}

	// 		if (chassis_move.wz_set < -0.6f)
	// 		{
	// 			chassis_move.wz_set = -0.6f;
	// 		}
	// 		else if (chassis_move.wz_set > 0.6f)
	// 		{
	// 			chassis_move.wz_set = 0.6f;
	// 		}

	// 		if (chassis_move.wz_set != 0)
	// 		{
	// 			chassis_move.turn_set = chassis_move.total_yaw + chassis_move.wz_set;
	// 		}
	// 		break;

	// 	case RC_SW_DOWN:
	// 		switch (rc_data->toggle.SC)
	// 		{
	// 			case RC_SW_UP:

	// 				break;
	// 			case RC_SW_MID:
	// 				DM_Motor_DISABLE(&hfdcan1,
	// 				                 chassis_move.joint_motor[2].para.id,
	// 				                 chassis_move.joint_motor[2]
	// 				                 .mode);
	// 				DM_Motor_DISABLE(&hfdcan1,
	// 				                 chassis_move.joint_motor[3].para.id,
	// 				                 chassis_move.joint_motor[3]
	// 				                 .mode);
	// 				DM_Motor_DISABLE(&hfdcan1,
	// 				                 chassis_move.joint_motor[0].para.id,
	// 				                 chassis_move.joint_motor[0]
	// 				                 .mode);
	// 				DM_Motor_DISABLE(&hfdcan1,
	// 				                 chassis_move.joint_motor[1].para.id,
	// 				                 chassis_move.joint_motor[1]
	// 				                 .mode);
	// 				break;
	// 			case RC_SW_DOWN:
	// 				DM_Motor_ENABLE(&hfdcan1,
	// 				                chassis_move.joint_motor[2].para.id,
	// 				                chassis_move.joint_motor[2]
	// 				                .mode);
	// 				DM_Motor_ENABLE(&hfdcan1,
	// 				                chassis_move.joint_motor[3].para.id,
	// 				                chassis_move.joint_motor[3]
	// 				                .mode);
	// 				DM_Motor_ENABLE(&hfdcan1,
	// 				                chassis_move.joint_motor[0].para.id,
	// 				                chassis_move.joint_motor[0]
	// 				                .mode);
	// 				DM_Motor_ENABLE(&hfdcan1,
	// 				                chassis_move.joint_motor[1].para.id,
	// 				                chassis_move.joint_motor[1]
	// 				                .mode);
	// 				break;
	// 			default:
	// 				break;
	// 		}
	// 		break;
	// 	default:
	// 		break;
	//}

	//		if(rc_data->toggle.SA != RC_SW_DOWN)
	//		{
	//			switch( rc_data->toggle.SC )
	//			{
	//				case RC_SW_UP:
	//					chassis_move.leg_control_flag = 0;
	//					break;
	//				case RC_SW_MID:
	//					chassis_move.leg_control_flag = 1;
	//					break;
	//				case RC_SW_DOWN:
	//					chassis_move.leg_control_flag = 1;
	//					break;
	//				default:
	//					break;
	//			}
	//		}
	//		switch( rc_data->toggle.SC )
	//		{
	//			case RC_SW_DOWN:
	//				rc_data->rc.ch[RC_CH_LX] += 4;
	//				gimbal_motor_tar_pos[0] -= ( float )rc_data->rc.ch[RC_CH_LX] * 0.00001f;
	//				if( gimbal_motor_tar_pos[0] > PI / 2 )
	//				{
	//					gimbal_motor_tar_pos[0] = PI / 2;
	//				}
	//				else if( gimbal_motor_tar_pos[0] < -PI / 2 )
	//				{
	//					gimbal_motor_tar_pos[0] = -PI / 2;
	//				}

	//				gimbal_motor_tar_pos[1] = map_range( ( float )rc_data->rc.ch[RC_CH_LY], -671, 671, -0.6, 1 );
	//				switch( rc_data->toggle.SD )
	//				{
	//					case RC_SW_UP:
	//						break;
	////            case RC_SW_MID:
	////                Power_OUT1_ON;
	////                break;
	////            case RC_SW_DOWN:
	////                Power_OUT1_OFF;
	////                break;
	//					default:
	//						break;
	//				}
	//				break;
	//			case RC_SW_MID:
	//				gimbal_motor_tar_pos[0] = map_range( pc_rx_communicate.yaw, -1, 1, -PI / 2, PI / 2 );
	//				gimbal_motor_tar_pos[1] = map_range( pc_rx_communicate.pitch, -1, 1, -0.6, 0.6 );
	////            if (pc_rx_communicate.shoot)
	////                Power_OUT1_ON;
	////            else
	////                Power_OUT1_OFF;
	//				break;
	//		}
}
