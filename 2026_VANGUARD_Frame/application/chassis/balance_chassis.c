/**
******************************************************************************
 * @file    balance_chassis.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include "balance_chassis.h"

#if (CHASSIS_TYPE == CHASSIS_BALANCE)

#include "INS.h"
#include "digital_pid.h"
#include "kalman_filter.h"
#include "wfly_control.h"
#include "user_lib.h"
#include "lms.h"

#include "bsp_dwt.h"

const float half_MG_real = 35.5152f;//（此用3.624kg）5.93kg * 9.8m/s^2
const float MG_FF        = 8.8788f;//1/4

float F_0_forward_test;
float F_N_forward_test;
float chassis_err_left_test[6]  = {0.0f};
float chassis_err_right_test[6] = {0.0f};

float Poly_Coefficient_Speed[12][4] = {

	{-150.9391, 199.1542, -150.0745, 1.4630},
	{6.4359, -7.4437, -17.6610, 0.4634},
	{-81.8249, 83.0131, -29.8361, -2.5931},
	{-119.2550, 123.1879, -48.2221, -3.9030},
	{-177.4336, 215.9923, -104.3637, 26.3289},
	{-10.7630, 14.9249, -8.3502, 2.9954},
	{342.1236, -318.5390, 85.8589, 15.4577},
	{48.4916, -52.0475, 20.8012, 1.1576},
	{-158.7641, 187.8003, -86.8108, 19.4387},
	{-216.3937, 258.1695, -121.2425, 28.5768},
	{807.9336, -827.4131, 302.8047, 13.6805},
	{96.3176, -100.7392, 38.2987, -1.2835},
	// {-211.1604, 262.4733, -142.3981, 0.4066},
	// {3.4911, -1.5272, -9.4294, 0.3226},
	// {-35.4574, 38.2174, -14.8582, -0.2048},
	// {-56.7793, 61.7068, -25.0284, -0.3671},
	// {-25.5730, 42.9355, -28.1365, 8.9481},
	// {-3.5400, 7.1552, -5.3137, 1.9401},
	// {238.2745, -215.5877, 51.8643, 9.2288},
	// {23.4550, -25.7294, 9.9475, 0.2743},
	// {-17.5001, 27.4665, -16.9699, 4.9790},
	// {-26.1363, 41.9664, -26.5049, 8.0921},
	// {180.6515, -195.8207, 76.9185, 0.0872},
	// {38.3391, -41.9819, 16.7751, -0.2964}

};

digital_PID_t leg_l_length_pid; // 左腿的腿长pd
digital_PID_t leg_r_length_pid; // 右腿的腿长pd

digital_PID_t leg_roll_position_pid; // 横滚角补偿pd
digital_PID_t leg_roll_speed_pid; // 横滚角补偿pd

digital_PID_t leg_Tp_pid;   // 防劈叉补偿pd
digital_PID_t leg_turn_pid; // 转向pd

/******************************底盘电机初始化结构体*****************************/

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
		.control_button = TORQUE_DIRECT_CONTROL,
	},

	.motor_type = DM8009P,

	.can_init_config = {
		.can_handle = &hfdcan1,
		.tx_id = 0x00,
		.rx_id = 0x10,
	},

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
		.control_button = TORQUE_DIRECT_CONTROL,
	},

	.motor_type = DM8009P,

	.can_init_config = {
		.can_handle = &hfdcan1,
		.tx_id = 0x01,
		.rx_id = 0x11,
	},

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
		.control_button = TORQUE_DIRECT_CONTROL,
	},

	.motor_type = DM8009P,

	.can_init_config = {
		.can_handle = &hfdcan1,
		.tx_id = 0x02,
		.rx_id = 0x12,
	},

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
		.control_button = TORQUE_DIRECT_CONTROL,
	},

	.motor_type = DM8009P,

	.can_init_config = {
		.can_handle = &hfdcan1,
		.tx_id = 0x03,
		.rx_id = 0x13,
	},
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
		.control_button = TORQUE_DIRECT_CONTROL,
	},

	.motor_type = M3508,

	.can_init_config = {
		.can_handle = &hfdcan3,
		.tx_id = 0x01,
		.rx_id = 0x00,
	},

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
		.control_button = TORQUE_DIRECT_CONTROL,
	},

	.motor_type = M3508,

	.can_init_config = {
		.can_handle = &hfdcan3,
		.tx_id = 0x02,
		.rx_id = 0x00,
	},

};

/******************************底盘电机初始化结构体*****************************/

/******************************底盘电机结构体*****************************/

DM_motor_instance_t *DM_right_front_motor_instance;
DM_motor_instance_t *DM_right_back_motor_instance;
DM_motor_instance_t *DM_left_front_motor_instance;
DM_motor_instance_t *DM_left_back_motor_instance;
DJI_motor_instance_t *DJI_right_motor_instance;
DJI_motor_instance_t *DJI_left_motor_instance;

/******************************底盘电机结构体*****************************/

/******************************底盘PID结构体*****************************/

digital_PID_t leg_l_length_pid; // 左腿的腿长pd
digital_PID_t leg_r_length_pid; // 右腿的腿长pd

digital_PID_t leg_roll_position_pid; // 横滚角补偿pd
digital_PID_t leg_Tp_pid;   // 防劈叉补偿pd
digital_PID_t leg_turn_pid; // 转向pd

/******************************底盘PID结构体*****************************/

/******************************底盘模型结构体*****************************/

vmc_leg_t chassis_right_leg;
vmc_leg_t chassis_left_leg;

balance_chassis_t balance_chassis;

/******************************底盘模型结构体*****************************/

/******************************底盘滤波器初始化*****************************/
ramp_init_config_t left_leg_F_0_ramp_init = {
	.decrease_value = 1.0f,
	.increase_value = 1.0f,
	.frame_period = 0.001f,
	.max_value = 0.0f,
	.min_value = 0.0f,
	.ramp_state = SLOPE_FIRST_REAL,
};

ramp_init_config_t right_leg_F_0_ramp_init = {
	.decrease_value = 1.0f,
	.increase_value = 1.0f,
	.frame_period = 0.001f,
	.max_value = 0.0f,
	.min_value = 0.0f,
	.ramp_state = SLOPE_FIRST_REAL,
};

ramp_function_source_t *left_leg_F_0_ramp;
ramp_function_source_t *right_leg_F_0_ramp;

nlms_t left_leg_F_N_nlms;
nlms_t right_leg_F_N_nlms;

KalmanFilter_t vaEstimateKF; // 卡尔曼滤波器结构体
/******************************底盘滤波器初始化*****************************/

extern INS_behaviour_t INS;
extern wfly_t *rc_data;

static void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);

void Chassis_Init(void)
{
	/******************************底盘电机初始化*****************************/
	DM_right_back_motor_instance  = DM_Motor_Init(&DM_right_back_motor_init);
	DM_right_front_motor_instance = DM_Motor_Init(&DM_right_front_motor_init);
	DM_left_front_motor_instance  = DM_Motor_Init(&DM_left_front_motor_init);
	DM_left_back_motor_instance   = DM_Motor_Init(&DM_left_back_motor_init);
	DJI_right_motor_instance      = DJI_Motor_Init(&DJI_right_motor_init);
	DJI_left_motor_instance       = DJI_Motor_Init(&DJI_left_motor_init);

	balance_chassis.joint_motor[0] = DM_right_back_motor_instance;
	balance_chassis.joint_motor[1] = DM_right_front_motor_instance;
	balance_chassis.joint_motor[2] = DM_left_front_motor_instance;
	balance_chassis.joint_motor[3] = DM_left_back_motor_instance;

	balance_chassis.wheel_motor[0] = DJI_right_motor_instance;
	balance_chassis.wheel_motor[1] = DJI_left_motor_instance;
	/******************************底盘电机初始化*****************************/

	/******************************底盘模型初始化*****************************/
	VMC_Init(&chassis_right_leg);
	VMC_Init(&chassis_left_leg);
	/******************************底盘模型初始化*****************************/

	/******************************底盘预测器初始化*****************************/
	xvEstimateKF_Init(&vaEstimateKF);
	/******************************底盘预测器初始化*****************************/

	/******************************底盘控制器初始化*****************************/
	digital_PID_t digital_PID_temp = {
		.Kp = 80.0f,
		.Ki = 0.0f,
		.Kd = 4.8f,
		.Kf = 0.0f,

		.dead_band = 0.02f,

		.output_LPF_RC = 0.0f,

		.improve = PID_IMPROVE_NONE_MOTION,

		.integral_limit = 0.0f,

		.output_max = 12.0f,
	};

	leg_roll_position_pid = digital_PID_temp;

	digital_PID_temp.Kp                = 0.0f;
	digital_PID_temp.Ki                = 0.0f;
	digital_PID_temp.Kd                = 0.0f;
	digital_PID_temp.Kf                = 0.0f;
	digital_PID_temp.dead_band         = 0.0f;
	digital_PID_temp.output_LPF_RC     = 0.0f;
	digital_PID_temp.improve           = PID_IMPROVE_NONE_MOTION;
	digital_PID_temp.integral_limit    = 0.0f;
	digital_PID_temp.output_max        = 0.0f;

	leg_Tp_pid = digital_PID_temp;

	digital_PID_temp.Kp                = 10.0f;
	digital_PID_temp.Ki                = 0.0f;
	digital_PID_temp.Kd                = 1.0f;
	digital_PID_temp.Kf                = 0.0f;
	digital_PID_temp.dead_band         = 0.0025f;
	digital_PID_temp.output_LPF_RC     = 0.0f;
	digital_PID_temp.improve           = PID_IMPROVE_NONE_MOTION;
	digital_PID_temp.integral_limit    = 0.0f;
	digital_PID_temp.output_max        = 8.1f;

	leg_turn_pid = digital_PID_temp;

	digital_PID_temp.Kp             = 100.0f;
	digital_PID_temp.Ki             = 0.0f;
	digital_PID_temp.Kd             = 12.0f;
	digital_PID_temp.Kf             = 0.0f;
	digital_PID_temp.output_LPF_RC  = 0.0f;
	digital_PID_temp.improve        = PID_IMPROVE_NONE_MOTION;
	digital_PID_temp.integral_limit = 0.0f;
	digital_PID_temp.output_max     = 12.0f;

	leg_l_length_pid = digital_PID_temp;
	leg_r_length_pid = digital_PID_temp;
	/******************************底盘控制器初始化*****************************/

	/******************************底盘滤波器初始化*****************************/
	left_leg_F_0_ramp  = ramp_init(&left_leg_F_0_ramp_init);
	right_leg_F_0_ramp = ramp_init(&right_leg_F_0_ramp_init);

	Nlms_Init(&left_leg_F_N_nlms, 25, 0);
	Nlms_Init(&right_leg_F_N_nlms, 25, 0);
	/******************************底盘滤波器初始化*****************************/

	/******************************底盘初始姿态初始化*****************************/
	balance_chassis.roll_set = 0.0f;
	balance_chassis.leg_set  = LEG_LENGTH_INIT;
	/******************************底盘初始姿态初始化*****************************/
}

static void Chassis_Speed_Update(void);

static void Chassis_Leg_Update(void);

void Chassis_Observer(void)
{
	Chassis_Leg_Update( );
	Chassis_Speed_Update( );

	chassis_left_leg.v  = balance_chassis.v_filter;
	chassis_left_leg.x  = balance_chassis.x_filter;
	chassis_right_leg.v = -balance_chassis.v_filter;
	chassis_right_leg.x = -balance_chassis.x_filter;
}

void Chassis_Handle_Exception(void)
{
	;
}

void Chassis_Set_Mode(void)
{
	if ((rc_data->toggle.SF == WFLY_SW_UP) && (rc_data->online == 1) || (balance_chassis.chassis_mode != CHASSIS_OFF))
	{
		balance_chassis.start_flag   = 1;
		balance_chassis.chassis_mode = CHASSIS_STAND_UP;

		DJI_Motor_Enable(balance_chassis.wheel_motor[0]);
		DJI_Motor_Enable(balance_chassis.wheel_motor[1]);
		DM_Motor_Start(balance_chassis.joint_motor[0]);
		DM_Motor_Start(balance_chassis.joint_motor[1]);
		DM_Motor_Start(balance_chassis.joint_motor[2]);
		DM_Motor_Start(balance_chassis.joint_motor[3]);
	}
	else
	{
		balance_chassis.start_flag   = 0;
		balance_chassis.x_filter     = 0.0f;
		balance_chassis.x_set        = balance_chassis.x_filter;
		balance_chassis.leg_set      = LEG_LENGTH_INIT;
		balance_chassis.turn_set     = balance_chassis.total_yaw;
		balance_chassis.chassis_mode = CHASSIS_ZERO_FORCE;

		DJI_Motor_Disable(balance_chassis.wheel_motor[0]);
		DJI_Motor_Disable(balance_chassis.wheel_motor[1]);
		DM_Motor_Stop(balance_chassis.joint_motor[0]);
		DM_Motor_Stop(balance_chassis.joint_motor[1]);
		DM_Motor_Stop(balance_chassis.joint_motor[2]);
		DM_Motor_Stop(balance_chassis.joint_motor[3]);
	}
}

void Chassis_Reference(void)
{
	switch (rc_data->toggle.SA)
	{
		case WFLY_SW_UP:
			if (rc_data->rc.ch[WFLY_CH_RY])
			{
				balance_chassis.leg_set += (float) rc_data->rc.ch[WFLY_CH_RY] * 0.000005f; // 调腿长
				balance_chassis.leg_length_change_flag = 1;
			}
			else
			{
				balance_chassis.leg_length_change_flag = 0;
			}

			if (balance_chassis.leg_set > LEG_LENGTH_MAX)
			{
				balance_chassis.leg_set = LEG_LENGTH_MAX;
			}
			else if (balance_chassis.leg_set < LEG_LENGTH_MIN)
			{
				balance_chassis.leg_set = LEG_LENGTH_MIN;
			}

			if (rc_data->toggle.SC == WFLY_SW_MID)
			{
				balance_chassis.leg_set = LEG_LENGTH_INIT;
			}

			break;

		case WFLY_SW_MID:
			balance_chassis.v_set = (float) rc_data->rc.ch[WFLY_CH_RY] * 0.0025f; // 调速度

			balance_chassis.x_set = 0.0f;

			balance_chassis.wz_set = -(float) rc_data->rc.ch[WFLY_CH_RX] * 0.00125f; // 调角速度

			if (user_abs(balance_chassis.wz_set) < 0.025f)
			{
				balance_chassis.wz_set = 0.0f;
			}

			if (balance_chassis.wz_set < -0.6f)
			{
				balance_chassis.wz_set = -0.6f;
			}
			else if (balance_chassis.wz_set > 0.6f)
			{
				balance_chassis.wz_set = 0.6f;
			}

			if (balance_chassis.wz_set != 0)
			{
				balance_chassis.turn_set = balance_chassis.total_yaw + balance_chassis.wz_set;
			}
			break;
		case WFLY_SW_DOWN:
			switch (rc_data->toggle.SC)
			{
				case WFLY_SW_UP:
					DM_Motor_Enable(balance_chassis.joint_motor[0]);
					DM_Motor_Enable(balance_chassis.joint_motor[1]);
					DM_Motor_Enable(balance_chassis.joint_motor[2]);
					DM_Motor_Enable(balance_chassis.joint_motor[3]);
					break;
				case WFLY_SW_MID:
					DM_Motor_Disable(balance_chassis.joint_motor[0]);
					DM_Motor_Disable(balance_chassis.joint_motor[1]);
					DM_Motor_Disable(balance_chassis.joint_motor[2]);
					DM_Motor_Disable(balance_chassis.joint_motor[3]);
					break;
				case WFLY_SW_DOWN:
					DM_Motor_Set_Zeropoint(balance_chassis.joint_motor[0]);
					DM_Motor_Set_Zeropoint(balance_chassis.joint_motor[1]);
					DM_Motor_Set_Zeropoint(balance_chassis.joint_motor[2]);
					DM_Motor_Set_Zeropoint(balance_chassis.joint_motor[3]);
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}

	switch (rc_data->toggle.SD)
	{
		case WFLY_SW_UP:
			balance_chassis.chassis_mode = CHASSIS_ZERO_FORCE;
			break;
		case WFLY_SW_MID:
			balance_chassis.chassis_mode = CHASSIS_CALIBRATE;
			break;
		case WFLY_SW_DOWN:
			balance_chassis.chassis_mode = CHASSIS_DEBUG;
			break;
		default:
			break;
	}

	chassis_left_leg.v_tar  = balance_chassis.v_set;
	chassis_left_leg.x_tar  = balance_chassis.x_set;
	chassis_right_leg.v_tar = -balance_chassis.v_set;
	chassis_right_leg.x_tar = -balance_chassis.x_set;
}

static float LQR_K_L[12];
static float LQR_K_R[12];

static void Leg_Saturate_Torque(float *val, float min, float max);

void Chassis_Console(void)
{
	static float chassis_err[6] = {0.0f};

	balance_chassis.turn_T = leg_turn_pid.Kp * (balance_chassis.turn_set - balance_chassis.total_yaw) -
	                         leg_turn_pid.Kd * INS.Gyro[2]; // 这样计算更稳一点
	Leg_Saturate_Torque(&balance_chassis.turn_T, -1.0f * leg_turn_pid.output_max, leg_turn_pid.output_max);

	if (balance_chassis.standup_start_cnt > 300)
	{
		balance_chassis.roll_f0 = leg_roll_position_pid.Kp * (balance_chassis.roll_set - balance_chassis.roll)
		                          - leg_roll_position_pid.Kd * INS.Gyro[1];
		Leg_Saturate_Torque(&balance_chassis.roll_f0,
		                    -1.0f * leg_roll_position_pid.output_max,
		                    leg_roll_position_pid.output_max);
	}
	else
	{
		balance_chassis.roll_f0 =
				leg_roll_position_pid.Kp * (balance_chassis.roll_set - balance_chassis.roll) -
				leg_roll_position_pid.Kd * INS.Gyro[1];
		balance_chassis.roll_f0 = float_constrain(balance_chassis.roll_f0, (-0.25f), (0.25f));
	}

	balance_chassis.leg_tp = Digital_PID_Position(&leg_Tp_pid, balance_chassis.theta_err, 0.0f); // 防劈叉pid计算

	/******************************補償量測試*****************************/
	// balance_chassis.leg_tp = 0.0f;
	// balance_chassis.roll_f0	= 0.0f;
	// balance_chassis.turn_T = 0.0f;
	/******************************補償量測試*****************************/

	if (chassis_left_leg.off_ground_flag == 1 && chassis_right_leg.off_ground_flag == 1)
	{
		balance_chassis.body_offground_flag = 1;
		balance_chassis.x_filter            = 0.0f;
		balance_chassis.x_set               = 0.0f;
		balance_chassis.turn_set            = balance_chassis.total_yaw;
	}
	else
	{
		balance_chassis.body_offground_flag = 0;
	}

	for (int i = 0 ; i < 12 ; i++)
	{
		LQR_K_L[i] = LQR_K_Calc(&Poly_Coefficient_Speed[i][0], chassis_left_leg.L0);
	}

	for (int i = 0 ; i < 12 ; i++)
	{
		LQR_K_R[i] = LQR_K_Calc(&Poly_Coefficient_Speed[i][0], chassis_right_leg.L0);
	}

	/******************************右腿控制输出量计算*****************************/

	chassis_err[0] = 0.0f - chassis_right_leg.theta;
	chassis_err[1] = 0.0f - chassis_right_leg.d_theta;
	chassis_err[2] = chassis_right_leg.x_tar - chassis_right_leg.x;
	chassis_err[3] = chassis_right_leg.v_tar - chassis_right_leg.v;
	chassis_err[4] = 0.0f - chassis_right_leg.pitch;
	chassis_err[5] = 0.0f - chassis_right_leg.d_pitch;

	for (uint8_t i = 0 ; i < 6 ; i++)
	{
		chassis_err[i] = float_deadband(chassis_err[i], -0.0025f, 0.0025f);
	}

	/******************************平衡误差测试*****************************/
	chassis_err_right_test[0] = chassis_err[0];
	chassis_err_right_test[1] = chassis_err[1];
	chassis_err_right_test[2] = chassis_err[2];
	chassis_err_right_test[3] = chassis_err[3];
	chassis_err_right_test[4] = chassis_err[4];
	chassis_err_right_test[5] = chassis_err[5];
	/******************************平衡误差测试*****************************/

	balance_chassis.wheel_motor[0]->motor_controller.pid_ref = 0.0f;
	chassis_right_leg.wheel_torque                           = 0.0f;

	for (int i = 0 ; i < 6 ; i++)
	{
		chassis_right_leg.wheel_torque += LQR_K_R[i] * chassis_err[i];
	}

	// 转向环
	chassis_right_leg.wheel_torque += balance_chassis.turn_T;

	chassis_right_leg.Tp = 0.0f;

	for (int i = 6 ; i < 12 ; i++)
	{
		chassis_right_leg.Tp -= LQR_K_R[i] * chassis_err[i - 6];
	}

	chassis_right_leg.Tp += balance_chassis.leg_tp;
	// 防劈腿

	chassis_right_leg.F_0_forward = MG_FF / arm_cos_f32(chassis_right_leg.theta);

	/******************************前馈推力測試*****************************/

	// F_0_forward_test = ramp_calc(left_leg_F_0_ramp, chassis_left_leg.F_0_forward);

	/******************************前馈推力測試*****************************/

	// 支持力
	chassis_right_leg.F0 = chassis_right_leg.F_0_forward +
	                       Digital_PID_Position(&leg_r_length_pid, chassis_right_leg.L0, balance_chassis.leg_set);

	if (balance_chassis.standup_start_cnt < 250)
	{
		chassis_right_leg.F0 = float_constrain(chassis_right_leg.F0, (-10.0f), (10.0f));
	}

	chassis_right_leg.T1 = balance_chassis.joint_motor[2]->receive_data.torque;
	chassis_right_leg.T2 = balance_chassis.joint_motor[3]->receive_data.torque;

	/******************************虛擬離地測試*****************************/
	// chassis_right_leg.T1 = chassis_right_leg.back_joint_torque;
	// chassis_right_leg.T2 = chassis_right_leg.front_joint_torque;
	/******************************虛擬離地測試*****************************/

	Leg_Force_Calc(&chassis_right_leg);

	/******************************支持力滤波測試*****************************/
	// F_N_forward_test = chassis_left_leg.F_N;
	/******************************支持力滤波測試*****************************/

	Nlms_Filter(&right_leg_F_N_nlms, chassis_right_leg.F_N);

	chassis_right_leg.F_N = half_MG_real + right_leg_F_N_nlms.y;
	// + LowPass_Filter1p_Update( &right_F_N_lpf, chassis_right_leg.F_N );

	// VMC_FN_Ground_Detection_R( &chassis_right_leg, &right_F_N_maf, 20.0f );

	if (balance_chassis.recover_flag == 0 && chassis_right_leg.F_N < 20.0f && balance_chassis.leg_length_change_flag == 0)
	{ // 离地了
		chassis_right_leg.off_ground_flag                        = 1;
		balance_chassis.wheel_motor[0]->motor_controller.pid_ref = 0.0f;
		chassis_right_leg.Tp                                     = 0.0f;
		chassis_right_leg.Tp -= LQR_K_R[6] * chassis_err[0];
		chassis_right_leg.Tp -= LQR_K_R[7] * chassis_err[1];
		chassis_right_leg.Tp += balance_chassis.leg_tp;
	}
	else
	{
		chassis_right_leg.off_ground_flag = 0;
		chassis_right_leg.F0 -= balance_chassis.roll_f0;
	}

	Leg_Saturate_Torque(&chassis_right_leg.F0, -1.0f * MAX_F0, MAX_F0);

	// if (balance_chassis.standup_start_cnt < 250)
	// {
	// 	chassis_right_leg.Tp = float_constrain(chassis_right_leg.Tp, (-1.0f), (1.0f));
	// }

	VMC_Calc_T_Joint(&chassis_right_leg);

	Leg_Saturate_Torque(&chassis_right_leg.wheel_torque, -1.0f * MAX_TORQUE_DJI3508, MAX_TORQUE_DJI3508);
	Leg_Saturate_Torque(&chassis_right_leg.back_joint_torque, -1.0f * MAX_TORQUE_DM8009P, MAX_TORQUE_DM8009P);
	Leg_Saturate_Torque(&chassis_right_leg.front_joint_torque, -1.0f * MAX_TORQUE_DM8009P, MAX_TORQUE_DM8009P);
	/******************************右腿控制输出量计算*****************************/

	/******************************左腿控制输出量计算*****************************/
	chassis_err[0] = 0.0f - chassis_left_leg.theta;
	chassis_err[1] = 0.0f - chassis_left_leg.d_theta;
	chassis_err[2] = chassis_left_leg.x_tar - chassis_left_leg.x;
	chassis_err[3] = chassis_left_leg.v_tar - chassis_left_leg.v;
	chassis_err[4] = 0.0f - chassis_left_leg.pitch;
	chassis_err[5] = 0.0f - chassis_left_leg.d_pitch;

	for (uint8_t i = 0 ; i < 6 ; i++)
	{
		chassis_err[i] = float_deadband(chassis_err[i], -0.0025f, 0.0025f);
	}

	/******************************平衡误差测试*****************************/
	chassis_err_left_test[0] = chassis_err[0];
	chassis_err_left_test[1] = chassis_err[1];
	chassis_err_left_test[2] = chassis_err[2];
	chassis_err_left_test[3] = chassis_err[3];
	chassis_err_left_test[4] = chassis_err[4];
	chassis_err_left_test[5] = chassis_err[5];
	/******************************平衡误差测试*****************************/

	balance_chassis.wheel_motor[1]->motor_controller.pid_ref = 0.0f;
	chassis_left_leg.wheel_torque                            = 0.0f;

	for (int i = 0 ; i < 6 ; i++)
	{
		chassis_left_leg.wheel_torque += LQR_K_L[i] * chassis_err[i];
	}

	// 转向环
	chassis_left_leg.wheel_torque += balance_chassis.turn_T;

	chassis_left_leg.Tp = 0.0f;

	for (int i = 6 ; i < 12 ; i++)
	{
		chassis_left_leg.Tp -= LQR_K_L[i] * chassis_err[i - 6];
	}

	chassis_left_leg.Tp += balance_chassis.leg_tp;

	chassis_left_leg.F_0_forward = MG_FF / arm_cos_f32(chassis_left_leg.theta);

	/******************************前馈推力測試*****************************/

	// F_0_forward_test = ramp_calc(left_leg_F_0_ramp, chassis_left_leg.F_0_forward);

	/******************************前馈推力測試*****************************/

	// 支持力
	chassis_left_leg.F0 = chassis_left_leg.F_0_forward +
	                      Digital_PID_Position(&leg_l_length_pid, chassis_left_leg.L0, balance_chassis.leg_set);

	if (balance_chassis.standup_start_cnt < 250)
	{
		chassis_left_leg.F0 = float_constrain(chassis_left_leg.F0, (-10.0f), (10.0f));
	}

	chassis_left_leg.T1 = balance_chassis.joint_motor[0]->receive_data.torque;
	chassis_left_leg.T2 = balance_chassis.joint_motor[1]->receive_data.torque;

	/******************************虛擬離地測試*****************************/
	// chassis_left_leg.T1 = chassis_left_leg.back_joint_torque;
	// chassis_left_leg.T2 = chassis_left_leg.front_joint_torque;
	/******************************虛擬離地測試*****************************/

	Leg_Force_Calc(&chassis_left_leg);

	/******************************支持力滤波測試*****************************/
	// F_N_forward_test = chassis_left_leg.F_N;
	/******************************支持力滤波測試*****************************/

	Nlms_Filter(&left_leg_F_N_nlms, chassis_left_leg.F_N);

	chassis_left_leg.F_N = half_MG_real + left_leg_F_N_nlms.y;
	//+ LowPass_Filter1p_Update( &left_F_N_lpf, chassis_left_leg.F_N );

	// VMC_FN_Ground_Detection_L( &chassis_left_leg, &left_F_N_maf, 20.0f );

	if (balance_chassis.recover_flag == 0 && chassis_left_leg.F_N < 20.0f && balance_chassis.leg_length_change_flag == 0)
	{
		// 离地了
		chassis_left_leg.off_ground_flag                         = 1;
		balance_chassis.wheel_motor[1]->motor_controller.pid_ref = 0.0f;
		chassis_left_leg.Tp                                      = 0.0f;
		chassis_left_leg.Tp -= LQR_K_L[6] * chassis_err[0];
		chassis_left_leg.Tp -= LQR_K_L[7] * chassis_err[1];
		chassis_left_leg.Tp += balance_chassis.leg_tp;
	}
	else
	{
		chassis_left_leg.off_ground_flag = 0;
		chassis_left_leg.F0 += balance_chassis.roll_f0;
	}

	Leg_Saturate_Torque(&chassis_left_leg.F0, -1.0f * MAX_F0, MAX_F0);

	// if (balance_chassis.standup_start_cnt < 250)
	// {
	// 	chassis_left_leg.Tp = float_constrain(chassis_left_leg.Tp, (-1.0f), (1.0f));
	// }

	VMC_Calc_T_Joint(&chassis_left_leg);

	Leg_Saturate_Torque(&chassis_left_leg.wheel_torque, -1.0f * MAX_TORQUE_DJI3508, MAX_TORQUE_DJI3508);
	Leg_Saturate_Torque(&chassis_left_leg.front_joint_torque, -1.0f * MAX_TORQUE_DM8009P, MAX_TORQUE_DM8009P);
	Leg_Saturate_Torque(&chassis_left_leg.back_joint_torque, -1.0f * MAX_TORQUE_DM8009P, MAX_TORQUE_DM8009P);
	/******************************左腿控制输出量计算*****************************/
}

void Chassis_Send_Cmd(void)
{
	static uint32_t chassis_send_cnt = 0;

	chassis_send_cnt++;
	/******************************底盘部分測試*****************************/
	// balance_chassis.wheel_motor[0]->motor_controller.pid_ref = (float) rc_data->rc.ch[WFLY_CH_RY] / 671 * 1.5f;
	// balance_chassis.wheel_motor[1]->motor_controller.pid_ref = (float) rc_data->rc.ch[WFLY_CH_RY] / 671 * 1.5f;
	//	static uint32_t chassis_cnt = 0;
	//	chassis_cnt++;
	//	if ((chassis_cnt % 1500) == 0) // 100Hz
	//	{
	//		DM_Motor_Enable(NULL);
	//	}
	//	else if ((chassis_cnt % 1000) == 0)
	//	{
	//		DM_Motor_Disable(NULL);
	//	}
	if ((chassis_send_cnt % 2) == 0) // 500Hz
	{
		balance_chassis.wheel_motor[0]->motor_controller.pid_ref = chassis_right_leg.wheel_torque;
		balance_chassis.wheel_motor[1]->motor_controller.pid_ref = chassis_left_leg.wheel_torque;
		// DJI_Motor_Control(NULL);
	}
	else
	{
		if (balance_chassis.chassis_mode == CHASSIS_OFF)
		{
			DM_Motor_Disable(NULL);
		}
		else if (balance_chassis.chassis_mode == CHASSIS_DEBUG || balance_chassis.chassis_mode == CHASSIS_CALIBRATE)
		{
			for (uint8_t i = 0 ; i < 4 ; i++)
			{
				if(balance_chassis.joint_motor[i]->contorl_mode_state != ABSOLUTE_STATE)
				{
					balance_chassis.joint_motor[i]->contorl_mode_state = ABSOLUTE_STATE;	
				}
				if(balance_chassis.joint_motor[i]->receive_data.state == 0)
				{
					DM_Motor_Enable(balance_chassis.joint_motor[i]);
				}
			}
		}
		else
		{
			DM_Motor_Disable(NULL);
		}
	}
	/******************************底盘部分測試*****************************/
}

#endif

static float vaEstimateKF_F[4] = {
	1.0f, 0.001f,
	0.0f, 1.0f}; // 状态转移矩阵，控制周期为0.001s

static float vaEstimateKF_P[4] = {
	1.0f, 0.0f,
	0.0f, 1.0f}; // 后验估计协方差初始值

// static float vaEstimateKF_Q[4] = {
// 	0.05f, 0.0f,
// 	0.0f, 0.01f}; // Q预测矩阵初始值 0--速度噪声 3--加速度噪声  增益

// static float vaEstimateKF_R[4] = {
// 	1200.0f, 0.0f,
// 	0.0f, 2000.0f}; // R量测噪声矩阵初始值 0--速度噪声 3--加速度噪声  惩罚

static float vaEstimateKF_Q[4] = {
	0.0001f, 0.0f,
	0.0f, 70.0f}; // Q预测矩阵初始值 0--速度噪声 3--加速度噪声  增益

static float vaEstimateKF_R[4] = {
	250.0f, 0.0f,
	0.0f, 500.0f}; // R量测噪声矩阵初始值 0--速度噪声 3--加速度噪声  惩罚

static const float vaEstimateKF_H[4] = {
	1.0f, 0.0f,
	0.0f, 1.0f}; // 设置矩阵H为常量

static float vaEstimateKF_K[4];

#define SPEED_CHISQUARE_KF 0

#if SPEED_CHISQUARE_KF == 1
/* ---------------- 卡方检验相关变量定义 ---------------- */
// 卡方检验阈值：越小越灵敏。
// 对于2维状态，阈值建议 5.0 ~ 15.0。
// 如果发现稍微一动就拒绝更新（波形发直），调大这个值。
static float Speed_ChiSquareTestThreshold = 10.0f;

// 存储计算出来的卡方值（调试用，观察它来调整阈值）
volatile float Speed_ChiSquare_Val = 0.0f;

// 连续错误计数器：防止滤波器永久拒绝更新（假死）
static uint16_t Speed_ErrorCount = 0;

// 临时矩阵数据缓存（利用KF结构体里已有的temp矩阵，防止malloc）
// 不需要额外定义，直接复用 kf->temp_vector 等
/* ---------------------------------------------------- */

/**
 * @brief 自定义的卡尔曼更新函数，带卡方检验 (Chi-Square Test)
 * 用于检测并剔除异常的观测值（如轮子打滑）
 * @param kf 卡尔曼滤波器结构体指针
 */
/**
 * @brief 自定义的卡尔曼更新函数 (V2.0 修复维度Bug版)
 * 修复了之前矩阵乘法维度不匹配导致计算失败的问题
 */
static void Speed_KF_Check_And_Update(KalmanFilter_t *kf)
{
	// ================== 1. 计算 S = H * P' * H^T + R ==================
	// 2x2 * 2x2 = 2x2
	kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix);
	// 2x2 * 2x2 = 2x2
	kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1);
	// S 存放在 temp_matrix1 (为了省一步加法目标内存，先算逆前的准备)
	// 注意：这里需确保 S 矩阵正确累加 R
	kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S);

	// ================== 2. 计算 S的逆矩阵 ==================
	// temp_matrix1 = S^-1
	kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);

	// ================== 3. 计算残差 r = z - H * x' ==================
	// temp_vector = H * x' (2x1)
	kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector);

	// temp_vector1 = z - temp_vector (残差 r) (2x1)
	kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1);

	// ================== 4. 计算卡方值 (修复点) ==================
	// 目标: ChiSquare = r^T * S^-1 * r

	// Step A: 计算 vec = S^-1 * r
	// S^-1 (2x2) * r (2x1) -> Result (2x1)
	// 【关键修复】使用 temp_vector (2x1) 作为接收容器，而不是 temp_matrix (2x2)
	kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_vector);

	// Step B: 手动点积 (Scalar = r^T * vec)
	float *r_ptr   = kf->temp_vector1.pData;   // 残差 r
	float *vec_ptr = kf->temp_vector.pData;  // S^-1 * r

	// 直接计算 1x1 结果 (r[0]*vec[0] + r[1]*vec[1])
	Speed_ChiSquare_Val = (r_ptr[0] * vec_ptr[0]) + (r_ptr[1] * vec_ptr[1]);

	// ================== 5. 判断逻辑 ==================
	uint8_t update_flag = 1;

	// 保护：如果计算出的卡方值是NaN或负数(数学上不可能，但计算溢出可能)，强制归零
	if (isnan(Speed_ChiSquare_Val) || Speed_ChiSquare_Val < 0)
		Speed_ChiSquare_Val = 0.0f;

	if (Speed_ChiSquare_Val > Speed_ChiSquareTestThreshold)
	{
		Speed_ErrorCount++;
		if (Speed_ErrorCount > 50) // 50ms 连续异常则重置
		{
			update_flag      = 1;
			Speed_ErrorCount = 0;
		}
		else
		{
			update_flag = 0; // 拒绝更新
		}
	}
	else
	{
		Speed_ErrorCount = 0;
		update_flag      = 1;
	}

	// ================== 6. 执行更新 ==================
	if (update_flag)
	{
		// K = P' * H^T * S^-1
		// temp_matrix = P' * H^T
		kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix);
		// K = temp_matrix * S^-1 (S^-1 在 temp_matrix1)
		kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

		// x = x' + K * r
		// 这里的 r 还在 temp_vector1 中
		// temp_vector = K * r (2x1)
		kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector);

		// x = x' + temp_vector
		kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);

		kf->SkipEq5 = 0; // 执行P更新
	}
	else
	{
		// 拒绝更新：直接继承预测值
		memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
		memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
		kf->SkipEq5 = 1; // 跳过P更新
	}
}
#else
#endif

static void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
	Kalman_Filter_Init(EstimateKF, 2, 0, 2); // 状态向量2维 没有控制量 测量向量2维

	memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
	memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
	memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
	memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
	memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

#if SPEED_CHISQUARE_KF == 1
	// 【新增】注册自定义更新函数
	// User_Func3_f 对应标准库中 SetK 之后的位置，这里我们将替代 SetK 和 xhatUpdate
	EstimateKF->User_Func3_f = Speed_KF_Check_And_Update;

	// 【新增】设置跳过标志位
	// 告诉标准库：不要执行标准的计算增益(Eq3)和状态更新(Eq4)
	// 因为我们在 Speed_KF_Check_And_Update 里自己算了
	EstimateKF->SkipEq3 = 1;
	EstimateKF->SkipEq4 = 1;
	// SkipEq5 (P更新) 的标志位会在我们的自定义函数里动态决定
#else
#endif
}

static void xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float acc, float vel, float fusion_v_data[])
{
	// 卡尔曼滤波器测量值更新
	EstimateKF->MeasuredVector[0] = vel; // 测量速度
	EstimateKF->MeasuredVector[1] = acc; // 测量加速度

	// 卡尔曼滤波器更新函数
	Kalman_Filter_Update(EstimateKF);

	// 提取估计值
	for (uint8_t i = 0 ; i < 2 ; i++)
	{
		fusion_v_data[i] = EstimateKF->FilteredValue[i];
	}
}

float Q_1 = 0.0f, Q_3 = 20.0f, R_1 = 100.0f, R_3 = 1500.0f;

// uint32_t speed_kf_test_cnt;

static void Chassis_Speed_Update(void)
{
	static float wheel_z_e_right, wheel_z_e_left = 0.0F;
	static float v_right_b_axis, v_left_b_axis   = 0.0F;
	static float aver_v                          = 0.0F;

	static float fusion_v_data[2];

	static float last_vel_acc   = 0.0F;
	static uint32_t observe_dwt = 0;
	static float dt             = 0.0F;

	dt = DWT_GetDeltaT(&observe_dwt);

	balance_chassis.total_yaw = INS.YawTotalAngle;
	balance_chassis.roll      = INS.Roll;
	balance_chassis.theta_err = 0.0f - (chassis_right_leg.theta + chassis_left_leg.theta);

	vaEstimateKF_F[1] = dt;

	/******************************QR矩阵调试*****************************/
	vaEstimateKF_Q[0] = Q_1;
	vaEstimateKF_Q[3] = Q_3;
	vaEstimateKF_R[0] = R_1;
	vaEstimateKF_R[3] = R_3;

	memcpy(vaEstimateKF.Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
	memcpy(vaEstimateKF.R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
	/******************************QR矩阵调试*****************************/

	wheel_z_e_right = -balance_chassis.wheel_motor[0]->receive_data.speed_rps * 2.0f * PI + INS.Gyro[0] + chassis_right_leg.d_theta;// 右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
	wheel_z_e_left  = -balance_chassis.wheel_motor[1]->receive_data.speed_rps * 2.0f * PI + INS.Gyro[0] + chassis_left_leg.d_theta;// 左边驱动轮转子相对大地角速度，这里定义的是顺时针为正

	v_right_b_axis = wheel_z_e_right * WHEEL_RADIUS +
	                 chassis_right_leg.L0 * chassis_right_leg.d_theta * arm_cos_f32(chassis_right_leg.theta) +
	                 chassis_right_leg.d_L0 * arm_sin_f32(chassis_right_leg.theta); // 机体b系的速度
	v_left_b_axis = wheel_z_e_left * WHEEL_RADIUS +
	                chassis_left_leg.L0 * chassis_left_leg.d_theta * arm_cos_f32(chassis_left_leg.theta) +
	                chassis_left_leg.d_L0 * arm_sin_f32(chassis_left_leg.theta); // 机体b系的速度

	aver_v = ((v_right_b_axis - v_left_b_axis) / 2.0f);

	float user_acc = -INS.MotionAccel_b[1];
	float user_v   = aver_v;

	// if(fabsf(aver_v) < 0.03f && fabsf(-INS.MotionAccel_b[1]) < 0.15f)
	// {
	// 	user_v = 0.0f;
	// 	user_acc = 0.0f;

	// 	// 【新增关键代码】
	// 	// 当判断物理静止时，直接暴力将滤波器内部状态清零
	// 	// 防止因为之前的异常导致滤波器卡在某个很大的速度上，被卡方检验一直拒绝
	// 	vaEstimateKF.xhat_data[0] = 0.0f; // 速度置0
	// 	vaEstimateKF.xhat_data[1] = 0.0f; // 加速度置0
	// 	// speed_kf_test_cnt ++;
	// }

	xvEstimateKF_Update(&vaEstimateKF, user_acc, user_v, fusion_v_data);

	balance_chassis.v_filter = fusion_v_data[0];

	if (fabsf(balance_chassis.v_filter) < 0.005f)
	{
		balance_chassis.v_filter = 0.0f;
	}

	if (balance_chassis.observe_flag == 0)
	{
		if (fabs(fusion_v_data[1] - last_vel_acc) / dt < 0.0001f)
		{
			balance_chassis.observe_flag = 1;
			balance_chassis.x_filter     = 0.0f;
		}
		else
		{
			last_vel_acc = fusion_v_data[1];
		}
	}
	else if (balance_chassis.observe_flag == 1)
	{
		;
	}

	if (balance_chassis.start_flag == 1)
	{
		balance_chassis.standup_start_cnt++;

		if (balance_chassis.v_set != 0.0f)
		{
			balance_chassis.nomotion_start_cnt = 0;
			balance_chassis.x_filter           = 0.0f;
		}
		else
		{
			balance_chassis.nomotion_start_cnt++;
			if (balance_chassis.nomotion_start_cnt > 150)
			{
				balance_chassis.x_filter +=
						balance_chassis.v_filter * dt;
			}
		}
	}
	else
	{
		balance_chassis.nomotion_start_cnt = 0;
		balance_chassis.standup_start_cnt  = 0;
		// balance_chassis.v_filter           = 0.0f;
		balance_chassis.x_filter = 0.0f;
	}
}

static void Chassis_Leg_Update(void)
{
	chassis_left_leg.phi1 = PI / 2.0f + balance_chassis.joint_motor[2]->receive_data.position + 1.88106048f;
	chassis_left_leg.phi4 = PI / 2.0f + balance_chassis.joint_motor[3]->receive_data.position - 1.88106048f;

	chassis_right_leg.phi1 = PI / 2.0f + balance_chassis.joint_motor[0]->receive_data.position + 1.88106048f;
	chassis_right_leg.phi4 = PI / 2.0f + balance_chassis.joint_motor[1]->receive_data.position - 1.88106048f;

	chassis_left_leg.pitch   = (-INS.Pitch) * 0.8f + chassis_left_leg.pitch * 0.2f;
	chassis_left_leg.d_pitch = (-INS.Gyro[0]) * 0.9f + chassis_left_leg.d_pitch * 0.1f;

	chassis_right_leg.pitch   = (INS.Pitch) * 0.8f + chassis_right_leg.pitch * 0.2f;
	chassis_right_leg.d_pitch = (INS.Gyro[0]) * 0.9f + chassis_right_leg.d_pitch * 0.1f;

	balance_chassis.pitch_l      = chassis_left_leg.pitch;
	balance_chassis.pitch_gyro_l = chassis_left_leg.d_pitch;

	balance_chassis.pitch_r      = chassis_right_leg.pitch;
	balance_chassis.pitch_gyro_r = chassis_right_leg.d_pitch;

	static uint32_t leg_dwt = 0;
	static float dt         = 0.0F;

	if (chassis_left_leg.pitch > 0.34f || chassis_left_leg.pitch < -0.34f)
	{
		balance_chassis.recover_flag = 1;
	}
	else
	{
		balance_chassis.recover_flag = 0;
	}

	dt = DWT_GetDeltaT(&leg_dwt);

	VMC_Calc_Base_Data(&chassis_left_leg, &INS, dt);
	VMC_Calc_Base_Data(&chassis_right_leg, &INS, dt);
}

static void Leg_Saturate_Torque(float *val, float min, float max)
{
	if (*val > max)
	{
		*val = max;
	}
	else if (*val < min)
	{
		*val = min;
	}
	else
	{
		return;
	}
}

static void Chassis_Control_Leg(void)
{
	// 	balance_chassis.leg_tp = Digital_PID_Position(&leg_r_length_pid, chassis_right_leg.L0, balance_chassis.leg_set);
	// }
	// else
	// {
	// 	balance_chassis.leg_tp = 0.0f;
	// }

	// leg_roll_position_pid.ref_p = balance_chassis.roll_set;
	// leg_roll_position_pid.fab_p = INS.Roll;
	// leg_roll_position_pid.dt = 0.001f;
	// balance_chassis.roll_f0 = Digital_PID_Position(&leg_roll_position_pid, INS.Roll, balance_chassis.roll_set);

	// leg_Tp_pid.ref_p = 0.0f;
	// leg_Tp_pid.fab_p = balance_chassis.theta_err;
	// leg_Tp_pid.dt = 0.001f;
	// balance_chassis.leg_tp += Digital_PID_Position(&leg_Tp_pid, balance_chassis.theta_err, 0.0f);

	// leg_turn_pid.ref_p = balance_chassis.turn_set;
	// leg_turn_pid.fab_p = INS.YawTotalAngle;
	// leg_turn_pid.dt = 0.001f;
	// balance_chassis.turn_T = Digital_PID_Position(&leg_turn_pid, INS.YawTotalAngle, balance_chassis.turn_set);
}

//static void Forward_Kinematics(float left_leg_set, float right_leg_set, vmc_leg_t *chassis_left_leg, vmc_leg_t *chassis_right_leg)
//{
//	float L0_temp = 0.0f;
//	float theta   = 0.0f;

//	Inverse_Kinematics(left_leg_set, chassis_left_leg->phi1, chassis_left_leg->phi4, &L0_temp, &theta);
//	chassis_left_leg->L0    = L0_temp;
//	chassis_left_leg->theta = theta;

//	Inverse_Kinematics(right_leg_set, chassis_right_leg->phi1, chassis_right_leg->phi4, &L0_temp, &theta);
//	chassis_right_leg->L0    = L0_temp;
//	chassis_right_leg->theta = theta;
//}

static void Chassis_Debug_Control(void)
{
	float joint_pos_right[2] = {0};
	float joint_pos_left[2]  = {0};

	if(!(Judge_IF_NAN(joint_pos_left[0]) || Judge_IF_NAN(joint_pos_left[1]) || Judge_IF_NAN(joint_pos_right[0]) || Judge_IF_NAN(joint_pos_right[1])))
	{
		balance_chassis.joint_motor[0]->transmit_data.position_des = joint_pos_right[0];
		balance_chassis.joint_motor[1]->transmit_data.position_des = joint_pos_right[1];
		balance_chassis.joint_motor[2]->transmit_data.position_des = joint_pos_left[0];
		balance_chassis.joint_motor[3]->transmit_data.position_des = joint_pos_left[1];
	}

	float_constrain(balance_chassis.joint_motor[0]->transmit_data.position_des, MIN_JOINT0_POS_LIMIT, MAX_JOINT0_POS_LIMIT);
	float_constrain(balance_chassis.joint_motor[1]->transmit_data.position_des, MIN_JOINT1_POS_LIMIT, MAX_JOINT1_POS_LIMIT);
	float_constrain(balance_chassis.joint_motor[2]->transmit_data.position_des, MIN_JOINT2_POS_LIMIT, MAX_JOINT2_POS_LIMIT);
	float_constrain(balance_chassis.joint_motor[3]->transmit_data.position_des, MIN_JOINT3_POS_LIMIT, MAX_JOINT3_POS_LIMIT);
}