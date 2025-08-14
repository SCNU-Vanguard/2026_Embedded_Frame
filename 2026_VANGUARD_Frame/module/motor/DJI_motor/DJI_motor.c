#include "DJI_motor.h"

#include "bsp_dwt.h"

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* DJI电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static DJI_motor_instance_t *dji_motor_instance[DJI_MOTOR_CNT] = {NULL}; // 会在control任务中遍历该指针数组进行pid计算

void DJI_Motor_Error_Detection(DJI_motor_instance_t *motor);

/**
 * @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用6个(2can*3group)can_instance专门负责发送
 *        该变量将在 DJI_Motor_Control() 中使用,分组在 Motor_Sender_Grouping()中进行
 *
 * @note  因为只用于发送,所以不需要在bsp_can中注册
 *
 * C610(m2006)/C620(m3508):0x1ff,0x200;
 * GM6020:0x1ff,0x2ff *0x1fe,0x2fe(新固件)
 * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
 */
static CAN_t sender_assignment[15] = {
    [0] = {.can_handle = &hfdcan1, .tx_header.Identifier = 0x1ff, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [1] = {.can_handle = &hfdcan1, .tx_header.Identifier = 0x200, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [2] = {.can_handle = &hfdcan1, .tx_header.Identifier = 0x2ff, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [3] = {.can_handle = &hfdcan2, .tx_header.Identifier = 0x1ff, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [4] = {.can_handle = &hfdcan2, .tx_header.Identifier = 0x200, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [5] = {.can_handle = &hfdcan2, .tx_header.Identifier = 0x2ff, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [6] = {.can_handle = &hfdcan3, .tx_header.Identifier = 0x1ff, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [7] = {.can_handle = &hfdcan3, .tx_header.Identifier = 0x200, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [8] = {.can_handle = &hfdcan3, .tx_header.Identifier = 0x2ff, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [9] = {.can_handle = &hfdcan1, .tx_header.Identifier = 0x1fe, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [10] = {.can_handle = &hfdcan2, .tx_header.Identifier = 0x1fe, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [11] = {.can_handle = &hfdcan3, .tx_header.Identifier = 0x1fe, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [12] = {.can_handle = &hfdcan1, .tx_header.Identifier = 0x2fe, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [13] = {.can_handle = &hfdcan2, .tx_header.Identifier = 0x2fe, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [14] = {.can_handle = &hfdcan3, .tx_header.Identifier = 0x2fe, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
};

/**
 * @brief 9个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
 *        flag的初始化在 Motor_Sender_Grouping()中进行
 */
static uint8_t sender_enable_flag[15] = {0};

/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令
 */
static void Motor_Sender_Grouping(DJI_motor_instance_t *motor, can_init_config_t *config)
{
    uint8_t motor_id = config->tx_id - 1; // 下标从零开始,先减一方便赋值
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    switch (motor->motor_type) {
        case M2006:
        case M3508:
            if (motor_id < 4) // 根据ID分组
            {
                motor_send_num = motor_id;
                motor_grouping = config->can_handle == &hfdcan1 ? 1 : (config->can_handle == &hfdcan2 ? 4 : 7);
            } else {
                motor_send_num = motor_id - 4;
                motor_grouping = config->can_handle == &hfdcan1 ? 0 : (config->can_handle == &hfdcan2 ? 3 : 6);
            }

            // 计算接收id并设置分组发送id
            config->rx_id                      = 0x200 + motor_id + 1; // 把ID+1,进行分组设置
            sender_enable_flag[motor_grouping] = 1;                    // 设置发送标志位,防止发送空帧
            motor->message_num                 = motor_send_num;
            motor->sender_group                = motor_grouping;

            // 检查是否发生id冲突
            for (size_t i = 0; i < idx; ++i) {
                if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id) {

                    uint8_t can_bus = motor->motor_can_instance->can_handle == &hfdcan1 ? 1 : (motor->motor_can_instance->can_handle == &hfdcan2 ? 2 : 3);
                    while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
											;
                }
            }
            break;

        case GM6020:
            if (motor_id < 4) {
                motor_send_num = motor_id;
                motor_grouping = config->can_handle == &hfdcan1 ? 9 : (config->can_handle == &hfdcan2 ? 10 : 11);
            } else {
                motor_send_num = motor_id - 4;
                motor_grouping = config->can_handle == &hfdcan1 ? 12 : (config->can_handle == &hfdcan2 ? 13 : 14);
            }

            config->rx_id                      = 0x204 + motor_id + 1; // 把ID+1,进行分组设置
            sender_enable_flag[motor_grouping] = 1;                    // 只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
            motor->message_num                 = motor_send_num;
            motor->sender_group                = motor_grouping;

            for (size_t i = 0; i < idx; ++i) {
                if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id) {

                    uint16_t can_bus;
                    can_bus = motor->motor_can_instance->can_handle == &hfdcan1 ? 1 : (motor->motor_can_instance->can_handle == &hfdcan2 ? 2 : 3);
                    while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
											;
                }
            }
            break;

        default: // other motors should not be registered here
            while (1)	
							;
    }	
}

/**
 * @todo  是否可以简化多圈角度的计算？
 * @brief 根据返回的can_instance对反馈报文进行解析
 *
 * @param _instance 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
 */
static void Decode_DJI_Motor(CAN_t *_instance)
{
    // 这里对can instance的id进行了强制转换,从而获得电机的instance实例地址
    // _instance指针指向的id是对应电机instance的地址,通过强制转换为电机instance的指针,再通过->运算符访问电机的成员motor_measure,最后取地址获得指针
    uint8_t *rxbuff              = _instance->rx_buff;
    DJI_motor_instance_t *motor      = (DJI_motor_instance_t *)_instance->id;
    DJI_motor_callback_t *measure = &motor->measure; // measure要多次使用,保存指针减小访存开销

    Supervisor_Reload(motor->supervisor);
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册
    measure->last_ecd           = measure->ecd;
    measure->ecd                = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->ecd                = ((measure->ecd >= measure->offset_ecd) ? (measure->ecd - measure->offset_ecd) : (measure->ecd + 8191 - measure->offset_ecd));
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->last_speed_aps     = measure->speed_aps;
    measure->speed_aps          = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->temperature = rxbuff[6];

    if ((int16_t)(measure->ecd - measure->last_ecd) > 4096)
        measure->total_round--;
    else if ((int16_t)(measure->ecd - measure->last_ecd) < -4096)
        measure->total_round++;

    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;

    DJI_Motor_Error_Detection(motor);
}

static void DJI_Motor_Lost_Callback(void *motor_ptr)
{
    uint16_t can_bus;
    DJI_motor_instance_t *motor = (DJI_motor_instance_t *)motor_ptr;
    can_bus                 = motor->motor_can_instance->can_handle == &hfdcan1 ? 1 : (motor->motor_can_instance->can_handle == &hfdcan2 ? 2 : 3);

}

// 电机初始化,返回一个电机实例
DJI_motor_instance_t *DJI_Motor_Init(motor_init_config_t *config)
{
    DJI_motor_instance_t *instance = (DJI_motor_instance_t *)malloc(sizeof(DJI_motor_instance_t));
    memset(instance, 0, sizeof(DJI_motor_instance_t));

    // motor basic setting 电机基本设置
    instance->motor_type     = config->motor_type;                     // 6020 or 2006 or 3508
    instance->motor_settings = config->controller_setting_init_config; // 正反转,闭环类型等

    // motor controller init 电机控制器初始化
    PID_Init(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PID_Init(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PID_Init(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    PID_Init(&instance->motor_controller.torque_PID, &config->controller_param_init_config.torque_PID);
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.current_feedforward_ptr  = config->controller_param_init_config.current_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr    = config->controller_param_init_config.speed_feedforward_ptr;

    // 后续增加电机前馈控制器(速度和电流)

    // 电机分组,因为至多4个电机可以共用一帧CAN控制报文
    Motor_Sender_Grouping(instance, &config->can_init_config);

    // 注册电机到CAN总线
    config->can_init_config.can_module_callback = Decode_DJI_Motor; // set callback
    config->can_init_config.id                  = instance;       // set id,eq to address(it is IdTypentity)
    instance->motor_can_instance                = CAN_Register(&config->can_init_config);

    // 注册守护线程
    supervisor_init_config_t supervisor_config = {
        .handler_callback     = DJI_Motor_Lost_Callback,
        .owner_id     = instance,
        .reload_count = 2, // 20ms未收到数据则丢失
    };
    instance->supervisor = Supervisor_Register(&supervisor_config);

    DJI_Motor_Stop(instance);
    dji_motor_instance[idx++] = instance;
    return instance;
}

/* 电流只能通过电机自带传感器监测,后续考虑加入力矩传感器应变片等 */
void DJI_Motor_Change_Feedback(DJI_motor_instance_t *motor, closeloop_type_e loop, feedback_type_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
			;
}

void DJI_Motor_Stop(DJI_motor_instance_t *motor)
{
    motor->stop_flag = MOTOR_DISABLE;
}

void DJI_Motor_Enable(DJI_motor_instance_t *motor)
{
    motor->stop_flag = MOTOR_ENABLE;
}

/* 修改电机的实际闭环对象 */
void DJI_Motor_Change_Outerloop(DJI_motor_instance_t *motor, closeloop_type_e outer_loop)
{
    motor->motor_settings.outer_loop_type = outer_loop;
}

// 设置参考值
void DJI_Motor_Set_Ref(DJI_motor_instance_t *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

// 异常检测
void DJI_Motor_Error_Detection(DJI_motor_instance_t *motor)
{

}

// 为所有电机实例计算三环PID,发送控制报文
void DJI_Motor_Control(void)
{

}
