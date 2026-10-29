#include "step_motor.h"
#include <string.h>

/* 参考达妙电机的写法，静态保存实例 */
static StepMotorInstance step_motor_instance[STEP_MOTOR_CNT];
static uint8_t idx_step;


/*
 * 保存最新运动目标。
 *
 * repeat = 0：
 * 相同目标不重复发送，用于速度和绝对位置。
 *
 * repeat = 1：
 * 每次调用都产生请求，用于相对位置。
 */
static void StepMotorSetTarget(
    StepMotorInstance *motor,
    const uint8_t *command,
    uint8_t length,
    uint8_t repeat)
{
        memcpy(motor->target_cmd, command, length);
        motor->target_len = length;
        motor->motion_pending = 1;  
}

/* 初始化电机实例 */
StepMotorInstance *StepMotorInit(StepMotor_Init_Config_s *config)
{
    StepMotorInstance *motor =  &step_motor_instance[idx_step++];

    memset(motor, 0, sizeof(*motor));

    motor->can_handle = config->can_init_config.can_handle;

    motor->address = (uint8_t)config->can_init_config.tx_id;

    motor->motor_reverse_flag = config->motor_reverse_flag;

    motor->stop_flag = MOTOR_STOP;

    motor->last_tx_ms = HAL_GetTick() - STEP_MOTOR_CMD_INTERVAL_MS;


    return motor;
}

/* 提交使能请求 */
void StepMotorEnable(StepMotorInstance *motor)
{
   motor->stop_flag = MOTOR_ENABLED;
   motor->enable_pending = 1;

}

/* 提交停止请求，并取消尚未开始发送的运动 */
void StepMotorStop(StepMotorInstance *motor)
{
    if (motor->stop_flag != MOTOR_STOP)
        motor->stop_pending = 1;

    motor->stop_flag = MOTOR_STOP;
    motor->enable_pending = 0;

    motor->motion_pending = 0;
    motor->target_len = 0;
}

/* 设置速度目标 */
void StepMotorSetSpeed(
    StepMotorInstance *motor,
    uint8_t direction,
    uint16_t speed,
    uint8_t acceleration)
{
    uint8_t dir =(direction != 0) ^
        (motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE);

    /*
     * 厂商命令：
     * 地址 F6 方向 速度高 速度低 加速度 同步 6B
     *
     * CAN发送时，地址放入扩展帧ID。
     */
    const uint8_t command[] = {0xF6,dir,(uint8_t)(speed >> 8),(uint8_t)speed,acceleration,0,0x6B};

    StepMotorSetTarget(motor,command,sizeof(command),0);
}

/* 设置位置目标 */
void StepMotorSetPosition(
    StepMotorInstance *motor,
    uint8_t direction,
    uint16_t speed,
    uint8_t acceleration,
    uint32_t pulses,
    uint8_t absolute)
{
    uint8_t dir =
        (direction != 0) ^
        (motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE);

    const uint8_t command[] = {0xFD,dir,(uint8_t)(speed >> 8),(uint8_t)speed,acceleration,(uint8_t)(pulses >> 24),(uint8_t)(pulses >> 16),(uint8_t)(pulses >> 8),(uint8_t)pulses,absolute ? 1 : 0,0,0x6B};

    StepMotorSetTarget(motor,command,sizeof(command),absolute == 0);
}

/*
 * 取出下一条待发送命令。
 *
 * 优先级：
 * 停止 > 使能 > 运动
 */
static void StepMotorLoadCommand(
    StepMotorInstance *motor,
    uint32_t now)
{
    if (motor->send_len != 0)
        return;

    if (!motor->stop_pending &&
        (uint32_t)(now - motor->last_tx_ms) < STEP_MOTOR_CMD_INTERVAL_MS)
        return;

    if (motor->stop_pending)
    {
        const uint8_t command[] = {
            0xFE, 0x98, 0, 0x6B
        };

        memcpy(motor->send_cmd, command,
            sizeof(command));

        motor->send_len = sizeof(command);
        motor->stop_pending = 0;
    }
    else if (motor->enable_pending)
    {
        const uint8_t command[] = {
            0xF3, 0xAB, 1, 0, 0x6B
        };

        memcpy(
            motor->send_cmd,
            command,
            sizeof(command));

        motor->send_len = sizeof(command);
        motor->enable_pending = 0;
    }
    else if (motor->stop_flag == MOTOR_ENABLED &&
             motor->motion_pending)
    {
        memcpy(
            motor->send_cmd,
            motor->target_cmd,
            motor->target_len);

        motor->send_len = motor->target_len;
        motor->motion_pending = 0;
    }

    if (motor->send_len)
    {
        /* 第0字节为功能码，参数从第1字节开始 */
        motor->send_offset = 1;
        motor->packet_index = 0;
    }
}

/*
 * 尝试发送一个CAN分包。
 *
 * 扩展ID = 电机地址左移8位 | 分包序号
 * 数据   = 功能码 + 最多7字节参数
 *
 * 邮箱不足时直接返回，下次Control继续发送。
 */
static void StepMotorSendPacket(StepMotorInstance *motor)
{
    if (HAL_CAN_GetState(motor->can_handle) != HAL_CAN_STATE_LISTENING)
        return;

    if (HAL_CAN_GetTxMailboxesFreeLevel(motor->can_handle) == 0)
        return;

    CAN_TxHeaderTypeDef header = {0};

    uint8_t data[8] = {0};
    uint32_t mailbox;

    uint8_t size = motor->send_len - motor->send_offset;
    if (size > 7)
        size = 7;

    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;

    header.ExtId =((uint32_t)motor->address << 8) | motor->packet_index;

    header.DLC = size + 1;
    header.TransmitGlobalTime = DISABLE;

    data[0] = motor->send_cmd[0];

    memcpy(data + 1,motor->send_cmd + motor->send_offset,size);

    HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(motor->can_handle,&header,data,&mailbox);
    if (result != HAL_OK)
    {
        return;
    }

    motor->send_offset += size;
    ++motor->packet_index;

    if (motor->send_offset >= motor->send_len)
    {
      motor->send_len = 0;
      motor->last_tx_ms = HAL_GetTick();
    }

}

/* 步进电机统一控制接口 */
void StepMotorControl(void)
{
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < idx_step; ++i)
    {
        StepMotorInstance *motor = &step_motor_instance[i];
        StepMotorLoadCommand(motor, now);

        if (motor->send_len)
            StepMotorSendPacket(motor);
    }
}
