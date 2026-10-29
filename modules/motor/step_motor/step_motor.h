#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include "bsp_can.h"
#include "motor_def.h"
#include <stdint.h>

#define STEP_MOTOR_CNT             6
#define STEP_MOTOR_CMD_INTERVAL_MS  10

/* 步进电机初始化配置 */
typedef struct
{
    CAN_Init_Config_s can_init_config;
    Motor_Reverse_Flag_e motor_reverse_flag;

} StepMotor_Init_Config_s;

/* 步进电机实例 */
typedef struct
{
    CAN_HandleTypeDef *can_handle;

    uint8_t address;
    Motor_Reverse_Flag_e motor_reverse_flag;
    Motor_Working_Type_e stop_flag;

    /* 待处理请求 */
    uint8_t enable_pending;/*表示有一个使能请求等待处理*/
    uint8_t stop_pending;
    uint8_t motion_pending;/*表示有新的速度或位置命令等待处理；复制到发送缓冲区后清零*/

    /* 最新运动目标 */
    uint8_t target_cmd[12];/*保存最新的运动命令，包括功能码、方向、速度、脉冲数*/
    uint8_t target_len;/*target_cmd 中有效数据的长度*/

    /* 当前正在发送的命令 */
    uint8_t send_cmd[12];/*保存当前正在发送的完整命令*/
    uint8_t send_len;/*当前命令的有效长度*/
    uint8_t send_offset;/*当前命令的参数已经发送到哪个位置*/
    uint8_t packet_index;

    uint32_t last_tx_ms;
} StepMotorInstance;

/* 初始化一次；超过实例数量上限时返回NULL */
StepMotorInstance *StepMotorInit( StepMotor_Init_Config_s *config);

/* 使能电机 */
void StepMotorEnable(StepMotorInstance *motor);

/* 停止运动，保持使能 */
void StepMotorStop(StepMotorInstance *motor);

/*
 * 速度模式
 * direction：0正向，1反向
 * speed：0~5000 RPM
 * acceleration：0~255，0表示直接启动
 */
void StepMotorSetSpeed(
    StepMotorInstance *motor,
    uint8_t direction,
    uint16_t speed,
    uint8_t acceleration);

/*
 * 位置模式
 * direction：0正向，1反向
 * speed：1~5000 RPM
 * acceleration：0~255
 * pulses：脉冲数
 * absolute：0相对位置，1绝对位置
 */
void StepMotorSetPosition(
    StepMotorInstance *motor,
    uint8_t direction,
    uint16_t speed,
    uint8_t acceleration,
    uint32_t pulses,
    uint8_t absolute);


void StepMotorControl(void);

#endif
