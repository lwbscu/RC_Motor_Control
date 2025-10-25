#ifndef CAN_COMMUNICATION_H
#define CAN_COMMUNICATION_H

#include "main.h"

// 关键修改：定义两个电调的ID
#define MOTOR_ID_3      0x203 // 新增电机
#define MOTOR_ID_4      0x204 // 原有电机
// 发送ID 0x200 保持不变 (C610控制1-4号电机)
#define CAN_SEND_ID         0x200

typedef struct {
    uint16_t angle;
    int16_t speed;
    int16_t current;
    uint8_t temperature;
    uint8_t data_updated; // 标志位，表示数据是否已更新
} M2006_Motor_Data_t; // 结构体名称保持不变

// 关键修改：将 motor_data 扩展为数组，[0] 对应 ID 3, [1] 对应 ID 4
extern M2006_Motor_Data_t motor_data[2];
extern FDCAN_HandleTypeDef hfdcan1;

void CAN_Init(void);
// 关键修改：发送函数现在一次发送两个电机的电流
void CAN_SendMotorCommand(int16_t current_id3, int16_t current_id4);
void CAN_ReceiveCallback(FDCAN_HandleTypeDef *hfdcan);

#endif