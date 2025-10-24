#ifndef CAN_COMMUNICATION_H
#define CAN_COMMUNICATION_H

#include "main.h"

// 重命名宏，ID 0x204 保持不变 (对应电机ID 4)
#define M2006_MOTOR_ID      0x204
// 发送ID 0x200 保持不变 (C610控制1-4号电机)
#define CAN_SEND_ID         0x200

typedef struct {
    uint16_t angle;
    int16_t speed;
    int16_t current;
    uint8_t temperature;
    uint8_t data_updated;
} M3508_Motor_Data_t; // 结构体名称可以保持不变，因为它只是CAN数据的通用容器

extern M3508_Motor_Data_t motor_data;
extern FDCAN_HandleTypeDef hfdcan1;

void CAN_Init(void);
void CAN_SendMotorCommand(int16_t current);
void CAN_ReceiveCallback(FDCAN_HandleTypeDef *hfdcan);

#endif