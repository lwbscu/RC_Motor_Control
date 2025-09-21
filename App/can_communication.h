#ifndef CAN_COMMUNICATION_H
#define CAN_COMMUNICATION_H

#include "main.h"  // 包含main.h，它会包含所有必要的HAL头文件

#define M3508_MOTOR_ID_1    0x201
#define M3508_MOTOR_ID_2    0x202
#define M3508_MOTOR_ID_3    0x203
#define M3508_MOTOR_ID_4    0x204

#define CAN_SEND_ID         0x200

typedef struct {
    uint16_t angle;         // 机械角度
    int16_t speed;          // 转速RPM
    int16_t current;        // 实际转矩电流
    uint8_t temperature;    // 温度
} M3508_Motor_Data_t;

extern M3508_Motor_Data_t motor_data[4];
extern FDCAN_HandleTypeDef hfdcan1;  // 声明外部FDCAN句柄

void CAN_Init(void);
void CAN_SendMotorCommand(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_ReceiveCallback(FDCAN_HandleTypeDef *hfdcan);

#endif