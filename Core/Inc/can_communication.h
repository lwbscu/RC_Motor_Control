#ifndef CAN_COMMUNICATION_H
#define CAN_COMMUNICATION_H

#include "main.h"

#define M3508_MOTOR_ID_1    0x201
#define M3508_MOTOR_ID_2    0x202
#define M3508_MOTOR_ID_3    0x203
#define M3508_MOTOR_ID_4    0x204
#define CAN_SEND_ID         0x200

typedef struct {
    uint16_t angle;         
    int16_t speed;          
    int16_t current;        
    uint8_t temperature;    
    uint8_t data_updated;   // 数据更新标志
} M3508_Motor_Data_t;

extern M3508_Motor_Data_t motor_data[4];
extern FDCAN_HandleTypeDef hfdcan1;

void CAN_Init(void);
void CAN_SendMotorCommand(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_ReceiveCallback(FDCAN_HandleTypeDef *hfdcan);

#endif