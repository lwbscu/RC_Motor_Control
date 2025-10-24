#ifndef CAN_COMMUNICATION_H
#define CAN_COMMUNICATION_H

#include "main.h"

#define M3508_MOTOR_ID      0x204
#define CAN_SEND_ID         0x200

typedef struct {
    uint16_t angle;
    int16_t speed;
    int16_t current;
    uint8_t temperature;
    uint8_t data_updated;
} M3508_Motor_Data_t;

extern M3508_Motor_Data_t motor_data;
extern FDCAN_HandleTypeDef hfdcan1;

void CAN_Init(void);
void CAN_SendMotorCommand(int16_t current);
void CAN_ReceiveCallback(FDCAN_HandleTypeDef *hfdcan);

#endif