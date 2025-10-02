#ifndef VESC_COMMUNICATION_H
#define VESC_COMMUNICATION_H

#include "main.h"

// VESC CAN数据包类型
#define CAN_PACKET_SET_DUTY           0
#define CAN_PACKET_SET_CURRENT        1
#define CAN_PACKET_SET_CURRENT_BRAKE  2
#define CAN_PACKET_SET_RPM            3
#define CAN_PACKET_SET_POS            4
#define CAN_PACKET_STATUS             9
#define CAN_PACKET_STATUS_4           16

// VESC控制器ID
#define VESC_CONTROLLER_ID            96

typedef struct {
    int32_t rpm;
    int16_t current;
    int16_t duty;
    uint8_t data_updated;
} VESC_Status_t;

typedef struct {
    int16_t temp_fet;
    int16_t temp_motor;
    int16_t current_in;
    int16_t pid_pos;
    uint8_t data_updated;
} VESC_Status4_t;

extern VESC_Status_t vesc_status;
extern VESC_Status4_t vesc_status4;
extern FDCAN_HandleTypeDef hfdcan1;

void VESC_CAN_Init(void);
void VESC_SetDuty(float duty);
void VESC_SetCurrent(float current);
void VESC_SetRPM(int32_t rpm);
void VESC_SetBrakeCurrent(float current);
void VESC_ReceiveCallback(FDCAN_HandleTypeDef *hfdcan);

#endif