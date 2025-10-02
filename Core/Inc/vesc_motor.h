#ifndef VESC_MOTOR_H
#define VESC_MOTOR_H

#include "main.h"
#include "vesc_communication.h"

typedef enum {
    CONTROL_DUTY = 0,      // 占空比控制
    CONTROL_CURRENT = 1,   // 电流控制
    CONTROL_RPM = 2,       // 转速控制
    CONTROL_BRAKE = 3      // 刹车控制
} Control_Type_t;

typedef struct {
    float target_duty;
    float target_current;
    int32_t target_rpm;
    float target_brake;

    float current_rpm;
    float current_current;
    float current_duty;

    float current_temp_fet;
    float current_temp_motor;

    uint8_t control_type;
    uint8_t enabled;
    uint8_t initialized;
} VESC_Motor_t;

extern VESC_Motor_t motor;

void VESC_Motor_Init(void);
void VESC_Motor_UpdateFeedback(void);
void VESC_Motor_ControlUpdate(void);
void VESC_Motor_SetTarget(float value);

#endif