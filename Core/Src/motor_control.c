#include "motor_control.h"
#include "vesc_motor.h"
#include "vesc_communication.h"

void MotorControl_Init(void) {
    VESC_CAN_Init();
    HAL_Delay(100);
    VESC_Motor_Init();
    VOFA_Init();
}

void MotorControl_Task(void) {
    VESC_Motor_UpdateFeedback();
    VOFA_ProcessCommand();
    VESC_Motor_ControlUpdate();
}