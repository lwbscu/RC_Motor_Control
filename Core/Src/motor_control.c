#include "motor_control.h"
#include "m3508_motor.h"
#include "can_communication.h"

void MotorControl_Init(void) {
    CAN_Init();
    HAL_Delay(100);
    M3508_Init();
    VOFA_Init();
}

void MotorControl_Task(void) {
    M3508_UpdateFeedback();
    VOFA_ProcessCommand();
    M3508_ControlUpdate();
}