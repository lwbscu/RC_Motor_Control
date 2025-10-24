#include "motor_control.h"
// 关键修改：包含新的头文件
#include "m2006_motor.h"
#include "can_communication.h"

void MotorControl_Init(void) {
    CAN_Init();
    HAL_Delay(100);
    // 关键修改：调用新的初始化函数
    M2006_Init();
    VOFA_Init();
}

void MotorControl_Task(void) {
    // 关键修改：调用新的函数
    M2006_UpdateFeedback();
    VOFA_ProcessCommand();
    M2006_ControlUpdate();
}