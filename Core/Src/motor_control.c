#include "motor_control.h"
#include "m3508_motor.h"
#include "can_communication.h"

static uint32_t test_counter = 0;

void MotorControl_Init(void) {
    CAN_Init();
    HAL_Delay(100);
    M3508_Init();
}

void MotorControl_Task(void) {
    M3508_UpdateFeedback();
    M3508_ControlUpdate();
}

void MotorControl_Test(void) {
    test_counter++;

    static uint8_t test_phase = 0;

    if (test_counter % 500 == 0) { // 每5秒改变
        switch(test_phase) {
            case 0:
                M3508_SetTargetSpeed(200.0f);  // 正转200RPM
                break;
            case 1:
                M3508_SetTargetSpeed(-200.0f); // 反转200RPM
                break;
            case 2:
                M3508_SetTargetSpeed(0.0f);    // 停止
                break;
            default:
                test_phase = 0;
                return;
        }
        test_phase++;
        if (test_phase > 2) test_phase = 0;
    }
}