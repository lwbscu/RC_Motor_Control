#include "motor_control.h"
#include "m3508_motor.h"
#include "can_communication.h"

static uint32_t test_counter = 0;

void MotorControl_Init(void) {
    CAN_Init();
    HAL_Delay(100);  // 等待CAN稳定
    M3508_Init();
}

void MotorControl_Task(void) {
    // 更新所有电机反馈数据
    for (int i = 0; i < MOTOR_COUNT; i++) {
        M3508_UpdateFeedback(i);
    }

    // 执行PID控制更新
    M3508_ControlUpdate();
}

void MotorControl_Test(void) {
    test_counter++;

    // 简单的速度测试，先测试速度环是否工作
    static uint8_t test_phase = 0;

    if (test_counter % 500 == 0) { // 每5秒改变
        switch(test_phase) {
            case 0:
                M3508_SetTargetSpeed(0, 100.0f);  // 电机1正转100RPM
                break;
            case 1:
                M3508_SetTargetSpeed(0, -100.0f); // 电机1反转100RPM
                break;
            case 2:
                M3508_SetTargetSpeed(0, 0.0f);    // 停止
                break;
            default:
                test_phase = 0;
                return;
        }
        test_phase++;
        if (test_phase > 2) test_phase = 0;
    }
}