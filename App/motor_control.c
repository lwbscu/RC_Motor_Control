#include "motor_control.h"
#include "m3508_motor.h"
#include "can_communication.h"

static uint32_t test_counter = 0;

void MotorControl_Init(void) {
    // 初始化CAN通信
    CAN_Init();

    // 初始化电机
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

    // 测试程序：电机1旋转360度，然后反向
    static float target_angle = 0;
    static uint8_t direction = 1;

    if (test_counter % 300 == 0) { // 每3秒改变目标
        if (direction) {
            target_angle += 360.0f; // 正转360度
        } else {
            target_angle -= 360.0f; // 反转360度
        }
        direction = !direction;

        M3508_SetTargetAngle(0, target_angle); // 控制电机1
    }
}