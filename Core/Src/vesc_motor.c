#include "vesc_motor.h"
#define MAX_SAFE_DUTY 0.99f  // 调试期间最大安全占空比
VESC_Motor_t motor;

void VESC_Motor_Init(void) {
    motor.target_duty = 0.0f;
    motor.target_current = 0.0f;
    motor.target_rpm = 0;
    motor.target_brake = 0.0f;

    motor.current_rpm = 0.0f;
    motor.current_current = 0.0f;
    motor.current_duty = 0.0f;

    motor.current_temp_fet = 0.0f;
    motor.current_temp_motor = 0.0f;

    motor.control_type = CONTROL_DUTY;
    motor.enabled = 1;
    motor.initialized = 1;
}

void VESC_Motor_UpdateFeedback(void) {
    if (vesc_status.data_updated) {
        motor.current_rpm = (float)vesc_status.rpm;
        motor.current_current = (float)vesc_status.current / 10.0f;
        motor.current_duty = (float)vesc_status.duty / 1000.0f;
        vesc_status.data_updated = 0;
    }

    if (vesc_status4.data_updated) {
        motor.current_temp_fet = (float)vesc_status4.temp_fet / 10.0f;
        motor.current_temp_motor = (float)vesc_status4.temp_motor / 10.0f;
        vesc_status4.data_updated = 0;
    }
}

void VESC_Motor_ControlUpdate(void) {
    if (!motor.initialized || !motor.enabled) return;

    switch (motor.control_type) {
        case CONTROL_DUTY:
            VESC_SetDuty(motor.target_duty);
            break;

        case CONTROL_CURRENT:
            VESC_SetCurrent(motor.target_current);
            break;

        case CONTROL_RPM:
            VESC_SetRPM(motor.target_rpm);
            break;

        case CONTROL_BRAKE:
            VESC_SetBrakeCurrent(motor.target_brake);
            break;
    }
}


void VESC_Motor_SetTarget(float value) {
    switch (motor.control_type) {
        case CONTROL_DUTY:
            motor.target_duty = value;

            // 严格的占空比限制
            if (motor.target_duty > MAX_SAFE_DUTY) {
                motor.target_duty = MAX_SAFE_DUTY;
            }
            if (motor.target_duty < -MAX_SAFE_DUTY) {
                motor.target_duty = -MAX_SAFE_DUTY;
            }
            break;

        case CONTROL_CURRENT:
            motor.target_current = value;
            break;


        case CONTROL_RPM:
            motor.target_rpm = (int32_t)value;
            break;

        case CONTROL_BRAKE:
            motor.target_brake = value;
            if (motor.target_brake < 0.0f) motor.target_brake = 0.0f;
            break;
    }
}