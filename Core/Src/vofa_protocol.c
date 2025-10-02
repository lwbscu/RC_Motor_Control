#include "vofa_protocol.h"
#include <stdio.h>    // 新增：支持 sscanf、snprintf
#include <string.h>
VOFA_Protocol_t vofa = {0};

void VOFA_Init(void) {
    HAL_UART_Receive_IT(&huart1, &vofa.rx_buffer[0], 1);
}

void VOFA_ProcessCommand(void) {
    if (!vofa.cmd_ready) return;

    char *cmd = (char*)vofa.cmd_buffer;
    float value1;

    // 控制模式切换
    if (sscanf(cmd, "mode:%f", &value1) == 1) {
        motor.control_type = (uint8_t)value1;
    }

        // 占空比控制 (duty: -1.0 to 1.0)
    else if (sscanf(cmd, "duty:%f", &value1) == 1) {
        motor.control_type = CONTROL_DUTY;
        VESC_Motor_SetTarget(value1);
    }

        // 电流控制 (current: Amperes)
    else if (sscanf(cmd, "current:%f", &value1) == 1) {
        motor.control_type = CONTROL_CURRENT;
        VESC_Motor_SetTarget(value1);
    }

        // 转速控制 (rpm: revolutions per minute)
    else if (sscanf(cmd, "rpm:%f", &value1) == 1) {
        motor.control_type = CONTROL_RPM;
        VESC_Motor_SetTarget(value1);
    }

        // 刹车控制 (brake: Amperes)
    else if (sscanf(cmd, "brake:%f", &value1) == 1) {
        motor.control_type = CONTROL_BRAKE;
        VESC_Motor_SetTarget(value1);
    }

        // 设置目标值（根据当前模式）
    else if (sscanf(cmd, "target:%f", &value1) == 1) {
        VESC_Motor_SetTarget(value1);
    }

        // 使能/禁用控制
    else if (sscanf(cmd, "enable:%f", &value1) == 1) {
        motor.enabled = (uint8_t)value1;
    }

        // 停止电机
    else if (strcmp(cmd, "stop") == 0) {
        motor.target_duty = 0.0f;
        motor.target_current = 0.0f;
        motor.target_rpm = 0;
        motor.target_brake = 0.0f;
    }

    vofa.cmd_ready = 0;
}

void VOFA_SendData(void) {
    char buffer[256];
    int len = snprintf(buffer, sizeof(buffer),
                       "%.3f,%.3f,%.1f,%.2f,%.2f,%.1f,%.1f,%d\n",
                       motor.target_duty,
                       motor.current_duty,
                       motor.current_rpm,
                       motor.target_current,
                       motor.current_current,
                       motor.current_temp_fet,
                       motor.current_temp_motor,
                       motor.control_type
    );
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 100);
}

void VOFA_UART_RxCallback(void) {
    uint8_t received = vofa.rx_buffer[0];

    if (received == '\n') {
        vofa.cmd_buffer[vofa.rx_index] = '\0';
        vofa.cmd_ready = 1;
        vofa.rx_index = 0;
    } else if (vofa.rx_index < CMD_BUFFER_SIZE - 1) {
        vofa.cmd_buffer[vofa.rx_index++] = received;
    }

    HAL_UART_Receive_IT(&huart1, &vofa.rx_buffer[0], 1);
}
