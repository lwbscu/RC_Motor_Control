#include "vofa_protocol.h"

VOFA_Protocol_t vofa = {0};

void VOFA_Init(void) {
    HAL_UART_Receive_IT(&huart1, &vofa.rx_buffer[0], 1);
}

void VOFA_ProcessCommand(void) {
    if (!vofa.cmd_ready) return;

    char *cmd = (char*)vofa.cmd_buffer;
    float value1;

    // --- 移除速度环PID参数设置 ---
    // if (sscanf(cmd, "pid:kp,%f", &value1) == 1) { ... }
    // else if (sscanf(cmd, "pid:ki,%f", &value1) == 1) { ... }
    // else if (sscanf(cmd, "pid:kd,%f", &value1) == 1) { ... }

    // 位置PID参数 (保留)
    if (sscanf(cmd, "pos_pid:kp,%f", &value1) == 1) {
        M3508_SetPositionPID(value1, motor.position_pid.ki, motor.position_pid.kd);
    }
    else if (sscanf(cmd, "pos_pid:ki,%f", &value1) == 1) {
        M3508_SetPositionPID(motor.position_pid.kp, value1, motor.position_pid.kd);
    }
    else if (sscanf(cmd, "pos_pid:kd,%f", &value1) == 1) {
        M3508_SetPositionPID(motor.position_pid.kp, motor.position_pid.ki, value1);
    }

        // 控制模式 (保留，用于切换到位置模式)
    else if (sscanf(cmd, "control_mode:%f", &value1) == 1) {
        motor.control_source = (uint8_t)value1;
    }
    else if (sscanf(cmd, "control_type:%f", &value1) == 1) {
        motor.control_type = (uint8_t)value1;
    }

        // --- 运动控制 (移除speed和cascade) ---
        // else if (sscanf(cmd, "speed:%f", &value1) == 1) { ... }
    else if (sscanf(cmd, "position:%f", &value1) == 1) {
        M3508_SetTargetPosition(value1);
        motor.control_type = CONTROL_POSITION; // 自动切换到位置模式
    }
        // else if (sscanf(cmd, "cascade:%f,%f", &value1, &value2) == 2) { ... }

        // --- 档位控制 (移除speed_mode) ---
        // else if (sscanf(cmd, "speed_mode:%f", &value1) == 1) { ... }
    else if (sscanf(cmd, "position_mode:%f", &value1) == 1){
        M3508_SetTargetPosition(value1);
        motor.control_type = CONTROL_POSITION; // 自动切换到位置模式
    }

        // 基础控制 (保留)
    else if (sscanf(cmd, "dir:%f", &value1) == 1) {
        motor.direction = (uint8_t)value1;
    }
    else if (sscanf(cmd, "enable:%f", &value1) == 1) {
        motor.enabled = (uint8_t)value1;
    }

    vofa.cmd_ready = 0;
}

void VOFA_SendData(void) {
    char buffer[128]; // 缓冲区可以减小

    // 移除 target_speed, current_speed, speed_pid.output
    int len = snprintf(buffer, sizeof(buffer),
                       "%.4f,%.4f,%.2f,%.2f,%.2f\n",
                       motor.target_position,     // 1. 目标位置（圈）
                       motor.current_position,    // 2. 当前位置（圈）
                       motor.position_pid.output, // 3. 位置环PID原始输出（电流）
                       (float)motor.output_current, // 4. 最终输出电流
                       (float)motor.control_type    // 5. 当前控制模式
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