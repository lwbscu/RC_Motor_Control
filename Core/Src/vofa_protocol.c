#include "vofa_protocol.h"

VOFA_Protocol_t vofa = {0};

void VOFA_Init(void) {
    HAL_UART_Receive_IT(&huart1, &vofa.rx_buffer[0], 1);
}

void VOFA_ProcessCommand(void) {
    if (!vofa.cmd_ready) return;

    char *cmd = (char*)vofa.cmd_buffer;
    float value1, value2;

    // 关键修改：重命名 M3508_ -> M2006_
    // PID参数设置
    if (sscanf(cmd, "pid:kp,%f", &value1) == 1) {
        M2006_SetSpeedPID(value1, motor.speed_pid.ki, motor.speed_pid.kd);
    }
    else if (sscanf(cmd, "pid:ki,%f", &value1) == 1) {
        M2006_SetSpeedPID(motor.speed_pid.kp, value1, motor.speed_pid.kd);
    }
    else if (sscanf(cmd, "pid:kd,%f", &value1) == 1) {
        M2006_SetSpeedPID(motor.speed_pid.kp, motor.speed_pid.ki, value1);
    }

        // 位置PID参数
    else if (sscanf(cmd, "pos_pid:kp,%f", &value1) == 1) {
        M2006_SetPositionPID(value1, motor.position_pid.ki, motor.position_pid.kd);
    }
    else if (sscanf(cmd, "pos_pid:ki,%f", &value1) == 1) {
        M2006_SetPositionPID(motor.position_pid.kp, value1, motor.position_pid.kd);
    }
    else if (sscanf(cmd, "pos_pid:kd,%f", &value1) == 1) {
        M2006_SetPositionPID(motor.position_pid.kp, motor.position_pid.ki, value1);
    }

        // 控制模式
    else if (sscanf(cmd, "control_mode:%f", &value1) == 1) {
        motor.control_source = (uint8_t)value1;
    }
    else if (sscanf(cmd, "control_type:%f", &value1) == 1) {
        motor.control_type = (uint8_t)value1;
    }

        // 运动控制
    else if (sscanf(cmd, "speed:%f", &value1) == 1) {
        M2006_SetTargetSpeed(value1);
        motor.control_type = CONTROL_SPEED;
    }
    else if (sscanf(cmd, "position:%f", &value1) == 1) {
        M2006_SetTargetPosition(value1);
        motor.control_type = CONTROL_POSITION;
    }
    else if (sscanf(cmd, "cascade:%f,%f", &value1, &value2) == 2) {
        M2006_SetCascadeTarget(value1, value2);
        motor.control_type = CONTROL_CASCADE;
    }

        // 串级控制专用命令 - 只设置参数，不切换模式
    else if (sscanf(cmd, "type3_speed:%f", &value1) == 1) {
        M2006_SetTargetSpeed(value1);
    }
    else if (sscanf(cmd, "type3_pos:%f", &value1) == 1) {
        M2006_SetTargetPosition(value1);
    }
    else if (sscanf(cmd, "cascade3_enable:%f", &value1) == 1) {
        if (value1 > 0) {
            motor.enabled = 1;
            motor.control_type = CONTROL_CASCADE;
        } else {
            motor.enabled = 0;
        }
    }

        // 档位控制
    else if (sscanf(cmd, "speed_mode:%f", &value1) == 1) {
        float speeds[] = {0.5f, 1.0f, 2.0f};
        if (value1 >= 0 && value1 <= 2) {
            M2006_SetTargetSpeed(speeds[(int)value1]);
            motor.control_type = CONTROL_SPEED;
        }
    }
    else if (sscanf(cmd, "position_mode:%f", &value1) == 1){
        M2006_SetTargetPosition(value1);
        motor.control_type = CONTROL_POSITION;
    }

        // 基础控制
    else if (sscanf(cmd, "dir:%f", &value1) == 1) {
        motor.direction = (uint8_t)value1;
    }
    else if (sscanf(cmd, "enable:%f", &value1) == 1) {
        motor.enabled = (uint8_t)value1;
    }

    vofa.cmd_ready = 0;
}

void VOFA_SendData(void) {
    char buffer[256];
    int len = snprintf(buffer, sizeof(buffer),
                       "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                       motor.target_speed,
                       motor.current_speed,
                       motor.target_position,
                       motor.current_position,
                       motor.speed_pid.output,
                       motor.position_pid.output,
                       (float)motor.output_current,
                       (float)motor.control_type
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