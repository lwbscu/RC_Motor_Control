#include "vofa_protocol.h"

VOFA_Protocol_t vofa = {0};

void VOFA_Init(void) {
    HAL_UART_Receive_IT(&huart1, &vofa.rx_buffer[0], 1);
}

void VOFA_ProcessCommand(void) {
    if (!vofa.cmd_ready) return;

    char *cmd = (char*)vofa.cmd_buffer;
    float value1;

    // PID参数设置 (指令不变, 但现在调整的是基于弧度的PID)
    if (sscanf(cmd, "pos_pid:kp,%f", &value1) == 1) { // 指令保持 pos_pid: 不变
        M2006_SetPositionPID(value1, motor.position_pid.ki, motor.position_pid.kd);
    }
    else if (sscanf(cmd, "pos_pid:ki,%f", &value1) == 1) {
        M2006_SetPositionPID(motor.position_pid.kp, value1, motor.position_pid.kd);
    }
    else if (sscanf(cmd, "pos_pid:kd,%f", &value1) == 1) {
        M2006_SetPositionPID(motor.position_pid.kp, motor.position_pid.ki, value1);
    }

        // 运动控制 (输入仍然是 输出轴圈数)
    else if (sscanf(cmd, "position:%f", &value1) == 1) {
        M2006_SetTargetPosition(value1); // 函数内部会做单位转换
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

// =================================================================
// 关键修改：VOFA_SendData() - 发送外部单位
// =================================================================
void VOFA_SendData(void) {
    char buffer[256];

    // 发送给VOFA+的数据帧
    // 1: 目标位置 (输出轴圈数)
    // 2: 当前位置 (输出轴圈数)
    // 3: PID输出 (电流)
    // 4: 当前速度 (输出轴 圈/秒, rps) <- 转换回来方便观察
    float current_speed_rps_output = motor.current_speed_rad_s / (GEAR_RATIO * 2.0f * M_PI);

    int len = snprintf(buffer, sizeof(buffer),
                       "%.3f,%.3f,%.2f,%.3f\n",
                       motor.target_position_turns, // 发送外部目标
                       motor.current_position_turns, // 发送外部位置
                       motor.position_pid.output,    // PID输出电流
                       current_speed_rps_output      // 发送外部速度
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