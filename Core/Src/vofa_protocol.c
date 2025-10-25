#include "vofa_protocol.h"
#include <math.h> // 需要包含 math.h 以便使用 fabs

VOFA_Protocol_t vofa = {0};

void VOFA_Init(void) {
    HAL_UART_Receive_IT(&huart1, &vofa.rx_buffer[0], 1);
}

void VOFA_ProcessCommand(void) {
    if (!vofa.cmd_ready) return;

    char *cmd = (char*)vofa.cmd_buffer;
    float value1;

    // PID参数设置 (自动应用于两个电机)
    if (sscanf(cmd, "pos_pid:kp,%f", &value1) == 1) {
        M2006_SetPositionPID(value1, motor[0].position_pid.ki, motor[0].position_pid.kd);
    }
    else if (sscanf(cmd, "pos_pid:ki,%f", &value1) == 1) {
        // M2006_SetPositionPID(motor[0].position_pid.kp, value1, motor[0].position_pid.kd); // KI 已禁用
    }
    else if (sscanf(cmd, "pos_pid:kd,%f", &value1) == 1) {
        M2006_SetPositionPID(motor[0].position_pid.kp, motor[0].position_pid.ki, value1);
    }

        // 运动控制 (自动应用于两个电机)
    else if (sscanf(cmd, "position:%f", &value1) == 1) {
        M2006_SetTargetPosition(value1); // 函数内部会设置 +/- 目标
    }

        // 基础控制
    else if (sscanf(cmd, "dir:%f", &value1) == 1) {
        // 关键修改：禁用此指令，因为方向已在 M2006_Init 中硬编码
        // motor[0].direction = (uint8_t)value1;
        // motor[1].direction = (uint8_t)value1;
    }
    else if (sscanf(cmd, "enable:%f", &value1) == 1) {
        motor[0].enabled = (uint8_t)value1;
        motor[1].enabled = (uint8_t)value1;
    }

    vofa.cmd_ready = 0;
}

// =================================================================
// 关键修改：VOFA_SendData() - 修改通道6为“同步误差”
// =================================================================
void VOFA_SendData(void) {
    char buffer[256];

    // 发送给VOFA+的数据帧 (6个通道)
    // 1: 共同目标位置 (VOFA+输入的原始值, e.g. 5.0)
    // 2: 电机3 (ID 3) 当前位置 (e.g. -5.0)
    // 3: 电机4 (ID 4) 当前位置 (e.g. +5.0)
    // 4: 电机3 PID输出 (电流)
    // 5: 电机4 PID输出 (电流)
    // 6: 同步误差 (motor[0].pos + motor[1].pos), 理想情况为 0

    // motor[0] 的目标是 -target, motor[1] 的目标是 +target
    // motor[0] 的位置是 -value, motor[1] 的位置是 +value
    // 两者相加，理想值为 0。如果不为0，则表示升降不同步。
    float sync_error = motor[0].current_position_turns + motor[1].current_position_turns;

    // 从 VOFA+ 获取的原始目标值 (M2006_SetTargetPosition 会反转它, 我们发送原始值)
    // 我们可以从 motor[1] (正向) 获取原始目标
    float original_target = motor[1].target_position_turns;

    int len = snprintf(buffer, sizeof(buffer),
                       "%.3f,%.3f,%.3f,%.2f,%.2f,%.3f\n",
                       original_target,                // 1. 共同目标 (原始值)
                       motor[0].current_position_turns, // 2. 电机3 (ID 3) 位置 (负)
                       motor[1].current_position_turns, // 3. 电机4 (ID 4) 位置 (正)
                       motor[0].position_pid.output,    // 4. 电机3 PID输出
                       motor[1].position_pid.output,    // 5. 电机4 PID输出
                       sync_error                       // 6. 同步误差 (应为0)
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