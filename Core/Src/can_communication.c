#include "can_communication.h"

// 关键修改：初始化 motor_data 数组
M2006_Motor_Data_t motor_data[2] = {0};

void CAN_Init(void) {
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    // 关键修改：我们将过滤 0x203 和 0x204，因此过滤器范围可以设得更窄
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x203; // 监听的起始ID
    sFilterConfig.FilterID2 = 0x204; // 监听的结束ID (只包含 203 和 204)

    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

// 关键修改：实现同时发送两个电机电流的函数
void CAN_SendMotorCommand(int16_t current_id3, int16_t current_id4) {
    FDCAN_TxHeaderTypeDef TxHeader = {0};
    uint8_t TxData[8] = {0};

    TxHeader.Identifier = CAN_SEND_ID; // 0x200
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // C610 协议: 0x200 控制 1-4号电机 [cite: 147]
    // 字节[4][5] 对应 3号电机 (ID 0x203)
    TxData[4] = (current_id3 >> 8) & 0xFF;
    TxData[5] = current_id3 & 0xFF;

    // 字节[6][7] 对应 4号电机 (ID 0x204)
    TxData[6] = (current_id4 >> 8) & 0xFF;
    TxData[7] = current_id4 & 0xFF;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

// 关键修改：实现对两个ID的接收回调
void CAN_ReceiveCallback(FDCAN_HandleTypeDef *hfdcan) {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);

    uint8_t motor_index = 0; // 0 for ID 3, 1 for ID 4

    // 检查是哪个电机ID
    if (RxHeader.Identifier == MOTOR_ID_3) { // 0x203
        motor_index = 0;
    } else if (RxHeader.Identifier == MOTOR_ID_4) { // 0x204
        motor_index = 1;
    } else {
        return; // 不是我们关心的ID
    }

    // C610 反馈帧格式: [cite: 158]
    // [0][1]: 角度 (0-8191)
    // [2][3]: 速度 (RPM)
    // [4][5]: 实际电流
    // [6]: 温度
    motor_data[motor_index].angle = (RxData[0] << 8) | RxData[1];
    motor_data[motor_index].speed = (int16_t)((RxData[2] << 8) | RxData[3]);
    motor_data[motor_index].current = (int16_t)((RxData[4] << 8) | RxData[5]);
    motor_data[motor_index].temperature = RxData[6];
    motor_data[motor_index].data_updated = 1; // 标记数据已更新
}