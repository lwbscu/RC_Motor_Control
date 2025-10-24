#include "can_communication.h"

M3508_Motor_Data_t motor_data = {0}; // 保持不变

void CAN_Init(void) {
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x7FF;

    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void CAN_SendMotorCommand(int16_t current) {
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

    // C610 协议: 0x200 控制 1-4号电机
    // 字节[6][7] 对应 4号电机 (ID 0x204)
    // 此逻辑与C620兼容，无需修改
    TxData[6] = (current >> 8) & 0xFF;
    TxData[7] = current & 0xFF;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void CAN_ReceiveCallback(FDCAN_HandleTypeDef *hfdcan) {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);

    // 关键修改：检查 M2006_MOTOR_ID
    if (RxHeader.Identifier == M2006_MOTOR_ID) { // 0x204
        // C610 反馈帧格式:
        // [0][1]: 角度 (0-8191)
        // [2][3]: 速度 (RPM)
        // [4][5]: 实际电流
        // [6]: 温度
        motor_data.angle = (RxData[0] << 8) | RxData[1];
        motor_data.speed = (int16_t)((RxData[2] << 8) | RxData[3]);
        motor_data.current = (int16_t)((RxData[4] << 8) | RxData[5]);
        motor_data.temperature = RxData[6];
        motor_data.data_updated = 1;
    }
}