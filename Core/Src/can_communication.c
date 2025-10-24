#include "can_communication.h"

M3508_Motor_Data_t motor_data = {0};

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

    TxHeader.Identifier = CAN_SEND_ID;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // 只设置第4个电机的电流值（对应0x204）
    TxData[6] = (current >> 8) & 0xFF;
    TxData[7] = current & 0xFF;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void CAN_ReceiveCallback(FDCAN_HandleTypeDef *hfdcan) {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);

    if (RxHeader.Identifier == M3508_MOTOR_ID) {
        motor_data.angle = (RxData[0] << 8) | RxData[1];
        motor_data.speed = (int16_t)((RxData[2] << 8) | RxData[3]);
        motor_data.current = (int16_t)((RxData[4] << 8) | RxData[5]);
        motor_data.temperature = RxData[6];
        motor_data.data_updated = 1;
    }
}