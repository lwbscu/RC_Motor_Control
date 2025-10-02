#include "vesc_communication.h"

VESC_Status_t vesc_status = {0};
VESC_Status4_t vesc_status4 = {0};

void VESC_CAN_Init(void) {
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x1FFFFFFF;

    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void VESC_SetDuty(float duty) {
    FDCAN_TxHeaderTypeDef TxHeader = {0};
    uint8_t TxData[8] = {0};

    int32_t duty_value = (int32_t)(duty * 100000.0f);

    TxHeader.Identifier = (CAN_PACKET_SET_DUTY << 8) | VESC_CONTROLLER_ID;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_4;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // 大端序（Big-Endian）：MSB在前
    TxData[0] = (duty_value >> 24) & 0xFF;
    TxData[1] = (duty_value >> 16) & 0xFF;
    TxData[2] = (duty_value >> 8) & 0xFF;
    TxData[3] = (duty_value >> 0) & 0xFF;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void VESC_SetCurrent(float current) {
    FDCAN_TxHeaderTypeDef TxHeader = {0};
    uint8_t TxData[8] = {0};

    int32_t current_value = (int32_t)(current * 1000.0f);

    TxHeader.Identifier = (CAN_PACKET_SET_CURRENT << 8) | VESC_CONTROLLER_ID;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_4;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // 大端序
    TxData[0] = (current_value >> 24) & 0xFF;
    TxData[1] = (current_value >> 16) & 0xFF;
    TxData[2] = (current_value >> 8) & 0xFF;
    TxData[3] = (current_value >> 0) & 0xFF;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void VESC_SetRPM(int32_t rpm) {
    FDCAN_TxHeaderTypeDef TxHeader = {0};
    uint8_t TxData[8] = {0};

    TxHeader.Identifier = (CAN_PACKET_SET_RPM << 8) | VESC_CONTROLLER_ID;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_4;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // 大端序
    TxData[0] = (rpm >> 24) & 0xFF;
    TxData[1] = (rpm >> 16) & 0xFF;
    TxData[2] = (rpm >> 8) & 0xFF;
    TxData[3] = (rpm >> 0) & 0xFF;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void VESC_SetBrakeCurrent(float current) {
    FDCAN_TxHeaderTypeDef TxHeader = {0};
    uint8_t TxData[8] = {0};

    int32_t current_value = (int32_t)(current * 1000.0f);

    TxHeader.Identifier = (CAN_PACKET_SET_CURRENT_BRAKE << 8) | VESC_CONTROLLER_ID;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_4;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // 大端序
    TxData[0] = (current_value >> 24) & 0xFF;
    TxData[1] = (current_value >> 16) & 0xFF;
    TxData[2] = (current_value >> 8) & 0xFF;
    TxData[3] = (current_value >> 0) & 0xFF;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void VESC_ReceiveCallback(FDCAN_HandleTypeDef *hfdcan) {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);

    uint32_t packet_type = (RxHeader.Identifier >> 8) & 0xFFFFFF;
    uint8_t controller_id = RxHeader.Identifier & 0xFF;

    if (controller_id != VESC_CONTROLLER_ID) return;

    if (packet_type == CAN_PACKET_STATUS) {
        // 大端序解析：data[0]是MSB
        vesc_status.rpm = (int32_t)((RxData[0] << 24) | (RxData[1] << 16) |
                                    (RxData[2] << 8) | (RxData[3] << 0));
        vesc_status.current = (int16_t)((RxData[4] << 8) | (RxData[5] << 0));
        vesc_status.duty = (int16_t)((RxData[6] << 8) | (RxData[7] << 0));
        vesc_status.data_updated = 1;
    }
    else if (packet_type == CAN_PACKET_STATUS_4) {
        // 大端序解析
        vesc_status4.temp_fet = (int16_t)((RxData[0] << 8) | (RxData[1] << 0));
        vesc_status4.temp_motor = (int16_t)((RxData[2] << 8) | (RxData[3] << 0));
        vesc_status4.current_in = (int16_t)((RxData[4] << 8) | (RxData[5] << 0));
        vesc_status4.pid_pos = (int16_t)((RxData[6] << 8) | (RxData[7] << 0));
        vesc_status4.data_updated = 1;
    }
}