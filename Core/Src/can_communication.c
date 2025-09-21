#include "can_communication.h"

M3508_Motor_Data_t motor_data[4] = {0};

void CAN_Init(void) {
    FDCAN_FilterTypeDef sFilterConfig;

    // 配置接收所有标准ID的过滤器
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;  // 接收所有ID
    sFilterConfig.FilterID2 = 0x7FF;  // 标准帧最大ID

    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void CAN_SendMotorCommand(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
    FDCAN_TxHeaderTypeDef TxHeader = {0};
    uint8_t TxData[8];

    TxHeader.Identifier = CAN_SEND_ID;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // 高字节在前
    TxData[0] = (motor1 >> 8) & 0xFF;
    TxData[1] = motor1 & 0xFF;
    TxData[2] = (motor2 >> 8) & 0xFF;
    TxData[3] = motor2 & 0xFF;
    TxData[4] = (motor3 >> 8) & 0xFF;
    TxData[5] = motor3 & 0xFF;
    TxData[6] = (motor4 >> 8) & 0xFF;
    TxData[7] = motor4 & 0xFF;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void CAN_ReceiveCallback(FDCAN_HandleTypeDef *hfdcan) {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);
    
    uint32_t motor_id = RxHeader.Identifier;
    if (motor_id >= M3508_MOTOR_ID_1 && motor_id <= M3508_MOTOR_ID_4) {
        uint8_t motor_index = motor_id - M3508_MOTOR_ID_1;

        motor_data[motor_index].angle = (RxData[0] << 8) | RxData[1];
        motor_data[motor_index].speed = (int16_t)((RxData[2] << 8) | RxData[3]);
        motor_data[motor_index].current = (int16_t)((RxData[4] << 8) | RxData[5]);
        motor_data[motor_index].temperature = RxData[6];
        motor_data[motor_index].data_updated = 1;
    }
}