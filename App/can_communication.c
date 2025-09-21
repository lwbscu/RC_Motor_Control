#include "can_communication.h"

M3508_Motor_Data_t motor_data[4] = {0};

// 声明STM32CubeMX生成的初始化函数
extern void MX_FDCAN1_Init(void);

void CAN_Init(void) {
    FDCAN_FilterTypeDef sFilterConfig;

    // 调用STM32CubeMX生成的FDCAN初始化函数
    // 注意：MX_FDCAN1_Init()已经在main.c中调用过了，这里是额外的过滤器配置

    // 配置CAN过滤器
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = M3508_MOTOR_ID_1;
    sFilterConfig.FilterID2 = M3508_MOTOR_ID_4;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    // 启动FDCAN
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }

    // 激活通知
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }
}

void CAN_SendMotorCommand(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
    FDCAN_TxHeaderTypeDef TxHeader;
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

    // 组装数据：高字节在前，低字节在后
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

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        uint32_t motor_id = RxHeader.Identifier;

        if (motor_id >= M3508_MOTOR_ID_1 && motor_id <= M3508_MOTOR_ID_4) {
            uint8_t motor_index = motor_id - M3508_MOTOR_ID_1;

            // 解析电机数据
            motor_data[motor_index].angle = (RxData[0] << 8) | RxData[1];
            motor_data[motor_index].speed = (RxData[2] << 8) | RxData[3];
            motor_data[motor_index].current = (RxData[4] << 8) | RxData[5];
            motor_data[motor_index].temperature = RxData[6];
        }
    }
}