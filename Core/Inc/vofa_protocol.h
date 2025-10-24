#ifndef VOFA_PROTOCOL_H
#define VOFA_PROTOCOL_H

#include "main.h"
// 关键修改：包含新的头文件
#include "m2006_motor.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define UART_RX_BUFFER_SIZE 256
#define CMD_BUFFER_SIZE 128

extern UART_HandleTypeDef huart1;

typedef struct {
    uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
    uint8_t cmd_buffer[CMD_BUFFER_SIZE];
    uint16_t rx_index;
    uint8_t cmd_ready;
} VOFA_Protocol_t;

extern VOFA_Protocol_t vofa;

void VOFA_Init(void);
void VOFA_ProcessCommand(void);
void VOFA_SendData(void);
void VOFA_UART_RxCallback(void);

#endif