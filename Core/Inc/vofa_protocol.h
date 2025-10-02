#ifndef VOFA_PROTOCOL_H
#define VOFA_PROTOCOL_H

#include "main.h"
#include "vesc_motor.h"

#define CMD_BUFFER_SIZE 128

typedef struct {
    uint8_t rx_buffer[1];
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