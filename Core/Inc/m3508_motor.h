#ifndef M3508_MOTOR_H
#define M3508_MOTOR_H

#include "main.h"
#include "pid_controller.h"
#include "can_communication.h"
#include <math.h>

#define GEAR_RATIO 19.0f
#define ENCODER_RESOLUTION 8192.0f

typedef struct {
    float target_speed;
    float current_speed;

    PID_Controller_t speed_pid;

    int16_t output_current;
    uint16_t last_encoder;
    int32_t encoder_rounds;
    uint8_t initialized;
} M3508_Motor_t;

extern M3508_Motor_t motor;

void M3508_Init(void);
void M3508_UpdateFeedback(void);
void M3508_SetTargetSpeed(float target_speed);
void M3508_ControlUpdate(void);

#endif