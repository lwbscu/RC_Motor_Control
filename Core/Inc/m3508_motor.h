#ifndef M3508_MOTOR_H
#define M3508_MOTOR_H

#include "main.h"
#include "pid_controller.h"
#include "can_communication.h"
#include <math.h>

#define MOTOR_COUNT 4
#define GEAR_RATIO 19.0f    
#define ENCODER_RESOLUTION 8192.0f  

typedef struct {
    uint8_t id;                    
    float target_angle;            
    float target_speed;            
    float current_angle;           
    float current_speed;           
    float total_angle;             

    PID_Controller_t speed_pid;    
    PID_Controller_t position_pid; 

    int16_t output_current;        
    uint16_t last_encoder;         
    int32_t encoder_rounds;        
    uint8_t initialized;           // 初始化标志
} M3508_Motor_t;

extern M3508_Motor_t motors[MOTOR_COUNT];

void M3508_Init(void);
void M3508_UpdateFeedback(uint8_t motor_id);
void M3508_SetTargetAngle(uint8_t motor_id, float target_angle);
void M3508_SetTargetSpeed(uint8_t motor_id, float target_speed);
void M3508_ControlUpdate(void);

#endif