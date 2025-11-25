#ifndef MYMOTOR_H
#define MYMOTOR_H

#include "main.h"

typedef struct {
    TIM_HandleTypeDef *htim; // Timer handle for speed control
    uint32_t channel;        // Timer channel for speed control
    GPIO_TypeDef *dirPort;   // GPIO port for direction control
    uint16_t dirPin;         // GPIO pin for direction control
    uint32_t speed;          // Motor speed
    uint8_t dir;             // Motor direction
    GPIO_TypeDef *enPort;    // GPIO port for enable control
    uint16_t enPin;          // GPIO pin for enable control
    GPIO_TypeDef *speedPort; // GPIO port for speed control
    uint16_t speedPin;       // GPIO pin for speed control
} MyMotor;

void MyMotor_Init(MyMotor *motor, TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *dirPort, uint16_t dirPin, GPIO_TypeDef *enPort, uint16_t enPin, GPIO_TypeDef *speedPort, uint16_t speedPin);
void MyMotor_Start(MyMotor *motor);
void MyMotor_Stop(MyMotor *motor);
void MyMotor_Brake(MyMotor *motor);
void MyMotor_SetSpeed(MyMotor *motor, uint32_t speed);
void MyMotor_SetDirection(MyMotor *motor, uint8_t dir);
#endif // MYMOTOR_H