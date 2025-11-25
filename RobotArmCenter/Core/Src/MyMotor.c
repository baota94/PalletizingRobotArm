#include "MyMotor.h"
#include "math.h"
#define M_PI 3.14159265358979323846

void MyMotor_Init(MyMotor *motor, TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *dirPort, uint16_t dirPin, GPIO_TypeDef *enPort, uint16_t enPin, GPIO_TypeDef *speedPort, uint16_t speedPin)
{
    motor->htim = htim;
    motor->channel = channel;
    motor->dirPort = dirPort;
    motor->dirPin = dirPin;
    motor->speed = 0;
    motor->dir = 0;
    motor->enPort = dirPort;
    motor->enPin = dirPin;
    motor->speedPort = speedPort;
    motor->speedPin = speedPin;
    HAL_TIM_PWM_Start(motor->htim, motor->channel);
    HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->enPort, motor->enPin, GPIO_PIN_SET);
}
void MyMotor_Start(MyMotor *motor)
{
    HAL_GPIO_WritePin(motor->enPort, motor->enPin, GPIO_PIN_SET);
}
void MyMotor_Stop(MyMotor *motor)
{
    HAL_GPIO_WritePin(motor->enPort, motor->enPin, GPIO_PIN_RESET);
}
void MyMotor_Brake(MyMotor *motor)
{
    HAL_TIM_PWM_Stop(motor->htim, motor->channel);
    HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->enPort, motor->enPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->speedPort, motor->speedPin, GPIO_PIN_SET);
}
void MyMotor_SetSpeed(MyMotor *motor, uint32_t speed)
{
    motor->speed = speed;
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, motor->speed);
}
void MyMotor_SetDirection(MyMotor *motor, uint8_t dir)
{
    motor->dir = dir;
    HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, motor->dir);
}
