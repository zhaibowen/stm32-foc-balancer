#ifndef BLDCMotor_H
#define BLDCMotor_H
#include "MagneticSensor.h"
#include "CurrentSensor.h"

typedef struct {
    float inv_voltage_power_supply;
    int  pole_pairs;
    int8_t sensor_direction;
    float zero_electric_angle;
    GPIO_TypeDef* gpio;
    uint16_t pin;
    uint16_t PWM_Period;
    uint8_t number;
    TIM_TypeDef* TIMx;
} Motor;

int alignSensor(Motor* Motorx, ENCODER_TypeDef* Encoderx, uint8_t* txBuf);
void Motor_init(Motor* Motorx, uint8_t number, float vs, float pp, int8_t sd, float zea, uint32_t periph, GPIO_TypeDef* gpio, uint16_t pin, uint16_t pwm, TIM_TypeDef* TIMx);
void Motor_enable(Motor* Motorx);
void Motor_disable(Motor* Motorx);
float electricalAngle(Motor* Motorx, float shaft_angle);
void setPhaseVoltage(Motor* Motorx, float Uq, float Ud, float ct, float st);

#endif
