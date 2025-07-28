#ifndef BLDCMotor_H
#define BLDCMotor_H
#include "MagneticSensor.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "CurrentSensor.h"

typedef struct {
    float inv_voltage_power_supply;
    float voltage_limit;
    int  pole_pairs;
    int8_t sensor_direction;
    float zero_electric_angle;
    GPIO_TypeDef* gpio;
    uint16_t pin;
    uint16_t PWM_Period;
    uint8_t number;
} Motor;

int alignSensor(Motor* Motorx, ENCODER_TypeDef* Encoderx);
void Motor_init(Motor* Motorx, uint8_t number, float vs, float vl,float pp, int8_t sd, float zea, uint32_t periph, GPIO_TypeDef* gpio, uint16_t pin, uint16_t pwm);
void Motor_enable(Motor* Motorx);
void Motor_disable(Motor* Motorx);
float electricalAngle(Motor* Motorx, float shaft_angle);
void setPhaseVoltage(Motor* Motorx, float Uq, float angle_el);
float cal_Iq_Id(float current_a,float current_b,float angle_el);

#endif
