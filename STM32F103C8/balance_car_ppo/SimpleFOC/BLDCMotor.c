#include "stm32f10x.h"
#include <string.h>
#include "foc_utils.h"
#include "usart.h"
#include "delay.h"
#include "math.h"
#include "BLDCMotor.h"
#include "NRF24L01.h"

void Motor_init(Motor* Motorx, uint8_t number, float vs, float pp, int8_t sd, float zea, uint32_t periph, GPIO_TypeDef* gpio, uint16_t pin, uint16_t pwm, TIM_TypeDef* TIMx) {
	Motorx->number = number;
	Motorx->inv_voltage_power_supply = 1.0 / vs;
	Motorx->pole_pairs = pp;
	Motorx->sensor_direction = sd;
	Motorx->zero_electric_angle = zea;
	Motorx->gpio = gpio;
	Motorx->pin = pin;
	Motorx->PWM_Period = pwm;
    Motorx->TIMx = TIMx;

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(periph, ENABLE);
	GPIO_InitStructure.GPIO_Pin = pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(gpio, &GPIO_InitStructure);
	GPIO_ResetBits(gpio, pin); // disable
}

void Motor_enable(Motor* Motorx) {
	GPIO_SetBits(Motorx->gpio, Motorx->pin);
}

void Motor_disable(Motor* Motorx) {
	GPIO_ResetBits(Motorx->gpio, Motorx->pin);
}

float electricalAngle(Motor* Motorx, float shaft_angle){
  return _normalizeAngle((float)(Motorx->sensor_direction * Motorx->pole_pairs) * shaft_angle - Motorx->zero_electric_angle);
}

void setPhaseVoltage(Motor* Motorx, float Uq, float Ud, float ct, float st) {
    float Ualpha = Ud * ct - Uq * st;
    float Ubeta  = Ud * st + Uq * ct;

	float Ua = Ualpha * Motorx->inv_voltage_power_supply + 0.5;
	float Ub = 0.5 * (-Ualpha + _SQRT3 * Ubeta) * Motorx->inv_voltage_power_supply + 0.5;
	float Uc = 0.5 * (-Ualpha - _SQRT3 * Ubeta) * Motorx->inv_voltage_power_supply + 0.5;

	Ua = _constrain(Ua, 0, 1);
	Ub = _constrain(Ub, 0, 1);
	Uc = _constrain(Uc, 0, 1);

	TIM_SetCompare1(TIM2, Ua * Motorx->PWM_Period);
	TIM_SetCompare2(TIM2, Ub * Motorx->PWM_Period);
	TIM_SetCompare3(TIM2, Uc * Motorx->PWM_Period);
}

int alignSensor(Motor* Motorx, ENCODER_TypeDef* Encoderx, uint8_t* txBuf) {
    float cos_angle = _cos(_3PI_2);
    float sin_angle = _sin(_3PI_2);
	setPhaseVoltage(Motorx, 0.5, 0, cos_angle, sin_angle);  //计算零点偏移角度
	delay_ms(500);
    float shaft_angle = _normalizeAngle(getAngle(Encoderx));
	float zero_electric_angle = _normalizeAngle(_electricalAngle(Motorx->sensor_direction * shaft_angle, Motorx->pole_pairs));
    if (zero_electric_angle > 3) {
        zero_electric_angle -= _2PI;
    }
	delay_ms(500);

	float num = Motorx->number;
    memcpy(txBuf, &num, 4);
    memcpy(txBuf + 4, &shaft_angle, 4);
    memcpy(txBuf + 8, &zero_electric_angle, 4);
    memcpy(txBuf + 12, &Motorx->zero_electric_angle, 4);
	
	return 1;
}
