#include "stm32f10x.h"
#include "foc_utils.h"
#include "timer.h"
#include "usart.h"
#include "delay.h"
#include "math.h"
#include "BLDCMotor.h"

void Motor_init(Motor* Motorx, uint8_t number, float vs, float vl, float pp, int8_t sd, float zea, uint32_t periph, GPIO_TypeDef* gpio, uint16_t pin, uint16_t pwm) {
	Motorx->number = number;
	Motorx->inv_voltage_power_supply = 1.0 / vs;
	Motorx->voltage_limit = vl;
	Motorx->pole_pairs = pp;
	Motorx->sensor_direction = sd;
	Motorx->zero_electric_angle = zea;
	Motorx->gpio = gpio;
	Motorx->pin = pin;
	Motorx->PWM_Period = pwm;

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

void setPhaseVoltage(Motor* Motorx, float Uq, float angle_el) {
	if (Uq < 0) {
		angle_el += _PI;
		Uq = fabs(Uq);
	}

	angle_el = _normalizeAngle(angle_el + _PI_2);
	int sector = floor(angle_el * _inv_PI_3) + 1;
	// calculate the duty cycles
	float T1 = _SQRT3 * _sin(sector * _PI_3 - angle_el) * Uq * Motorx->inv_voltage_power_supply;
	float T2 = _SQRT3 * _sin(angle_el - (sector - 1.0) * _PI_3) * Uq * Motorx->inv_voltage_power_supply;
	float T0 = 1 - T1 - T2;

	float Ta, Tb, Tc;
	switch (sector){
		case 1:
			Ta = T1 + T2 + T0 * 0.5;
			Tb = T2 + T0 * 0.5;
			Tc = T0 * 0.5;
			break;
		case 2:
			Ta = T1 + T0 * 0.5;
			Tb = T1 + T2 + T0 * 0.5;
			Tc = T0 * 0.5;
			break;
		case 3:
			Ta = T0 * 0.5;
			Tb = T1 + T2 + T0 * 0.5;
			Tc = T2 + T0 * 0.5;
			break;
		case 4:
			Ta = T0 * 0.5;
			Tb = T1 + T0 * 0.5;
			Tc = T1 + T2 + T0 * 0.5;
			break;
		case 5:
			Ta = T2 + T0 * 0.5;
			Tb = T0 * 0.5;
			Tc = T1 + T2 + T0 * 0.5;
			break;
		case 6:
			Ta = T1 + T2 + T0 * 0.5;
			Tb = T0 * 0.5;
			Tc = T1 + T0 * 0.5;
			break;
		default:
			Ta = 0;
			Tb = 0;
			Tc = 0;
	}

	Ta = _constrain(Ta, 0, 1);
	Tb = _constrain(Tb, 0, 1);
	Tc = _constrain(Tc, 0, 1);

	if (Motorx->number == 1) {
		TIM_SetCompare1(TIM2, Ta * Motorx->PWM_Period);
		TIM_SetCompare2(TIM2, Tb * Motorx->PWM_Period);
		TIM_SetCompare3(TIM2, Tc * Motorx->PWM_Period);
	} else if (Motorx->number == 2) {
		TIM_SetCompare2(TIM3, Ta * Motorx->PWM_Period);
		TIM_SetCompare3(TIM3, Tb * Motorx->PWM_Period);
		TIM_SetCompare4(TIM3, Tc * Motorx->PWM_Period);
	}
}

//通过Ia,Ib,Ic计算Iq,Id(目前仅输出Iq)
float cal_Iq_Id(float current_a,float current_b,float angle_el) {
  float I_alpha=current_a;
  float I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;

  float ct = _cos(angle_el);
  float st = _sin(angle_el);
  float I_q = I_beta * ct - I_alpha * st;
  return I_q;
}

int alignSensor(Motor* Motorx, ENCODER_TypeDef* Encoderx) {
	long i;
	float angle;
	float mid_angle,end_angle;
	float moved;
	float pole_error;
	
	printf("MOT: Align sensor.\r\n");
	
	// find natural direction
	// move one electrical revolution forward
	for(i=0; i<=500; i++)
	{
		angle = _3PI_2 + _2PI * i / 500.0;
		setPhaseVoltage(Motorx, Motorx->voltage_limit, angle);
		delay_ms(2);
	}
	delay_ms(200);
	mid_angle=getAngle(Encoderx);
	
	for(i=500; i>=0; i--) 
	{
		angle = _3PI_2 + _2PI * i / 500.0 ;
		setPhaseVoltage(Motorx, Motorx->voltage_limit, angle);
		delay_ms(2);
	}
	delay_ms(200);
	end_angle=getAngle(Encoderx);
	setPhaseVoltage(Motorx, 0, 0);
	delay_ms(200);
	
	printf("mid_angle=%.4f\r\n",mid_angle);
	printf("end_angle=%.4f\r\n",end_angle);
	
	moved =  fabs(mid_angle - end_angle);
	if((mid_angle == end_angle)||(moved < 0.02))  //相等或者几乎没有动
	{
		printf("MOT: Failed to notice movement loop.\r\n");
		return 0;
	}
	else if(mid_angle < end_angle)
	{
		printf("MOT: sensor_direction = -1 (CCW)\r\n");
		Motorx->sensor_direction = -1;
	}
	else
	{
		printf("MOT: sensor_direction = 1 (CW)\r\n");
		Motorx->sensor_direction = 1;
	}
	
	printf("MOT: PP check: ");    //计算Pole_Pairs
	pole_error = fabs(moved * Motorx->pole_pairs - _2PI);
	if(pole_error > 0.5)
	{
		printf("fail error is %.4f\r\n - estimated pp:", pole_error);
		Motorx->pole_pairs = _2PI / moved + 0.5;     //浮点数转整形，四舍五入
		printf("%d\r\n", Motorx->pole_pairs);
  	} else {
		printf("OK! error is %.4f\r\n", pole_error);
	}
	
	setPhaseVoltage(Motorx, Motorx->voltage_limit, _3PI_2);  //计算零点偏移角度
	delay_ms(700);
	Motorx->zero_electric_angle = _normalizeAngle(_electricalAngle(Motorx->sensor_direction * getAngle(Encoderx), Motorx->pole_pairs));
	delay_ms(20);
	printf("MOT: Zero elec. angle:");
	printf("%.4f\r\n", Motorx->zero_electric_angle);
	
	setPhaseVoltage(0, 0, 0);
	delay_ms(200);
	
	return 1;
}
