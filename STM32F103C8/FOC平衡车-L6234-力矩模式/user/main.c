
#include "stm32f10x.h"
#include <stdlib.h>
#include "usart.h"
#include "delay.h"
#include "timer.h"
#include "iic.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "foc_utils.h"
#include "BLDCmotor.h" 
#include "MagneticSensor.h"
#include "CurrentSensor.h"
#include "MPU6050.h"
#include "NRF24L01.h"
#include "math.h"

uint8_t Buf[32] = {0};
float max_vel = 20; // rad/s
float max_steering = 0.33; // V
float voltage_supply = 8; // V
float voltage_limit = 2; // V
float pole_pairs = 7;
float i2c_freq = 400000; // Hz

void remote_control_run(float* target_vel, float* target_steering);
void commander_run(uint8_t* debug_flag);

int main(void)
{
	Motor Motor1, Motor2;
	ENCODER_TypeDef Encoder1, Encoder2;
	LowPassFilter LPF_M1_vel, LPF_M2_vel;
	LowPassFilter LPF_vel, LPF_pitch, LPF_steering; // 速度，俯仰角，转向
	PIDController PID_vel, PID_pitch; // 速度，俯仰角
	MPU6050_TypeDef MPU;

	NRF24L01_Init();
	uart_init(115200);
	Motor_init(&Motor1, 1, voltage_supply, voltage_limit, pole_pairs, -1, 6.18, RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_11, 1440); // 左轮
	Motor_init(&Motor2, 2, voltage_supply, voltage_limit, pole_pairs, -1, 0.19, RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_12, 1440); // 右轮
	TIM2_PWM_Init(Motor1.PWM_Period);
	TIM3_PWM_Init(Motor2.PWM_Period);
	TIM4_10us_Init();
	MagneticSensor_Init(&Encoder1, 1, AS5600_CPR, I2C1);
	MagneticSensor_Init(&Encoder2, 2, AS5600_CPR, I2C2);
	I2C_Init_(I2C1, i2c_freq);
	I2C_Init_(I2C2, i2c_freq);
	MPU6050_Init(&MPU, I2C2);
	printf("MPU ID: %x\r\n", MPU6050_GetID(&MPU));
	delay_ms(1000);            //Wait for the system to stabilize

	Motor_enable(&Motor1);
	Motor_enable(&Motor2);
  	printf("Motor ready.\r\n");

	if (0) {
		MPU6050_CalcGyroOffsets(&MPU);
		alignSensor(&Motor1, &Encoder1);
		alignSensor(&Motor2, &Encoder2);
		Motor_disable(&Motor1);
		Motor_disable(&Motor2);
		return 0;
	}

	LowPassFilter_init(&LPF_M1_vel);
	LowPassFilter_init(&LPF_M2_vel);
	LowPassFilter_init(&LPF_pitch);
	LowPassFilter_init(&LPF_vel);
	LowPassFilter_init(&LPF_steering);

	PID_Init(&PID_vel, 0.01, 0, 0, 0, 0.2);
	PID_Init(&PID_pitch, 4, 8, 0, 0, voltage_limit);

	float target_vel = 0;
	float target_steering = 0;
	float target_M1_volt = 0, target_M2_volt = 0; 
	int pknt = 0;
	unsigned long timestamp_prev = 0;
	uint8_t debug_flag = 0;
	float max_volt = 0;
	int falldown = 0;
	while(1) { // 0.0012s 一个循环
		// float shaft_velocity_M1 = getVelocity(&Encoder1) * -1; // 大于0是前进
		// float shaft_velocity_M2 = getVelocity(&Encoder2);

		// float lpf_vel_M1 = getLowPassFilter(&LPF_M1_vel, shaft_velocity_M1);
		// float lpf_vel_M2 = getLowPassFilter(&LPF_M2_vel, shaft_velocity_M2);
		
		// float shaft_angle_M1 = Encoder1.angle_vel_prev;
		// float shaft_angle_M2 = Encoder2.angle_vel_prev;
		// float avg_shaft_vel = (lpf_vel_M1 + lpf_vel_M2) * 0.5;

		float shaft_angle_M1 = getAngle(&Encoder1);
		float shaft_angle_M2 = getAngle(&Encoder2);

		float elec_angle_M1 = electricalAngle(&Motor1, shaft_angle_M1);
		float elec_angle_M2 = electricalAngle(&Motor2, shaft_angle_M2);

		MPU6050_Update(&MPU);		//获取MPU6050的数据
		float mpu_pitch = MPU.angleY - 0.05;
		if (fabs(mpu_pitch) > _PI_2) {
			falldown += 1;
			if (falldown > 1000) {
				setPhaseVoltage(&Motor1, 0, elec_angle_M1);
				setPhaseVoltage(&Motor2, 0, elec_angle_M2);
				continue; // 倒地停止
			}
		} else {
			falldown = 0;
		}
		
		float lpf_target_vel = getLowPassFilter(&LPF_vel, target_vel);
		float pid_vel = getPID(&PID_vel, lpf_target_vel - 0);
    	float target_pitch = getLowPassFilter(&LPF_pitch, pid_vel);
		float voltage_control = getPID(&PID_pitch, mpu_pitch - target_pitch);
		float steering_adj = getLowPassFilter(&LPF_steering, target_steering);	

		target_M1_volt = _constrain(voltage_control - steering_adj, -voltage_limit, voltage_limit);
		target_M2_volt = _constrain(-voltage_control - steering_adj, -voltage_limit, voltage_limit);
		
  		setPhaseVoltage(&Motor1, target_M1_volt, elec_angle_M1);
  		setPhaseVoltage(&Motor2, target_M2_volt, elec_angle_M2);

		max_volt = fmax(max_volt, fabs(voltage_control));
		if (debug_flag) {
			unsigned long now_us;
			float Ts;
			now_us = time_cntr;
			if (now_us < timestamp_prev) {
				Ts = (ulong_max_value - timestamp_prev + now_us) * 1e-5;
			} else {
				Ts = (now_us - timestamp_prev) * 1e-5;
			}
			timestamp_prev = now_us;

			if (pknt == 0) {
				printf("%.5f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f,\t %.4f, %.4f, %.4f, %.4f\r\n", 
					Ts, target_vel, lpf_target_vel, target_steering, steering_adj, mpu_pitch, MPU.gyroY, MPU.angleAccY, pid_vel, target_pitch, 
					voltage_control, target_M1_volt, -target_M2_volt, max_volt);
				max_volt = 0;
				pknt = 100;
			}
			pknt -= 1;
		}
		remote_control_run(&target_vel, &target_steering);
		commander_run(&debug_flag);
	}
}

void remote_control_run(float* target_vel, float* target_steering) {
	uint16_t X, Y;
	if (NRF24L01_Get_Value_Flag() == 0) {
		NRF24L01_GetRxBuf(Buf);
		X = Buf[1] * 256 + Buf[0];
		Y = Buf[3] * 256 + Buf[2];
		if (Y < 2500 && Y > 1500) Y = 2048;
		if (Y != 2048) { // 前进和转向分开
			if (X < 2500 && X > 1500) X = 2048;
		}
		*target_vel = -(Y - 2048) / 2048.0 * max_vel;
		*target_steering = -(X - 2048) / 2048.0 * max_steering;
	}
}

void commander_run(uint8_t* debug_flag) {
	if((USART_RX_STA&0x8000) != 0)
	{
		switch(USART_RX_BUF[0])
		{
			case 'D': // debug
				*debug_flag = atoi((const char *)(USART_RX_BUF+1));
				printf("debug_flag=%d\r\n", *debug_flag);
				break;
		}
		USART_RX_STA=0;
	}
}
