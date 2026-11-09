#include "stm32f10x.h"
#include <string.h>
#include <stdlib.h>
#include "usart.h"
#include "delay.h"
#include "timer.h"
#include "iic.h"
#include "lowpass_filter.h"
#include "PID.h"
#include "policy.h"
#include "foc_utils.h"
#include "BLDCmotor.h" 
#include "MagneticSensor.h"
#include "CurrentSensor.h"
#include "MPU6050.h"
#include "NRF24L01.h"

float max_vel = 0.3; // m/s
float max_steering = 0.75; // rad/s
float voltage_supply = 8; // V
float voltage_limit = 2; // V
float pole_pairs = 7;
float i2c_freq = 400000; // Hz

uint8_t rxBuf[33] = {0};
uint8_t txBuf[33] = {0};
uint8_t oled_flag[6] = {0};
float raw_obs[12] = {0};
float action[2] = {0};
float target_current[2] = {0};
float target_vel = 0;
float target_steering = 0;
uint32_t tim4_timestamp_prev = 0;
float tim4_time_interval = 0;
float tim4_time_interval2 = 0;
float shaft_angle_M1, shaft_angle_M2;
float shaft_velocity_M1, shaft_velocity_M2;

MPU6050_TypeDef MPU;
LowPassFilter LPF_M1_vel, LPF_M2_vel;
Motor Motor1, Motor2;
ENCODER_TypeDef Encoder1, Encoder2;
CurrentSensor CS1, CS2;
PIController PI1q, PI1d, PI2q, PI2d;

void control_pad_receive(float* target_vel, float* target_steering);
void control_pad_send(float a1, float a2, float a3, float a4, float a5, float a6, float a7, float a8);
void commander_run(uint8_t* debug_flag);

int main(void) {
	NRF24L01_Init();
	uart_init(115200);
	Motor_init(&Motor1, 1, voltage_supply, pole_pairs, -1, 0.10, RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_11, 1440, TIM2); // 左轮
	Motor_init(&Motor2, 2, voltage_supply, pole_pairs, -1, 0.19, RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_12, 1440, TIM3); // 右轮
	TIM2_PWM_Init(Motor1.PWM_Period);
	TIM3_PWM_Init(Motor2.PWM_Period);
    DWT_Init();
	I2C_Init_(I2C1, i2c_freq);
	I2C_Init_(I2C2, i2c_freq);
	MagneticSensor_Init(&Encoder1, 1, AS5600_CPR, I2C1);
	MagneticSensor_Init(&Encoder2, 2, AS5600_CPR, I2C2);
	MPU6050_Init(&MPU, I2C2);
	// printf("MPU ID: %x\r\n", MPU6050_GetID(&MPU));
	delay_ms(500);

    MPU6050_CalcGyroOffsets(&MPU);
    CurrentSensor_Init(&CS1, 1, ADC_Channel_3, ADC_Channel_4, -1, 1, 20, 0.012);
    CurrentSensor_Init(&CS2, 2, ADC_Channel_5, ADC_Channel_6, -1, 1, 20, 0.012);

	PI_Init(&PI1q, 1, 100, voltage_limit);
	PI_Init(&PI1d, 1, 100, voltage_limit);
	PI_Init(&PI2q, 1, 100, voltage_limit);
	PI_Init(&PI2d, 1, 100, voltage_limit);
    
	LowPassFilter_init(&LPF_M1_vel, 0.4);
	LowPassFilter_init(&LPF_M2_vel, 0.4);

	Motor_enable(&Motor1);
	Motor_enable(&Motor2);
  	// printf("Motor ready.\r\n");

	TIM4_1kHz_Init();
    uint8_t calibration = 0;
    while (calibration) {
        // Motor1.zero_electric_angle = 0;
        // Motor2.zero_electric_angle = 0;
        // alignSensor(&Motor1, &Encoder1, txBuf);
        // alignSensor(&Motor2, &Encoder2, txBuf + 16);
        // NRF24L01_SendBuf(txBuf);
        
        // MPU6050_CalcGyroOffsets(&MPU);
        // control_pad_send(MPU6050_GetID(&MPU), MPU.gyroXoffset, MPU.gyroYoffset, MPU.gyroZoffset, 0, 0, 0, MPU.AccZoffset);

        // CurrentSensor_Init(&CS1, 1, ADC_Channel_3, ADC_Channel_4, -1, 1, 20, 0.012);
        // CurrentSensor_Init(&CS2, 2, ADC_Channel_5, ADC_Channel_6, -1, 1, 20, 0.012);
        // control_pad_send(CS1.offset_ia, CS1.offset_ib, 0, 0, CS2.offset_ia, CS2.offset_ib, 0, 0);

        // float angle1 = getAngle(&Encoder1);
        // float angle2 = getAngle(&Encoder2);
        // control_pad_send(angle1, (float)Encoder1.status, Encoder1.full_rotation_offset, Encoder1.cpr3, angle2, Encoder1.angle_data_prev, 0, 0);
        
        delay_ms(100);
    }

    MPU.timestamp_prev = 0;
    Encoder1.timestamp_prev = 0;
    Encoder2.timestamp_prev = 0;
	uint32_t timestamp_prev = 0;
	int pknt = 0;
    while(1) {
        float time_interval = get_time_interval(&timestamp_prev);
		MPU6050_Update(&MPU);		//获取MPU6050的数据

        raw_obs[0] = MPU.GyroX;
        raw_obs[1] = MPU.GyroY;
        raw_obs[2] = MPU.GyroZ;
        raw_obs[3] = MPU.GravX;
        raw_obs[4] = MPU.GravY;
        raw_obs[5] = MPU.GravZ;
        raw_obs[6] = shaft_velocity_M1;
        raw_obs[7] = shaft_velocity_M2;
        raw_obs[8] = action[0];
        raw_obs[9] = action[1];
        raw_obs[10] = target_vel;
        raw_obs[11] = target_steering;
        model_forward(raw_obs, action);
        target_current[0] = action[0] * 2;
        target_current[1] = action[1] * 2;

        control_pad_receive(&target_vel, &target_steering);
        // commander_run(&calibration);
        pknt++;
        if (pknt > 10) {
            control_pad_send(time_interval, shaft_angle_M1, target_vel, action[0], MPU.pitch, shaft_angle_M2, target_steering, action[1]);
            // printf("%.5f, %.4f, %.4f, %.4f,   %.4f, %.4f, %.4f,   %.4f, %.4f, %.4f,   %.4f, %.4f, %.4f,   %.4f, %.4f, %.4f\r\n", time_interval, 
            //     MPU.AccX, MPU.AccY, MPU.AccZ, MPU.GyroX, MPU.GyroY, MPU.GyroZ, MPU.GravX, MPU.GravY, MPU.GravZ, MPU.roll, MPU.pitch, MPU.yaw, MPU.gyroXoffset, MPU.gyroYoffset, MPU.gyroZoffset);
            // printf("%.5f, %.4f, %.4f,   %.4f, %.4f, %.4f,   %.4f, %.4f, %.4f,   %.4f, %.4f,   %.4f, %.4f,   %.4f, %.4f\r\n", time_interval, 
            //     action[0], action[1], raw_obs[0], raw_obs[1], raw_obs[2], raw_obs[3], raw_obs[4], raw_obs[5], raw_obs[6], raw_obs[7], raw_obs[8],
            //     raw_obs[9], raw_obs[10], raw_obs[11]);
            pknt = 0;
        }
    }
}


void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET)
	{
	    TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
        // tim4_time_interval = get_time_interval(&tim4_timestamp_prev);
        
		shaft_angle_M1 = getAngle(&Encoder1);
		shaft_angle_M2 = getAngle(&Encoder2);
		shaft_velocity_M1 = getVelocity(&Encoder1, shaft_angle_M1);
		shaft_velocity_M2 = -getVelocity(&Encoder2, shaft_angle_M2);
    	shaft_velocity_M1 = getLowPassFilter(&LPF_M1_vel, shaft_velocity_M1);
    	shaft_velocity_M2 = getLowPassFilter(&LPF_M2_vel, shaft_velocity_M2);
        
		float elec_angle_M1 = electricalAngle(&Motor1, shaft_angle_M1);
		float elec_angle_M2 = electricalAngle(&Motor2, shaft_angle_M2);
        float cos_angle_M1 = _cos(elec_angle_M1);
        float sin_angle_M1 = _sin(elec_angle_M1);
        float cos_angle_M2 = _cos(elec_angle_M2);
        float sin_angle_M2 = _sin(elec_angle_M2);

		getPhaseCurrents(&CS1);
		getPhaseCurrents(&CS2);
		cal_Iq_Id(&CS1, cos_angle_M1, sin_angle_M1);
		cal_Iq_Id(&CS2, cos_angle_M2, sin_angle_M2);
        
		float vq1 = getPI(&PI1q, target_current[0] - CS1.Iq);
		float vd1 = getPI(&PI1d,                   - CS1.Id);
		float vq2 = getPI(&PI2q, target_current[1] - CS2.Iq);
		float vd2 = getPI(&PI2d,                   - CS2.Id);

  		setPhaseVoltage(&Motor1, vq1, vd1, cos_angle_M1, sin_angle_M1);
  		setPhaseVoltage(&Motor2, vq2, vd2, cos_angle_M2, sin_angle_M2);
        // tim4_time_interval2 = get_time_interval(&tim4_timestamp_prev);
	}
}

void control_pad_receive(float* target_vel, float* target_steering) {
	uint16_t X, Y;
	if (NRF24L01_Get_Value_Flag() == 0) {
		NRF24L01_GetRxBuf(rxBuf);
		X = rxBuf[1] * 256 + rxBuf[0];
		Y = rxBuf[3] * 256 + rxBuf[2];
		if (Y < 2150 && Y > 1950) Y = 2048;
        if (X < 2150 && X > 1950) X = 2048;
		*target_vel = -(Y - 2048) / 2048.0 * max_vel;
		*target_steering = -(X - 2048) / 2048.0 * max_steering;
        if (rxBuf[8] == 0) {
            oled_flag[0] = (oled_flag[0] + 1) % 2;
        }
	}
}

void control_pad_send(float a1, float a2, float a3, float a4, float a5, float a6, float a7, float a8) {
    memcpy(txBuf, &a1, 4);
    memcpy(txBuf + 4, &a2, 4);
    memcpy(txBuf + 8, &a3, 4);
    memcpy(txBuf + 12, &a4, 4);
    memcpy(txBuf + 16, &a5, 4);
    memcpy(txBuf + 20, &a6, 4);
    memcpy(txBuf + 24, &a7, 4);
    memcpy(txBuf + 28, &a8, 4);
    NRF24L01_SendBuf(txBuf);
}

void commander_run(uint8_t* calibration) {
	if((USART_RX_STA&0x8000) != 0)
	{
		switch(USART_RX_BUF[0])
		{
			case 'C': // debug
				*calibration = atoi((const char *)(USART_RX_BUF+1));
				printf("calibration=%d\r\n", *calibration);
				break;
		}
		USART_RX_STA=0;
	}
}
