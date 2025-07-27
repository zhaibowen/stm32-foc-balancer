#ifndef MAGNETICSENSOR_LIB_H
#define MAGNETICSENSOR_LIB_H

#define  AS5600_Address  0x36
#define  RAW_Angle_Hi    0x0C   //simpleFOC源码V2.1.1 bugfix
//#define  RAW_Angle_Lo    0x0D
#define  AS5600_CPR      4096

typedef struct 
{
	long  velocity_calc_timestamp;  //速度计时，用于计算速度
	uint8_t number;
	long  cpr;                      //编码器分辨率，AS5600=12bit(4096)
	long  angle_data_prev;          //获取角度用
	float angle_vel_prev;           //获取速度用
	float full_rotation_offset;     //角度累加
	I2C_TypeDef* i2c;  // i2c通道s
} ENCODER_TypeDef;

void MagneticSensor_Init(ENCODER_TypeDef* Coderx, uint8_t number, long cpr, I2C_TypeDef* i2c);
float getAngle(ENCODER_TypeDef* Coderx);
float getVelocity(ENCODER_TypeDef* Coderx);

#endif
