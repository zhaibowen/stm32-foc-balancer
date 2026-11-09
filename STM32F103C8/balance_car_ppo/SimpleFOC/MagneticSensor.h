#ifndef MAGNETICSENSOR_LIB_H
#define MAGNETICSENSOR_LIB_H

#define  AS5600_Address  0x36
#define  AS5600_CONF_Hi   0x07
#define  RAW_Angle_Hi    0x0C
#define  AS5600_CPR      4096

typedef struct {
	uint8_t number;
    uint8_t status;                 // 检测状态
	long  cpr;                      //编码器分辨率，AS5600=12bit(4096)
	float  cpr2;                     // 简化计算
	float  cpr3;                     // 简化计算
	long  angle_data_prev;          //原始数据, 0~4095
	float angle_prev;           //角度值
	float full_rotation_offset;     //角度累加
	uint32_t timestamp_prev;  //速度计时，用于计算速度
	I2C_TypeDef* i2c;  // i2c通道s
} ENCODER_TypeDef;

void MagneticSensor_Init(ENCODER_TypeDef* Coderx, uint8_t number, long cpr, I2C_TypeDef* i2c);
float getAngle(ENCODER_TypeDef* Coderx);
float getVelocity(ENCODER_TypeDef* Coderx, float shaft_angle);
#endif
