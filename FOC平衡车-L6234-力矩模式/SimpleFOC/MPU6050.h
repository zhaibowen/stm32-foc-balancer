#ifndef __MPU6050_H
#define __MPU6050_H

typedef struct 
{
	float accCoef, gyroCoef;
	float gyro_config, accel_config;
	int16_t rawAccX, rawAccY, rawAccZ, rawGyroY;
	float gyroYoffset;
  	float angleY, gyroY, angleAccY;
  	unsigned long preInterval;
	I2C_TypeDef* i2c;
} MPU6050_TypeDef;

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data, I2C_TypeDef* I2Cx);
uint8_t MPU6050_ReadReg(uint8_t RegAddress, I2C_TypeDef* I2Cx);

void MPU6050_Init(MPU6050_TypeDef* MPU, I2C_TypeDef* I2Cx);
void MPU6050_CalcGyroOffsets(MPU6050_TypeDef* MPU);
uint8_t MPU6050_GetID(MPU6050_TypeDef* MPU);
void MPU6050_Update(MPU6050_TypeDef* MPU);

#endif
