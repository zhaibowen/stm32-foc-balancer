#ifndef __MPU6050_H
#define __MPU6050_H

typedef struct 
{
	float alpha;
	float gyro_config, accel_config;
	int16_t rawAccX, rawAccY, rawAccZ;
    int16_t rawGyroX, rawGyroY, rawGyroZ;
	float gyroXoffset, gyroYoffset, gyroZoffset;
    float AccZoffset;
	float AccX, AccY, AccZ;
    float GyroX, GyroY, GyroZ;
    float GravX, GravY, GravZ;
	uint32_t timestamp_prev;
    float roll, pitch, yaw;
	I2C_TypeDef* i2c;
} MPU6050_TypeDef;

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data, I2C_TypeDef* I2Cx);
uint8_t MPU6050_ReadReg(uint8_t RegAddress, I2C_TypeDef* I2Cx);

void MPU6050_Init(MPU6050_TypeDef* MPU, I2C_TypeDef* I2Cx);
void MPU6050_CalcGyroOffsets(MPU6050_TypeDef* MPU);
uint8_t MPU6050_GetID(MPU6050_TypeDef* MPU);
void MPU6050_Update(MPU6050_TypeDef* MPU);

#endif
