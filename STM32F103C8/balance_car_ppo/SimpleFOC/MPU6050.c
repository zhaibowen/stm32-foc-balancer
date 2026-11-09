#include "stm32f10x.h"                  // Device header
#include "MPU6050_Reg.h"
#include "MPU6050.h"
#include "timer.h"
#include "delay.h"
#include "math.h"
#include "foc_utils.h"
#include "usart.h"

#define MPU6050_ADDRESS		0xD0		//MPU6050的I2C从机地址

unsigned char mpu_data[14] = {0};

void MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 10000;									//给定超时计数时间
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)	//循环等待指定事件
	{
		Timeout --;										//等待时，计数值自减
		if (Timeout == 0)								//自减到0后，等待超时
		{
			// printf("MPU6050_WaitEvent failed \r\n");
			break;										//跳出等待，不等了
		}
	}
}

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data, I2C_TypeDef* I2Cx)
{
	I2C_GenerateSTART(I2Cx, ENABLE);										//硬件I2C生成起始条件
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);					//等待EV5
	
	I2C_Send7bitAddress(I2Cx, MPU6050_ADDRESS, I2C_Direction_Transmitter);	//硬件I2C发送从机地址，方向为发送
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	//等待EV6
	
	I2C_SendData(I2Cx, RegAddress);											//硬件I2C发送寄存器地址
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING);			//等待EV8
	
	I2C_SendData(I2Cx, Data);												//硬件I2C发送数据
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);				//等待EV8_2
	
	I2C_GenerateSTOP(I2Cx, ENABLE);											//硬件I2C生成终止条件
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress, I2C_TypeDef* I2Cx)
{
	uint8_t Data;
	
	I2C_GenerateSTART(I2Cx, ENABLE);										//硬件I2C生成起始条件
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);					//等待EV5
	
	I2C_Send7bitAddress(I2Cx, MPU6050_ADDRESS, I2C_Direction_Transmitter);	//硬件I2C发送从机地址，方向为发送
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	//等待EV6
	
	I2C_SendData(I2Cx, RegAddress);											//硬件I2C发送寄存器地址
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);				//等待EV8_2
	
	I2C_GenerateSTART(I2Cx, ENABLE);										//硬件I2C生成重复起始条件
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);					//等待EV5
	
	I2C_Send7bitAddress(I2Cx, MPU6050_ADDRESS, I2C_Direction_Receiver);		//硬件I2C发送从机地址，方向为接收
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);		//等待EV6
	
	I2C_AcknowledgeConfig(I2Cx, DISABLE);									//在接收最后一个字节之前提前将应答失能
	I2C_GenerateSTOP(I2Cx, ENABLE);											//在接收最后一个字节之前提前申请停止条件
	
	MPU6050_WaitEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED);				//等待EV7
	Data = I2C_ReceiveData(I2Cx);											//接收数据寄存器
	
	I2C_AcknowledgeConfig(I2Cx, ENABLE);									//将应答恢复为使能，为了不影响后续可能产生的读取多字节操作
	
	return Data;
}

void i2c_ReadMulti(uint8_t RegAddress, I2C_TypeDef* I2Cx, int n, unsigned char* data)
{
    __disable_irq();
	volatile int temp;
	while (I2Cx->SR2 & I2C_SR2_BUSY){;}
	I2Cx->CR1|=I2C_CR1_START;
	while(!(I2Cx->SR1 & I2C_SR1_SB)){;}
	I2Cx->DR=MPU6050_ADDRESS;
	while(!(I2Cx->SR1 & I2C_SR1_ADDR)){;}
	temp=I2Cx->SR2;
	while(!(I2Cx->SR1&I2C_SR1_TXE)){;}
	I2Cx->DR = RegAddress;
	while(!(I2Cx->SR1&I2C_SR1_TXE)){;}
	I2Cx->CR1|=I2C_CR1_START;
	while(!(I2Cx->SR1 & I2C_SR1_SB)){;}
	I2Cx->DR=MPU6050_ADDRESS|1;
	while(!(I2Cx->SR1 & I2C_SR1_ADDR)){;}
	temp=I2Cx->SR2;
	I2Cx->CR1|=I2C_CR1_ACK;
	while(n>0U) {
		if(n==1U) {
			I2Cx->CR1&=~I2C_CR1_ACK;
			I2Cx->CR1|=I2C_CR1_STOP;
			while(!(I2Cx->SR1&I2C_SR1_RXNE)){;}
			*data++=I2Cx->DR;
			break;
		} else {
			while(!(I2Cx->SR1&I2C_SR1_RXNE)){;}
			(*data++)=I2Cx->DR;
			n--;
		}	
	}			
	I2Cx->CR1  |= 0x0400;//CR1_ACK_Set;
    __enable_irq();
}

void MPU6050_Init(MPU6050_TypeDef* MPU, I2C_TypeDef* I2Cx) {
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01, I2Cx);				//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00, I2Cx);				//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x00, I2Cx);				//采样率分频寄存器，配置采样率，采样频率1KHz
	MPU6050_WriteReg(MPU6050_CONFIG, 0x02, I2Cx);					//配置寄存器，配置DLPF，延迟3ms
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x10, I2Cx);			//陀螺仪配置寄存器，±1000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00, I2Cx);			//加速度计配置寄存器，±2g
	
	MPU->gyro_config = 1.0 / (32768.0 / 1000 * 180.0 / _PI);
	MPU->accel_config = 1.0 / (32768.0 / 2);
	MPU->alpha = 0.98;
	MPU->timestamp_prev = 0;
	MPU->gyroXoffset = -0.0340;
	MPU->gyroYoffset = 0.0303;
	MPU->gyroZoffset = 0.0368;
	MPU->AccZoffset = 0.06;
	MPU->i2c = I2Cx;
}

uint8_t MPU6050_GetID(MPU6050_TypeDef* MPU)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I, MPU->i2c);		//返回WHO_AM_I寄存器的值
}

void MPU6050_CalcGyroOffsets(MPU6050_TypeDef* MPU) {
	float x = 0, y = 0, z = 0;
	int16_t rx, ry, rz;

	int nknt = 200;
	for(int i = 0; i < nknt; i++){
	    delay_ms(5);
        // 读6个有误差，不知道为什么，必须读14个
        i2c_ReadMulti(MPU6050_ACCEL_XOUT_H, MPU->i2c, 14, mpu_data);
        rx = (mpu_data[8] << 8) | mpu_data[9];
        ry = (mpu_data[10] << 8) | mpu_data[11];
        rz = (mpu_data[12] << 8) | mpu_data[13];

		x += ((float)rx) * MPU->gyro_config;
		y += ((float)ry) * MPU->gyro_config;
		z += ((float)rz) * MPU->gyro_config;
	}
	MPU->gyroXoffset = x / nknt;
	MPU->gyroYoffset = y / nknt;
	MPU->gyroZoffset = z / nknt;
	// printf("MPU offset: %.4f, %.4f, %.4f\r\n", MPU->gyroXoffset, MPU->gyroYoffset, MPU->gyroZoffset);
}

void complementary_filtering(MPU6050_TypeDef* sensor, float dt) {
    float gx = sensor->GyroX;
    float gy = sensor->GyroY;
    float gz = sensor->GyroZ;
    float ax = sensor->AccX;
    float ay = sensor->AccY;
    float az = sensor->AccZ;
    float gyro_roll = sensor->roll;
    float gyro_pitch = sensor->pitch;
    float gyro_yaw = sensor->yaw;
    float alpha = sensor->alpha;

    //计算加速度计欧拉角
    float accel_roll = _atan2(ay,az);
    float accel_pitch = _atan2(-ax, _sqrtApprox(ay*ay + az*az));

    float s = sin(gyro_roll);
    float c = cos(gyro_roll);
    float t = tan(gyro_pitch);
    float sec = 1.0f / cos(gyro_pitch);
    //计算变化率
    float roll_rate = gx + s * t * gy + c * t * gz;
    float pitch_rate = c * gy - s * gz;
    float yaw_rate = (s * sec) * gy + (c * sec) * gz;
    //计算新的陀螺仪欧拉角
    float roll_new = gyro_roll + roll_rate * dt;
    float pitch_new = gyro_pitch + pitch_rate * dt;
    float yaw_new = gyro_yaw + yaw_rate * dt;

    sensor->roll=alpha*roll_new+(1-alpha)*accel_roll;
    sensor->pitch=alpha*pitch_new+(1-alpha)*accel_pitch;
    sensor->yaw=yaw_new;
}

void gravity_projection(MPU6050_TypeDef* sensor) {
    float pitch = sensor->pitch;
    float roll = sensor->roll;
    sensor->GravX = sin(pitch);
    sensor->GravY = -sin(roll) * cos(pitch);
    sensor->GravZ = -cos(roll) * cos(pitch);
}

void MPU6050_Update(MPU6050_TypeDef* MPU) {
	i2c_ReadMulti(MPU6050_ACCEL_XOUT_H, MPU->i2c, 14, mpu_data);
	MPU->rawAccX = (mpu_data[0] << 8) | mpu_data[1];
	MPU->rawAccY = (mpu_data[2] << 8) | mpu_data[3];
	MPU->rawAccZ = (mpu_data[4] << 8) | mpu_data[5];
	MPU->rawGyroX = (mpu_data[8] << 8) | mpu_data[9];
	MPU->rawGyroY = (mpu_data[10] << 8) | mpu_data[11];
	MPU->rawGyroZ = (mpu_data[12] << 8) | mpu_data[13];
    // 调整机体坐标系
	MPU->AccX = MPU->rawAccX * MPU->accel_config;
	MPU->AccY = MPU->rawAccY * MPU->accel_config;
	MPU->AccZ = -(MPU->rawAccZ * MPU->accel_config - MPU->AccZoffset);
	MPU->GyroX = -(((float)MPU->rawGyroX) * MPU->gyro_config - MPU->gyroXoffset);
	MPU->GyroY = -(((float)MPU->rawGyroY) * MPU->gyro_config - MPU->gyroYoffset);
	MPU->GyroZ = -(((float)MPU->rawGyroZ) * MPU->gyro_config - MPU->gyroZoffset);

    float time_interval = get_time_interval(&MPU->timestamp_prev);
    complementary_filtering(MPU, time_interval);
    gravity_projection(MPU);
    
    // 对齐urdf坐标系，前右上，逆时针为正
    MPU->GravX *= -1;
    MPU->GyroY *= -1;
}

