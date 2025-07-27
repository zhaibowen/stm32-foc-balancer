

#include "stm32f10x.h"
#include "timer.h"
#include "foc_utils.h"
#include "usart.h"
#include "MagneticSensor.h"

//I2C代码参考官方14年例程，直接操作寄存器，效率更高更简洁
unsigned short I2C_getRawCount(I2C_TypeDef* I2Cx)
{
	uint32_t Timeout;
	unsigned short temp;
	unsigned char dh,dl;	
	
	Timeout = 0xFFFF;
	/* Send START condition */
	I2Cx->CR1 |= 0x0100;//CR1_START_Set;
	/* Wait until SB flag is set: EV5 */
	while ((I2Cx->SR1&0x0001) != 0x0001)
	{
		if (Timeout-- == 0) {
			printf("I2C_getRawCount failed \r\n");
			return 0;
		}
	}
	/* Send the slave address, Reset the address bit0 for write*/
	I2Cx->DR = AS5600_Address<<1;
	Timeout = 0xFFFF;
	/* Wait until ADDR is set: EV6 */
	while ((I2Cx->SR1 &0x0002) != 0x0002)
	{
		if (Timeout-- == 0) {
			printf("I2C_getRawCount failed \r\n");
			return 0;
		}
	}
	/* Clear ADDR flag by reading SR2 register */
	temp = I2Cx->SR2;
	/* Write the first data in DR register (EV8_1) */
	I2Cx->DR = RAW_Angle_Hi;
	/* EV8_2: Wait until BTF is set before programming the STOP */
	while ((I2Cx->SR1 & 0x00004) != 0x000004);
	
	/////////////////////////////////////////////////////////////////////////
	/* Set POS bit */
	I2Cx->CR1 |= 0x0800;//CR1_POS_Set;
	Timeout = 0xFFFF;
	/* Send START condition */
	I2Cx->CR1 |= 0x0100;//CR1_START_Set;
	/* Wait until SB flag is set: EV5 */
	while ((I2Cx->SR1&0x0001) != 0x0001)
	{
		if (Timeout-- == 0) {
			printf("I2C_getRawCount failed \r\n");
			return 0;
		}
	}
	Timeout = 0xFFFF;
	/* Send slave address */
	I2Cx->DR = (AS5600_Address<<1)+1;

	/* Wait until ADDR is set: EV6 */
	while ((I2Cx->SR1&0x0002) != 0x0002)
	{
		if (Timeout-- == 0) {
			printf("I2C_getRawCount failed \r\n");
			return 0;
		}
	}
	/* EV6_1: The acknowledge disable should be done just after EV6,
	that is after ADDR is cleared, so disable all active IRQs around ADDR clearing and 
	ACK clearing */
	__disable_irq();
	/* Clear ADDR by reading SR2 register  */
	temp = I2Cx->SR2;
	/* Clear ACK */
	I2Cx->CR1 &= 0xFBFF;//CR1_ACK_Reset;
	/*Re-enable IRQs */
	__enable_irq();
	/* Wait until BTF is set */
	while ((I2Cx->SR1 & 0x00004) != 0x000004);
	/* Disable IRQs around STOP programming and data reading because of the limitation ?*/
	__disable_irq();
	/* Program the STOP */
	I2C_GenerateSTOP(I2Cx, ENABLE);
	/* Read first data */
	dh = I2Cx->DR;
	/* Re-enable IRQs */
	__enable_irq();
	/**/
	/* Read second data */
	dl = I2Cx->DR;
	/* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
	while ((I2Cx->CR1&0x200) == 0x200);
	/* Enable Acknowledgement to be ready for another reception */
	I2Cx->CR1  |= 0x0400;//CR1_ACK_Set;
	/* Clear POS bit */
	I2Cx->CR1  &= 0xF7FF;//CR1_POS_Reset;
	
	temp++;  //useless,otherwise warning
	return ((dh<<8)+dl);
}

void MagneticSensor_Init(ENCODER_TypeDef* Coderx, uint8_t number, long cpr, I2C_TypeDef* i2c)
{
	Coderx->number = number;
	Coderx->cpr=cpr;
	Coderx->full_rotation_offset = 0;
	Coderx->angle_data_prev = I2C_getRawCount(i2c);  
	Coderx->angle_vel_prev=0;
	Coderx->velocity_calc_timestamp=0;
	Coderx->i2c = i2c;
}

float getAngle(ENCODER_TypeDef* Coderx) {
	float angle_data = I2C_getRawCount(Coderx->i2c);
	float d_angle = angle_data - Coderx->angle_data_prev;
 	if (fabs(d_angle) > (0.8 * Coderx->cpr)) {
		Coderx->full_rotation_offset += d_angle > 0 ? -_2PI : _2PI; 
	}
	Coderx->angle_data_prev = angle_data;
	return  (Coderx->full_rotation_offset + ( angle_data / (float)Coderx->cpr) * _2PI) ;
}

// Shaft velocity calculation
float getVelocity(ENCODER_TypeDef* Coderx)
{
	unsigned long now_us;
	float Ts, angle_c, vel;

	now_us = time_cntr;
	if (now_us < Coderx->velocity_calc_timestamp) {
		Ts = (ulong_max_value - Coderx->velocity_calc_timestamp + now_us) * 1e-5;
	} else {
		Ts = (now_us - Coderx->velocity_calc_timestamp) * 1e-5;
	}
	if(Ts == 0 || Ts > 0.5) Ts = 1e-3;

	angle_c = getAngle(Coderx);
	vel = (angle_c - Coderx->angle_vel_prev) / Ts;

	Coderx->angle_vel_prev = angle_c;
	Coderx->velocity_calc_timestamp = now_us;
	
	return vel;
}



