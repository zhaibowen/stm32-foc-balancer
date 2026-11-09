

#include "stm32f10x.h"
#include "delay.h"
#include "timer.h"
#include "foc_utils.h"
#include "usart.h"
#include "MagneticSensor.h"

uint8_t AS5600_ReadReg(I2C_TypeDef* I2Cx, uint8_t reg)
{
    uint32_t Timeout;
	unsigned short temp;
    uint8_t data;

    Timeout = 0xFFFF;
    I2Cx->CR1 |= 0x0100;
    while ((I2Cx->SR1 & 0x0001) != 0x0001) {
        if (Timeout-- == 0) return 0;
    }
    I2Cx->DR = AS5600_Address << 1;
    Timeout = 0xFFFF;
    while ((I2Cx->SR1 & 0x0002) != 0x0002) {
        if (Timeout-- == 0) return 0;
    }
    temp = I2Cx->SR2;

    /* 写寄存器地址 */
    I2Cx->DR = reg;
    Timeout = 0xFFFF;
    while ((I2Cx->SR1 & 0x0004) != 0x0004) {
        if (Timeout-- == 0) return 0;
    }

    /* 重复START + 地址+读 */
    I2Cx->CR1 |= 0x0100;
    Timeout = 0xFFFF;
    while ((I2Cx->SR1 & 0x0001) != 0x0001) {
        if (Timeout-- == 0) return 0;
    }
    I2Cx->DR = (AS5600_Address << 1) + 1;
    Timeout = 0xFFFF;
    while ((I2Cx->SR1 & 0x0002) != 0x0002) {
        if (Timeout-- == 0) return 0;
    }

    /* 单字节接收：清ADDR后立即清ACK，等待BTF后发STOP */
    __disable_irq();
    temp = I2Cx->SR2;               /* 清ADDR */
    I2Cx->CR1 &= 0xFBFF;            /* 清ACK，准备回NACK */
    __enable_irq();

    Timeout = 0xFFFF;
    while ((I2Cx->SR1 & 0x0004) != 0x0004) {
        if (Timeout-- == 0) return 0;
    }

    __disable_irq();
    I2C_GenerateSTOP(I2Cx, ENABLE);
    data = I2Cx->DR;
    __enable_irq();

    while ((I2Cx->CR1 & 0x0200) == 0x0200);   /* 等STOP硬件清除 */
    I2Cx->CR1 |= 0x0400;                      /* 恢复ACK */

	temp++;  //useless,otherwise warning
    return data;
}


uint8_t AS5600_WriteReg(I2C_TypeDef* I2Cx, uint8_t reg, uint8_t data)
{
    uint32_t Timeout;
    unsigned short temp;

    Timeout = 0xFFFF;
    I2Cx->CR1 |= 0x0100;
    while ((I2Cx->SR1 & 0x0001) != 0x0001) {
        if (Timeout-- == 0) return 0;
    }
    I2Cx->DR = AS5600_Address << 1;
    Timeout = 0xFFFF;
    while ((I2Cx->SR1 & 0x0002) != 0x0002) {
        if (Timeout-- == 0) return 0;
    }
    temp = I2Cx->SR2;               /* 清ADDR */

    /* 写寄存器地址 */
    I2Cx->DR = reg;
    Timeout = 0xFFFF;
    while ((I2Cx->SR1 & 0x0004) != 0x0004) {
        if (Timeout-- == 0) return 0;
    }

    /* 写数据 */
    I2Cx->DR = data;
    Timeout = 0xFFFF;
    while ((I2Cx->SR1 & 0x0004) != 0x0004) {
        if (Timeout-- == 0) return 0;
    }

    /* STOP + 等待硬件清除STOP位 */
    __disable_irq();
    I2C_GenerateSTOP(I2Cx, ENABLE);
    __enable_irq();

    Timeout = 0xFFFF;
    while ((I2Cx->CR1 & 0x0200) == 0x0200) {   /* 等STOP硬件清除 */
        if (Timeout-- == 0) return 0;
    }

    temp++;  //useless,otherwise warning
    return 0;
}

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
			// printf("I2C_getRawCount failed \r\n");
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
			// printf("I2C_getRawCount failed \r\n");
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
			// printf("I2C_getRawCount failed \r\n");
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
			// printf("I2C_getRawCount failed \r\n");
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
    return ((dh & 0x0F) << 8) | dl;
}

void MagneticSensor_Init(ENCODER_TypeDef* Coderx, uint8_t number, long cpr, I2C_TypeDef* i2c)
{
    AS5600_WriteReg(i2c, AS5600_CONF_Hi, 0x02);  // 4x慢速滤波，延迟0.55ms，噪声0.03度
    Coderx->status = AS5600_ReadReg(i2c, AS5600_CONF_Hi);
	Coderx->number = number;
	Coderx->cpr=cpr;
	Coderx->cpr2=0.8 * cpr;
	Coderx->cpr3=_2PI / cpr;
	Coderx->full_rotation_offset = 0;
	Coderx->angle_data_prev = I2C_getRawCount(i2c);  
	Coderx->angle_prev = getAngle(Coderx);
	Coderx->timestamp_prev = 0;
	Coderx->i2c = i2c;
}

float getAngle(ENCODER_TypeDef* Coderx) {
	float angle_data = I2C_getRawCount(Coderx->i2c);
	float d_angle = angle_data - Coderx->angle_data_prev;
 	if (fabs(d_angle) > Coderx->cpr2) {
		Coderx->full_rotation_offset += d_angle > 0 ? -_2PI : _2PI; 
	}
	Coderx->angle_data_prev = angle_data;
	return  Coderx->full_rotation_offset + angle_data * Coderx->cpr3;
}

// Shaft velocity calculation
float getVelocity(ENCODER_TypeDef* Coderx, float shaft_angle) {
	float vel = (shaft_angle - Coderx->angle_prev) * 1000; // 1KHz
	Coderx->angle_prev = shaft_angle;
	return vel;
}



