#include "stm32f10x.h"                  // Device header
#include "AD.h"

void HW504_Init(void) {
	AD_Init();

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

uint16_t HW504_LX(void) {
	return AD_GetValue(ADC_Channel_1);
}

uint16_t HW504_LY(void) {
	return AD_GetValue(ADC_Channel_0);
}

uint16_t HW504_RX(void) {
	return 4096 - AD_GetValue(ADC_Channel_2);
}

uint16_t HW504_RY(void) {
	return AD_GetValue(ADC_Channel_3);
}

uint8_t HW504_Key0(void) {
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4);
}

uint8_t HW504_Key1(void) {
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5);
}

uint8_t HW504_Key2(void) {
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
}

uint8_t HW504_Key3(void) {
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7);
}

uint8_t HW504_Key4(void) {
	return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);
}

uint8_t HW504_Key5(void) {
	return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);
}
