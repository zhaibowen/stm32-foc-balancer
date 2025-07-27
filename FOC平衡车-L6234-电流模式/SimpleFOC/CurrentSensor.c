#include "CurrentSensor.h"
#include "delay.h"
#include "usart.h"
#include "adc.h"

void CurrentSensor_Init(CurrentSensor* CSx, uint8_t number, int pinA, int pinB, int dir_a, int dir_b, int gain, float resistor) {
    CSx->pinA = pinA;
    CSx->pinB = pinB;
    CSx->ratio_a = dir_a / resistor / gain;
    CSx->ratio_b = dir_b / resistor / gain;
	CSx->number = number;

    configureADCInline(pinA, pinB);

	float offset_ia = 0;
	float offset_ib = 0;
	int pknt = 100;
	for(int i = 0; i < pknt; i++)
	{
		offset_ia += _readADCVoltageInline(pinA);
		offset_ib += _readADCVoltageInline(pinB);
		delay_ms(1);
	}
	// calculate the mean offsets
	offset_ia = offset_ia / pknt;
	offset_ib = offset_ib / pknt;
	
    CSx->offset_ia = offset_ia;
    CSx->offset_ib = offset_ib;
	printf("Current Sampler %d offset_ia:%.4f, offset_ib:%.4f.\r\n", number, offset_ia, offset_ib);
}

void getPhaseCurrents(CurrentSensor* CSx) {
	CSx->ia = (_readADCVoltageInline(CSx->pinA) - CSx->offset_ia) * CSx->ratio_a;// amps
	CSx->ib = (_readADCVoltageInline(CSx->pinB) - CSx->offset_ib) * CSx->ratio_b; // amps
}
