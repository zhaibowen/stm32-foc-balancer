#include "CurrentSensor.h"
#include "delay.h"
#include "usart.h"
#include "adc.h"

void CurrentSensor_Init(CurrentSensor* CSx, int pinA, int pinC, int dir_a, int dir_c, int gain, float resistor) {
    CSx->pinA = pinA;
    CSx->pinC = pinC;
    CSx->ratio_a = dir_a / resistor / gain;
    CSx->ratio_c = dir_c / resistor / gain;

    configureADCInline(pinA, pinC);

	float offset_ia = 0;
	float offset_ic = 0;
	for(int i = 0; i < 1000; i++)
	{
		offset_ia += _readADCVoltageInline(pinA);
		offset_ic += _readADCVoltageInline(pinC);
		delay_ms(1);
	}
	// calculate the mean offsets
	offset_ia = offset_ia / 1000;
	offset_ic = offset_ic / 1000;
	
    CSx->offset_ia = offset_ia;
    CSx->offset_ic = offset_ic;
	printf("Current Sampler offset_ia:%.4f, offset_ic:%.4f.\r\n", offset_ia, offset_ic);
}

void getPhaseCurrents(CurrentSensor* CSx) {
	CSx->ia = (_readADCVoltageInline(CSx->pinA) - CSx->offset_ia) * CSx->ratio_a;// amps
	CSx->ic = (_readADCVoltageInline(CSx->pinC) - CSx->offset_ic) * CSx->ratio_c; // amps
    CSx->ib = -CSx->ia - CSx->ic;
}
