#ifndef __ADC_H
#define __ADC_H	
/******************************************************************************/	
#include "stm32f10x.h"
/******************************************************************************/
void configureADCInline(int a,int b);
unsigned short analogRead(unsigned char ch);
float _readADCVoltageInline(unsigned char ch);
/******************************************************************************/	

#endif 
