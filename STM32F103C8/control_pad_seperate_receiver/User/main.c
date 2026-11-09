#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "delay.h"
#include "OLED.h"
#include "NRF24L01.h"

uint8_t rxBuf[33] = {0};
float rnum;
char out_str[20];
char flag = 0;

int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	NRF24L01_Init();
	
	OLED_ShowChar(1, 1, 'A');
	OLED_ShowChar(2, 1, 'B');
	OLED_ShowChar(3, 1, 'C');
	OLED_ShowChar(4, 1, 'D');
	OLED_ShowChar(1, 9, 'E');
	OLED_ShowChar(2, 9, 'F');
	OLED_ShowChar(3, 9, 'G');
	OLED_ShowChar(4, 9, 'H');
	while (1) {
        if (NRF24L01_Get_Value_Flag() == 0) {
            NRF24L01_GetRxBuf(rxBuf);
            if (rxBuf[32] == 0) {
								OLED_ShowChar(1, 1, flag?'A':'T');
								flag ^= 1;
								for (int i = 0; i < 2; i++) {
									for (int j = 0; j < 4; j++) {
										memcpy(&rnum, rxBuf + i * 16 + j * 4, 4);
										sprintf(out_str, "%.5f", rnum);
										for (int k = 0; k < 7; k++){
											OLED_ShowChar(j+1, (i?10:2)+k, out_str[k]);
										}
									}
								}
            }
        }
	}
}
