#include "stm32f10x.h"                  // Device header
#include "usart.h"
#include "Delay.h"
#include "NRF24L01.h"
#include "HW504.h"

int main(void) {
	uint8_t Buf[33] = {0};	
	uint16_t Lx, Ly, Rx, Ry;
	uint8_t Key0, Key1, Key2, Key3, Key4, Key5;

	uart_init(115200);
	NRF24L01_Init();
	HW504_Init();
	
	while (1) {
		Lx = HW504_LX();
		Ly = HW504_LY();
		Rx = HW504_RX();
		Ry = HW504_RY();
		Key0 = HW504_Key0();
		Key1 = HW504_Key1();
		Key2 = HW504_Key2();
		Key3 = HW504_Key3();
		Key4 = HW504_Key4();
		Key5 = HW504_Key5();
		
		Buf[0] = (uint8_t)(Lx & 0xFF); // 获取低8位
    	Buf[1] = (uint8_t)((Lx >> 8) & 0xFF); // 获取高8位
		Buf[2] = (uint8_t)(Ly & 0xFF); // 获取低8位
    	Buf[3] = (uint8_t)((Ly >> 8) & 0xFF); // 获取高8位

		Buf[4] = (uint8_t)(Rx & 0xFF); // 获取低8位
    	Buf[5] = (uint8_t)((Rx >> 8) & 0xFF); // 获取高8位
		Buf[6] = (uint8_t)(Ry & 0xFF); // 获取低8位
    	Buf[7] = (uint8_t)((Ry >> 8) & 0xFF); // 获取高8位

		Buf[8] = Key0;
		Buf[9] = Key1;
		Buf[10] = Key2;
		Buf[11] = Key3;
		Buf[12] = Key4;
		Buf[13] = Key5;
		
		Buf[32] = 1;

		// printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\r\n", Lx, Ly, Rx, Ry, Key0, Key1, Key2, Key3, Key4, Key5);
		NRF24L01_SendBuf(Buf);
		Delay_ms(100);
	}
}
