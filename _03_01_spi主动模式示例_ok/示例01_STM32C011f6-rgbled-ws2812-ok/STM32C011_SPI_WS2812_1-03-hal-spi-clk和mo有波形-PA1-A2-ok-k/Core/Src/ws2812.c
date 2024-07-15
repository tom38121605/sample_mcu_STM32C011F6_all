#include "ws2812.h"
#include "spi.h"

//灯条显存SPI数据缓存
uint8_t ispidata[RGBLED_COUNT * 24] = {0};	
//灯条显存
tWs2812bCache_TypeDef ileddata[RGBLED_COUNT] = {

//R    G      B
0XFF, 0X00, 0X00,	//0
0X00, 0XFF, 0X00,	//1
0X00, 0X00, 0XFF,	//2
0X00, 0XFF, 0XFF,	//3
0XFF, 0X00, 0XFF,	//4
0XFF, 0XFF, 0X00,	//5
0XFF, 0XFF, 0XFF,	//6
0X00, 0X00, 0X00,	//7
};
		
void WS2812b_Set(uint16_t inum, uint8_t r,uint8_t g,uint8_t b)
{
	uint8_t *pR = &ispidata[(inum) * 24 + 8];
	uint8_t *pG = &ispidata[(inum) * 24];
	uint8_t *pB = &ispidata[(inum) * 24 + 16];
	
	for(uint8_t i = 0; i <  8; i++) {
		if(g & 0x80) {
			*pG = CODE_1;
		}           
		else {           
			*pG = CODE_0;
		}           
		if(r & 0x80) {           
			*pR = CODE_1;
		}           
		else {           
			*pR = CODE_0;
		}           
		if(b & 0x80) {           
			*pB = CODE_1;
		}           
		else {           
			*pB = CODE_0;
		}
		r <<= 1;
		g <<= 1;
		b <<= 1;
		pR++;
		pG++;
		pB++;
	}
}
void WS2812B_Task(void)
{
	uint8_t dat = 0;
	
	//将ileddata数据解析成SPI数据
	for(uint8_t iLED = 0; iLED < RGBLED_COUNT; iLED++)
	{
		WS2812b_Set(iLED, ileddata[iLED].R, ileddata[iLED].G, ileddata[iLED].B);
	}
	//总线输出数据
	HAL_SPI_Transmit(&hspi1, ispidata, sizeof(ispidata),0XFFFF);
   
	//使总线输出低电平
	HAL_SPI_Transmit(&hspi1, &dat, 1,0XFFFF);
	//帧信号：一个大于50us的低电平
	HAL_Delay(1);	
}
