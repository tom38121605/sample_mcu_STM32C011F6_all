#include "ws2812.h"
#include "spi.h"

//�����Դ�SPI���ݻ���
uint8_t ispidata[RGBLED_COUNT * 24] = {0};	
//�����Դ�
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
	
	//��ileddata���ݽ�����SPI����
	for(uint8_t iLED = 0; iLED < RGBLED_COUNT; iLED++)
	{
		WS2812b_Set(iLED, ileddata[iLED].R, ileddata[iLED].G, ileddata[iLED].B);
	}
	//�����������
	//HAL_SPI_Transmit(&hspi1, ispidata, sizeof(ispidata),0XFFFF);
   
   LL_SPI_Enable(SPI1);   
   
   for(uint16_t i=0; i< (8*24); i++)
   //for(uint16_t i=0; i< (8*1); i++)
   {
      LL_SPI_TransmitData8(SPI1, ispidata[i]);
      while(!LL_SPI_IsActiveFlag_TXE(SPI1));
   }
   
	//ʹ��������͵�ƽ
	//HAL_SPI_Transmit(&hspi1, &dat, 1,0XFFFF);
       LL_SPI_TransmitData8(SPI1, 0);
       while(!LL_SPI_IsActiveFlag_TXE(SPI1));
   
	//֡�źţ�һ������50us�ĵ͵�ƽ
	HAL_Delay(1);	
}
