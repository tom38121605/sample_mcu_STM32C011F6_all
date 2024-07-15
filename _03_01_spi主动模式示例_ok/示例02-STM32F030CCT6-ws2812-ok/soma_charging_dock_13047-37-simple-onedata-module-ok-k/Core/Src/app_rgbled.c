
#include "app_rgbled.h"


#define CODE_0		         0xC0
#define CODE_1		         0xF8

#define RGBLED_COUNT		   8
#define RGBLED_BYTE_COUNT	RGBLED_COUNT*3

uint8_t ispidata[RGBLED_BYTE_COUNT * 8] = {0};	

uint8_t ileddata[RGBLED_BYTE_COUNT] = 
{
   //R     G     B
   0XFF, 0X00, 0X00,	//0
   0X00, 0XFF, 0X00,	//1
   0X00, 0X00, 0XFF,	//2
   0X00, 0XFF, 0XFF,	//3
   0XFF, 0X00, 0XFF,	//4
   0XFF, 0XFF, 0X00,	//5
   0XFF, 0XFF, 0XFF,	//6
   0X00, 0X00, 0X00,	//7
   
};


void led2spi_onedata(uint8_t idatain,  uint8_t *sout)
{
   for(uint8_t i = 0; i <8; i++) 
   {
      if(idatain & 0x80)       
         *sout = CODE_1;          
      else           
         *sout = CODE_0;
       
      idatain <<= 1;
 
      sout++; 
   }
}

void led2spi_data(uint8_t *sledin,  uint8_t *sout, uint16_t icount)
{
   
	for(uint8_t i = 0; i < icount; i++)
	{      
      led2spi_onedata(ileddata[i], &sout[i*8]);
	}
   
}
 
void move_color(void)
{
   uint8_t i;
   uint8_t temp[3];
   
   temp[0] = ileddata[0] ;
   temp[1] = ileddata[1] ;
   temp[2] = ileddata[2] ;	
   
   for (i = 0; i < RGBLED_COUNT-1; i++)
   {
      ileddata[i*3] = ileddata[(i+1)*3 ];
      ileddata[i*3+1] = ileddata[(i+1)*3+1];
      ileddata[i*3+2] = ileddata[(i+1)*3+2];
   }
   
   ileddata[7*3]  = temp[0];
   ileddata[7*3+1] = temp[1];
   ileddata[7*3+2] = temp[2];
}


void rgbled_task(void)
{ 
   LL_SPI_Enable(SPI1);  

   led2spi_data(ileddata,  ispidata, RGBLED_BYTE_COUNT);
   
////test  log   
//uartsendcmd( (uint8_t *)ileddata, RGBLED_COUNT *3);
//uartsendcmd( ispidata, RGBLED_COUNT *3*8);
   
   //rgb data
   spi_transmitbytes(ispidata, RGBLED_BYTE_COUNT*8 );
   
   //reset data
   for(uint16_t i=0; i<500; i++)
   {
      LL_SPI_TransmitData8(SPI1, 0);
      while(!LL_SPI_IsActiveFlag_TXE(SPI1));   
   }
   
	//LL_mDelay(1);	
}
 
 

