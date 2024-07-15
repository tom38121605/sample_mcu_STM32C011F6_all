/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention


  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>           //add for sprintf
#include "string.h"          //add for memcmp
#include "spi.h"             //add 

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



//#define TXMODE
#define RXMODE

extern volatile uint8_t flg_rx;
extern volatile uint8_t irxdata[RXDATALEN+2];
extern volatile uint8_t irxdata2[RXDATALEN+2];
//extern volatile uint8_t icurrentindex;

extern volatile uint32_t irxcount;
extern volatile uint32_t irxcount2;


extern volatile uint8_t flg_timer1;

//uint32_t  isensorcount=0;
//uint32_t  iledcount=0;

uint32_t  immc0_ctrl_0=0;
uint32_t  immc1_ctrl_0=0;

uint8_t ireportdata[32];

 
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

  #define ADC_CALIBRATION_TIMEOUT_MS       (   1UL)
  #define ADC_ENABLE_TIMEOUT_MS            (   1UL)
  #define ADC_DISABLE_TIMEOUT_MS           (   1UL)
  #define ADC_STOP_CONVERSION_TIMEOUT_MS   (   1UL)
  #define ADC_CONVERSION_TIMEOUT_MS        (4000UL)

  /* Delay between ADC end of calibration and ADC enable.                     */
  /* Delay estimation in CPU cycles: Case of ADC enable done                  */
  /* immediately after ADC calibration, ADC clock setting slow                */
  /* (LL_ADC_CLOCK_ASYNC_DIV32). Use a higher delay if ratio                  */
  /* (CPU clock / ADC clock) is above 32.                                     */
  #define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)

/* Definitions of environment analog values */
  /* Value of analog reference voltage (Vref+), connected to analog voltage   */
  /* supply Vdda (unit: mV).                                                  */
  #define VDDA_APPLI                       (3300UL)

/* Definitions of data related to this example */
  /* Init variable out of expected ADC conversion data range */
  #define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1)
  
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//---MMC5983---  
#define DATA_HEADER_POINT  0
#define DATA_MMC0_POINT  2
#define DATA_MMC1_POINT  9
#define DATA_ICM0_POINT  16
#define DATA_TIME_POINT  28
#define DATA_ENDER_POINT 30


#define FROZEN_GAP   10000    //10s

#define MMC5983_XOUT_REG            0x00
#define MMC5983_TOUT_REG            0x07
#define MMC5983_STATUS_REG          0x08
#define MMC5983_CONTROL0_REG        0x09
#define MMC5983_CONTROL1_REG        0x0A
#define MMC5983_CONTROL2_REG        0x0B
#define MMC5983_CONTROL3_REG        0x0C
#define MMC5983_PRODUCT_ID_REG      0x2F

#define MMC5983_PRODUCT_ID_RESULT   0x30

#define MMC5983_MYSTERY_REG 0x29
#define MMC5983_MYSTERY_TC_DISABLE 0x01
#define MMC5983_UNLOCK_REG 0x0F
#define MMC5983_UNLOCK_VAL 0xE1

//MEMSIC values
#define MMC5983_CONTROL0_MAG_MEAS   0x01
#define MMC5983_CONTROL0_TEMP_MEAS  0x02
#define MMC5983_CONTROL0_INT_EN     0x04
#define MMC5983_CONTROL0_SET        0x08
#define MMC5983_CONTROL0_RESET      0x10
#define MMC5983_CONTROL0_AUTOSREN   0x20   //add

#define MMC5983_CONTROL0_INIT       0x00
#define MMC5983_CONTROL0_START_TEMP_CMD  (MMC5983_CONTROL0_TEMP_MEAS | MMC5983_CONTROL0_RESET)
#define MMC5983_CONTROL0_START_CMD  (MMC5983_CONTROL0_MAG_MEAS)

#ifdef OMMO_SENSOR_MEMSIC_MMC5983_SET
  #define MMC5983_CONTROL0_SET_CMD    (MMC5983_CONTROL0_SET)
#else
  #define MMC5983_CONTROL0_SET_CMD    (MMC5983_CONTROL0_RESET) //RESET will invert the axis making them right handed
#endif

#define MMC5983_CONTROL1_CHIP_RESET 0x80
#define MMC5983_CONTROL1_BW_100     0x00
#define MMC5983_CONTROL1_BW_200     0x01
#define MMC5983_CONTROL1_BW_400     0x02
#define MMC5983_CONTROL1_BW_800     0x03
#define MMC5983_CONTROL1_INIT       MMC5983_CONTROL1_BW_8

#define MMC5983_CONTROL2_CM_FREQ_1000  0X07    //add
#define MMC5983_CONTROL2_CMM_EN        0X08    //add
 
//---ICM42605---  
// Bank 0
#define ICM_DEVICE_ID                          0x42  

#define ICM42605_DEVICE_CONFIG_REG             0x11    
#define ICM42605_INT_CONFIG_REG                0x14   
#define ICM42605_ACCEL_DATA_X1_REG             0x1F  

#define ICM42605_INT_STATUS_REG                0x2D      

#define ICM42605_PWR_MGMT0_REG                 0x4E     
#define ICM42605_GYRO_CONFIG0_REG              0x4F     
#define ICM42605_ACCEL_CONFIG0_REG             0x50   
#define ICM42605_GYRO_CONFIG1_REG              0x51   

#define ICM42605_INT_CONFIG1_REG               0x64  
#define ICM42605_INT_SOURCE0_REG               0x65  
#define ICM42605_WHO_AM_I_REG                  0x75          

#define ICM42605_REG_BANK_SEL_REG              0x76      

// Bank 4
#define ICM42605_APEX_CONFIG5_REG              0x44   


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void iputs1(char *msg)
{
   while(*msg)
   Send_Data_To_UART1(*msg++);   
}


void iputbytes1(uint8_t *msg, uint32_t ilen)
{
   while(ilen--)
   Send_Data_To_UART1(*msg++);
}

void Send_Data_To_UART1 (uint8_t c)
{
   USART1->TDR = c;
   while( (USART1-> ISR & (1<<7) ) ==0 );
}


uint8_t get_checksum_8(uint8_t *ival, uint32_t inum)
{
   uint32_t i =0;
   uint8_t iret =0;
   uint16_t isum =0;

   for (i=0; i<inum; i++)
   {
      isum += ival[i]; 
   }

   iret = (uint8_t)(isum & 0xff);

   return iret;
}

void dowith_uart_tx(void)
{   
 
}


void dowith_uart_rx(void)
{  

 
} 


void delay_count(uint32_t icount)
{
   uint32_t inum = icount;
   while (inum--) ;
}


uint8_t SPI_transfer(uint8_t data)
{
   // transmit
	LL_SPI_TransmitData8(MMC_SPI, data);
	while(!LL_SPI_IsActiveFlag_TXE(MMC_SPI));

   // receive
	while(!LL_SPI_IsActiveFlag_RXNE(MMC_SPI));
	return LL_SPI_ReceiveData8(MMC_SPI);

}


//---------------------mmc0 module---------------------------

uint8_t mmc0_read_byte(uint8_t addr)
{
   uint8_t value;

   LL_GPIO_ResetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_3);

   SPI_transfer(0x80|addr );
   value  = SPI_transfer(0x00);

   LL_GPIO_SetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_3);

   return value;
}

void mmc0_write_byte(uint8_t addr,  uint8_t value)
{
 
   LL_GPIO_ResetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_3);

   SPI_transfer(addr);
   SPI_transfer(value);

   LL_GPIO_SetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_3);
 
}

void mmc0_read_bytes(uint8_t addr ,uint8_t length ,uint8_t *buffer)
{
   for(int index = 0; index < length; index++)
   {
       buffer[index]  = mmc0_read_byte(addr+index);
   } 
 
}

void mmc0_readfromspi( uint8_t iaddr, uint8_t ilen, uint8_t *idata)
{
 
   LL_GPIO_ResetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_3);      

   SPI_transfer(0x80 |iaddr);

   while(ilen--)
   {
       *idata  = SPI_transfer(0x00);
       idata++;
   }

   LL_GPIO_SetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_3);
 
}
 
void mmc0_init(void)
{
   uint8_t ivalue=0;

   //check device id
   ivalue = mmc0_read_byte(MMC5983_PRODUCT_ID_REG);  //read id    
   if(ivalue == MMC5983_PRODUCT_ID_RESULT)  //device id
   {
      //flg_mmc0_ready=1;      
      //iputs1("mmc0 ok");       

      //reset   
      mmc0_write_byte(MMC5983_CONTROL1_REG,MMC5983_CONTROL1_CHIP_RESET );  //reset
      LL_mDelay(20);                    

      //set control 0  
      mmc0_write_byte(MMC5983_CONTROL0_REG,MMC5983_CONTROL0_AUTOSREN|MMC5983_CONTROL0_INT_EN);  //auto ser/reset, enable interrupt    
       
      //set control 1   
      mmc0_write_byte(MMC5983_CONTROL1_REG,MMC5983_CONTROL1_BW_800);     //800Hz BW, Measur Time

      //set control 2   
      //mmc0_write_byte(MMC5983_CONTROL2_REG,MMC5983_CONTROL2_CM_FREQ_1000| MMC5983_CONTROL2_CMM_EN);  //1000Hz measure, continuous mode
      mmc0_write_byte(MMC5983_CONTROL2_REG,MMC5983_CONTROL2_CM_FREQ_1000| MMC5983_CONTROL2_CMM_EN|7<<4|0x80);  //1000Hz measure, continuous mode, 1000Hz prd set, enable prd set 

      //clear int
      ivalue = mmc0_read_byte(MMC5983_STATUS_REG);       //status  
      mmc0_write_byte(MMC5983_STATUS_REG, ivalue & 0x01);  
    
   }
   else
   {
      //flg_mmc0_ready=0;
      iputs1("mmc0 err");   
   }     
   
}

void mmc0_read_data(uint8_t *sdata, uint8_t ilen)
{
   uint8_t ivalue=0;  

   //clear int
   ivalue = mmc0_read_byte(MMC5983_STATUS_REG);
   mmc0_write_byte(MMC5983_STATUS_REG, ivalue & 0x01);

   //read data
   //mmc0_read_bytes(MMC5983_XOUT_REG, ilen, sdata);
   mmc0_readfromspi(MMC5983_XOUT_REG, ilen, sdata);
}   


void mmc0_init2(void)
{
   uint8_t ivalue=0;

   //check device id
   ivalue = mmc0_read_byte(MMC5983_PRODUCT_ID_REG);  //read id    
   if(ivalue == MMC5983_PRODUCT_ID_RESULT)  //device id
   {
      iputs1("mmc0 ok");       
      //flg_mmc0_ready=1;    

      //unfrozen test  
      mmc0_write_byte(MMC5983_CONTROL0_REG,MMC5983_CONTROL0_AUTOSREN|MMC5983_CONTROL0_INT_EN);      //auto ser/reset, enable interrupt    
      mmc0_write_byte(MMC5983_CONTROL2_REG,MMC5983_CONTROL2_CM_FREQ_1000| MMC5983_CONTROL2_CMM_EN); //1000Hz, continuous mode
      LL_mDelay(1);   
      mmc0_write_byte(MMC5983_CONTROL0_REG,MMC5983_CONTROL0_INIT);   //0  
      immc0_ctrl_0=MMC5983_CONTROL0_INIT;	
      mmc0_write_byte(MMC5983_CONTROL2_REG, 0x00);         

      //reset   
      mmc0_write_byte(MMC5983_CONTROL1_REG,MMC5983_CONTROL1_CHIP_RESET);  //reset
      LL_mDelay(20);        
      
      //set control 0  
      mmc0_write_byte(MMC5983_CONTROL0_REG,MMC5983_CONTROL0_INIT);   //0  
      immc0_ctrl_0=MMC5983_CONTROL0_INIT;			
       
      //set control 1   
      mmc0_write_byte(MMC5983_CONTROL1_REG,MMC5983_CONTROL1_BW_800);    //800Hz
    
   }
   else
   {
       //flg_mmc0_ready=0;
       iputs1("mmc0 err");   
   }     
   
}

void mmc0_read_data2(uint8_t *sdata, uint8_t ilen)
{
   static uint32_t ireadcount = 0;
 
   //start sample 
   mmc0_write_byte(MMC5983_CONTROL0_REG, immc0_ctrl_0 & 0x7f); 

   mmc0_write_byte(MMC5983_CONTROL0_REG, MMC5983_CONTROL0_MAG_MEAS);    //sample   
    //delay_count(2400);

   //read data
   mmc0_readfromspi(MMC5983_XOUT_REG, ilen, sdata);
   

   //set/reset + Temp command
   if (ireadcount>=1000)
   {
      ireadcount =0;

      mmc0_write_byte(MMC5983_CONTROL0_REG, immc0_ctrl_0 & 0x7f);            
      mmc0_write_byte(MMC5983_CONTROL0_REG, MMC5983_CONTROL0_TEMP_MEAS | MMC5983_CONTROL0_RESET ); 
      mmc0_write_byte(MMC5983_CONTROL1_REG, MMC5983_CONTROL1_BW_800);    //800Hz   
        
   }        
}    


//---------------------mmc0 module----end-----------------------


//---------------------mmc1 module---------------------------

uint8_t mmc1_read_byte(uint8_t addr)
{
   uint8_t value;

   LL_GPIO_ResetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_4);

   SPI_transfer(0x80|addr );
   value  = SPI_transfer(0x00);

   LL_GPIO_SetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_4);

   return value;
}

void mmc1_write_byte(uint8_t addr,  uint8_t value)
{ 
   LL_GPIO_ResetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_4);

   SPI_transfer(addr);
   SPI_transfer(value);

   LL_GPIO_SetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_4);
}

void mmc1_read_bytes(uint8_t addr ,uint8_t length ,uint8_t *buffer)
{
   //read data
   for(int index = 0; index < length; index++)
   {
      buffer[index]  = mmc1_read_byte(addr+index);
   } 
 
}

void mmc1_readfromspi( uint8_t iaddr, uint8_t ilen, uint8_t *idata)
{ 
   LL_GPIO_ResetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_4);      

   SPI_transfer(0x80 |iaddr);

   while(ilen--)
   {
      *idata  = SPI_transfer(0x00);
      idata++;
   }

   LL_GPIO_SetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_4);
}
 
void mmc1_init(void)
{
   uint8_t ivalue=0;

   //check device id
   ivalue = mmc1_read_byte(MMC5983_PRODUCT_ID_REG);  //read id    
   if(ivalue == MMC5983_PRODUCT_ID_RESULT)  //device id
   {
      //flg_mmc1_ready=1;      
      //iputs1("mmc1 ok");       

      //reset   
      mmc1_write_byte(MMC5983_CONTROL1_REG,MMC5983_CONTROL1_CHIP_RESET );  //reset
      LL_mDelay(20);                    

      //set control 0  
      mmc1_write_byte(MMC5983_CONTROL0_REG,MMC5983_CONTROL0_AUTOSREN|MMC5983_CONTROL0_INT_EN);  //auto ser/reset, enable interrupt    
       
      //set control 1   
      mmc1_write_byte(MMC5983_CONTROL1_REG,MMC5983_CONTROL1_BW_800);    //800Hz BW, Measur Time

      //set control 2   
      //mmc1_write_byte(MMC5983_CONTROL2_REG,MMC5983_CONTROL2_CM_FREQ_1000| MMC5983_CONTROL2_CMM_EN);  //1000Hz measure, continuous mode
      mmc1_write_byte(MMC5983_CONTROL2_REG,MMC5983_CONTROL2_CM_FREQ_1000| MMC5983_CONTROL2_CMM_EN|7<<4|0x80);  //1000Hz measure, continuous mode, 1000Hz prd set, enable prd set 

      //clear int
      ivalue = mmc1_read_byte(MMC5983_STATUS_REG);       //status  
      mmc1_write_byte(MMC5983_STATUS_REG, ivalue & 0x01);  
    
   }
   else
   {
      //flg_mmc1_ready=0;
      iputs1("mmc1 err");   
   }     
   
}

void mmc1_read_data(uint8_t *sdata, uint8_t ilen)
{
   uint8_t ivalue=0;   

   //clear int
   ivalue = mmc1_read_byte(MMC5983_STATUS_REG);
   mmc1_write_byte(MMC5983_STATUS_REG, ivalue & 0x01);

   //read data
   //mmc1_read_bytes(MMC5983_XOUT_REG, ilen, sdata);
   mmc1_readfromspi(MMC5983_XOUT_REG, ilen, sdata);
   
}   


void mmc1_init2(void)
{
   uint8_t ivalue=0;   

   //check device id
   ivalue = mmc1_read_byte(MMC5983_PRODUCT_ID_REG);  //read id    
   if(ivalue == MMC5983_PRODUCT_ID_RESULT)  //device id
   {
      iputs1("mmc1 ok");       
      //flg_mmc1_ready=1;   

      //unfrozen  test
      mmc1_write_byte(MMC5983_CONTROL0_REG,MMC5983_CONTROL0_AUTOSREN|MMC5983_CONTROL0_INT_EN);      //auto ser/reset, enable interrupt    
      mmc1_write_byte(MMC5983_CONTROL2_REG,MMC5983_CONTROL2_CM_FREQ_1000| MMC5983_CONTROL2_CMM_EN); //1000Hz, continuous mode
      LL_mDelay(1);  
      mmc1_write_byte(MMC5983_CONTROL0_REG,MMC5983_CONTROL0_INIT);   //0  
      immc1_ctrl_0=MMC5983_CONTROL0_INIT;		 
      mmc1_write_byte(MMC5983_CONTROL2_REG,0x00);       

      //reset   
      mmc1_write_byte(MMC5983_CONTROL1_REG,MMC5983_CONTROL1_CHIP_RESET);  //reset
      LL_mDelay(20);                    

      //set control 0  
      mmc1_write_byte(MMC5983_CONTROL0_REG,MMC5983_CONTROL0_INIT);   //0  
      immc1_ctrl_0	= MMC5983_CONTROL0_INIT;		
       
      //set control 1   
      mmc1_write_byte(MMC5983_CONTROL1_REG,MMC5983_CONTROL1_BW_800);    //800Hz
    
   }
   else
   {
      //flg_mmc1_ready=0;
      iputs1("mmc1 err");   
   }     
   
}

void mmc1_read_data2(uint8_t *sdata, uint8_t ilen)
{
   static uint32_t ireadcount = 0;

   //start sample
   mmc1_write_byte(MMC5983_CONTROL0_REG, immc1_ctrl_0 & 0x7f);  	
   mmc1_write_byte(MMC5983_CONTROL0_REG, MMC5983_CONTROL0_MAG_MEAS);    //sample

   //read data
   mmc1_readfromspi(MMC5983_XOUT_REG, ilen, sdata);

   //set/reset + Temp command
   if (ireadcount>=1000)
   {
      ireadcount =0;    
      mmc1_write_byte(MMC5983_CONTROL0_REG, immc1_ctrl_0 & 0x7f);      
      mmc1_write_byte(MMC5983_CONTROL0_REG, MMC5983_CONTROL0_TEMP_MEAS | MMC5983_CONTROL0_RESET ); 
      mmc1_write_byte(MMC5983_CONTROL1_REG, MMC5983_CONTROL1_BW_800);    //800Hz   
   }   
   
}   


//---------------------mmc1 module----end-----------------------


//---------------------icm0 module---------------------------

uint8_t icm0_read_byte(uint8_t addr)
{
   uint8_t value;

   LL_GPIO_ResetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_2);

   SPI_transfer(0x80|addr );
   value  = SPI_transfer(0x00);

   LL_GPIO_SetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_2);

   return value;
}

void icm0_write_byte(uint8_t addr,  uint8_t value)
{ 
   LL_GPIO_ResetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_2);

   SPI_transfer(addr);
   SPI_transfer(value);

   LL_GPIO_SetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_2); 
}

void icm0_read_bytes(uint8_t addr ,uint8_t length ,uint8_t *buffer)
{
   //read data
   for(int index = 0; index < length; index++)
   {
      buffer[index]  = icm0_read_byte(addr+index);
   } 
 
}

void icm0_readfromspi( uint8_t iaddr, uint8_t ilen, uint8_t *idata)
{ 
   LL_GPIO_ResetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_2);      

   SPI_transfer(0x80 |iaddr);

   while(ilen--)
   {
      *idata  = SPI_transfer(0x00);
      idata++;
   }

   LL_GPIO_SetOutputPin((GPIO_TypeDef *)GPIOA, LL_GPIO_PIN_2); 
}
   

void icm0_init(void)   
{
   uint8_t ivalue=0;

   icm0_write_byte( ICM42605_REG_BANK_SEL_REG, 0x00);    // set bank0

   //check device id
   ivalue =icm0_read_byte(ICM42605_WHO_AM_I_REG);     //read device id 
   if(ivalue == ICM_DEVICE_ID)  
   {
      //flg_icm0_ready=1;      
      iputs1("icm0 ok");       

      icm0_write_byte(ICM42605_DEVICE_CONFIG_REG, 0x01 );     //reset   
      LL_mDelay(100);     

      icm0_write_byte(ICM42605_PWR_MGMT0_REG, 0x0f);     // set GYRO and ACC in LN mode    
      LL_mDelay(1);

      icm0_write_byte( ICM42605_GYRO_CONFIG0_REG, 0x06 | (3 << 5) );  //GYRO ODR 1KHz,GYRO FS 250dps   
      icm0_write_byte(ICM42605_ACCEL_CONFIG0_REG, 0x06 | (3 << 5) );   //ACC ODR 1kHz, ACC FS  2g
      icm0_write_byte( ICM42605_GYRO_CONFIG1_REG, 0xD6);              //DLPF BW=5Hz; DLPF Latency=32ms, GYRO UI 2nd Order , DEC2 M2 3nd Order
      icm0_write_byte( ICM42605_INT_CONFIG_REG, 0x03);    //INT1  Push pull, Active high
      icm0_write_byte( ICM42605_INT_CONFIG1_REG, 0x00);    //proper INT1 and INT2 pin operation
      icm0_write_byte(ICM42605_INT_SOURCE0_REG, 0x09);     //UI data ready interrupt routed to INT1, UI AGC ready interrupt routed to INT1

      icm0_write_byte( ICM42605_REG_BANK_SEL_REG, 0x04);    //set bank 4     
      icm0_write_byte( ICM42605_INT_SOURCE0_REG, 0x8F);    //Defines mounting matrix, chip to device frame
      icm0_write_byte( ICM42605_REG_BANK_SEL_REG, 0x00);    //set bank0

      ivalue=icm0_read_byte(0x2d);     //INT STATUS
   }
   else
   {
      //flg_icm0_ready=0;
      iputs1("icm0 err");   
   }     
   
}
 

void icm0_read_data(uint8_t *sdata, uint8_t ilen) 
{
   uint8_t ivalue=0;  

   icm0_write_byte( 0x76, 0x00);    // set bank0	
   ivalue=icm0_read_byte(0x2d);      //INT STATUS

   //icm0_read_bytes(0x1f, ilen, sdata);
   icm0_readfromspi(0x1f, ilen, sdata);
}   

//---------------------icm0 module----end-----------------------



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
   uint8_t ivalue=0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  LL_SYSCFG_EnablePinRemap(LL_SYSCFG_PIN_RMP_PA11);
  LL_SYSCFG_EnablePinRemap(LL_SYSCFG_PIN_RMP_PA12);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
   LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_SYSCLK, LL_RCC_MCO1_DIV_4);   //12Mhz = 48Mhz/4

   iputs1("start...\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

   LL_SPI_Enable(SPI1);     

   //sensor init 
   mmc0_init2();
   mmc1_init2();
   icm0_init();

   
   while (1)
   {
      static uint32_t  itxcount=0;
      static uint32_t  isensorcount=0;
      static uint32_t  iledcount=0;
      static uint32_t  iresetcount=0;
      
      //LL_mDelay(1000);     

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
      //timer1
      if(flg_timer1==1)
      {
         flg_timer1=0;

         itxcount++;
         
         isensorcount++;
         iledcount++;
         iresetcount++;
      }		

      //dowith_uart_tx();  
      //dowith_uart_rx();  
      
#if (defined TXMODE) 
      uint8_t itxcmddata[4];
      
      if(itxcount>=1)   //1* 1ms =1ms
      {
         itxcount=0;   
         
         itxcmddata[0]=CMDHEADER2;   //0x55;
         itxcmddata[1]=CMDHEADER1;   //0xAA;
         itxcmddata[2]=CMDEND1;      //0x0D;
         itxcmddata[3]=CMDEND2;      //0x0A;
         iputbytes1(itxcmddata,4);
          
      }
#else      
      
      if (flg_rx==1)
      {
         flg_rx=0;
         //iputs1("rx in\r\n" );         
         //iputbytes1((uint8_t *)irxdata2, irxcount2);
         
         uint16_t itimnum = LL_TIM_GetCounter(TIM17);
         ireportdata[ DATA_TIME_POINT] = (uint8_t)(itimnum & 0xff);
         ireportdata[ DATA_TIME_POINT+1] = (uint8_t)( (itimnum>>8) & 0xff );
         
         iputbytes1(ireportdata,32);
      }
 
      //read sensor data
      if(isensorcount>=10)   //1* 1ms =1ms
      {
         isensorcount=0;          

         memset(ireportdata,0,32);

         ireportdata[DATA_HEADER_POINT]=CMDHEADER1;     //0xAA;
         ireportdata[DATA_HEADER_POINT+1]=CMDHEADER2;   //0x55;
         ireportdata[DATA_ENDER_POINT]=CMDEND1;         //0x0D;
         ireportdata[DATA_ENDER_POINT+1]=CMDEND2;        //0x0A;
         
         mmc0_read_data2(ireportdata+DATA_MMC0_POINT,7);    
         mmc1_read_data2(ireportdata+DATA_MMC1_POINT,7);  
         icm0_read_data(ireportdata+DATA_ICM0_POINT,12);
         
         //iputbytes1(ireportdata,32);
         
      }		
      
      if(iresetcount>=1000)
      {
         iresetcount=0;
         
         mmc0_write_byte(MMC5983_CONTROL0_REG, MMC5983_CONTROL0_RESET );           
         //mmc0_write_byte(MMC5983_CONTROL0_REG, MMC5983_CONTROL0_SET );          
   
         mmc1_write_byte(MMC5983_CONTROL0_REG, MMC5983_CONTROL0_RESET );           
         //mmc1_write_byte(MMC5983_CONTROL0_REG, MMC5983_CONTROL0_SET );   
      }
      
   #endif
    
      //led
      if(iledcount>=1000)   //1000* 1ms =1ms
      {
         iledcount=0;

         LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  
      }

   }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_SetHSIDiv(LL_RCC_HSI_DIV_1);
  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_HCLK_DIV_1);

  /* Sysclk activation on the HSI */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(48000000);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(48000000);
  LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_SYSCLK, LL_RCC_MCO1_DIV_1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
   /* User can add his own implementation to report the HAL error return state */
   __disable_irq();
   while (1)
   {
   }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
   /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
