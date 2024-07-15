/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>           //add for sprintf
#include "string.h"          //add for memcmp

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


#define TXMODE
//#define RXMODE

#define NOUSE_LEN_TX  0   //0--257 , 1 -- 256, 2 -- 255, 3 -- 254
#define NOUSE_LEN_RX  0   //0--cmp 257 , 1 --cmp 256, 2 --cmp 255, 3 --cmp 254

extern volatile uint8_t flg_rx;
extern volatile uint8_t irxdata[RXDATALEN];
extern volatile uint8_t irxdata2[RXDATALEN];
extern volatile uint32_t irxnum;
extern volatile uint32_t irxnum2;

extern volatile uint8_t timer1_flag;

 
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


uint8_t TESTTXDATA[TXDATALEN]=
{0x00,
 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F, 
 0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F,0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F, 
 0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x5B,0x5C,0x5D,0x5E,0x5F, 
 0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F,0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x7B,0x7C,0x7D,0x7E,0x7F, 
 0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8A,0x8B,0x8C,0x8D,0x8E,0x8F,0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0x9B,0x9C,0x9D,0x9E,0x9F, 
 0xA0,0xA1,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8,0xA9,0xAA,0xAB,0xAC,0xAD,0xAE,0xAF,0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xBB,0xBC,0xBD,0xBE,0xBF, 
 0xC0,0xC1,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xCB,0xCC,0xCD,0xCE,0xCF,0xD0,0xD1,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0xDB,0xDC,0xDD,0xDE,0xDF, 
 0xE0,0xE1,0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xEB,0xEC,0xED,0xEE,0xEF,0xF0,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,0xF9,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF 
};

uint8_t TESTRXDATA[RXDATALEN]=
{ 0X00,
 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F, 
 0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F,0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F, 
 0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x5B,0x5C,0x5D,0x5E,0x5F, 
 0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F,0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x7B,0x7C,0x7D,0x7E,0x7F, 
 0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8A,0x8B,0x8C,0x8D,0x8E,0x8F,0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0x9B,0x9C,0x9D,0x9E,0x9F, 
 0xA0,0xA1,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8,0xA9,0xAA,0xAB,0xAC,0xAD,0xAE,0xAF,0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xBB,0xBC,0xBD,0xBE,0xBF, 
 0xC0,0xC1,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xCB,0xCC,0xCD,0xCE,0xCF,0xD0,0xD1,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0xDB,0xDC,0xDD,0xDE,0xDF, 
 0xE0,0xE1,0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xEB,0xEC,0xED,0xEE,0xEF,0xF0,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,0xF9,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF 
};


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

///* Variables for ADC conversion data */
//__IO uint16_t uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE; /* ADC group regular conversion data */

///* Variables for ADC conversion data computation to physical values */
//uint16_t uhADCxConvertedData_Voltage_mVolt = 0;  /* Value of voltage calculated from ADC conversion data (unit: mV) */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//void ADC_Activate(void);
//void ConversionStartPoll_ADC_GrpRegular(void);

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



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_SYSCLK, LL_RCC_MCO1_DIV_4);//12Mhz = 48Mhz/4
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  
  iputs1("start...");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  /* Activate ADC */
  /* Perform ADC activation procedure to make it ready to convert. */
  //ADC_Activate();


  while (1)
  {
    //LL_mDelay(1000);
     static uint8_t flg_dataerr =0;
		
		 static uint32_t iuart_txcount=0;
		 static uint32_t iuart_txledcount=0;
		 static uint32_t iuart_rxcount=0;
		 static uint32_t iuart_rxledcount=0;
     
      /////* Perform ADC group regular conversion start, poll for conversion        */
      /////* completion.                                                            */
      ////ConversionStartPoll_ADC_GrpRegular();

      /////* Retrieve ADC conversion data */
      ////uhADCxConvertedData = LL_ADC_REG_ReadConversionData32(ADC1);

      /////* Computation of ADC conversions raw data to physical values             */
      /////* using helper macro.                                                    */
      ////uhADCxConvertedData_Voltage_mVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, uhADCxConvertedData, LL_ADC_RESOLUTION_12B);
     
     
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		if(timer1_flag==1)
		{
			timer1_flag=0;
			
			iuart_txcount++;
			iuart_txledcount++;
			iuart_rxcount++;
			iuart_rxledcount++;		
		}

		if(itimecount>=8)
		{
			itimecount=0;
			irxnum=0;			
		}

#if (defined TXMODE)  		 
     //uart tx
     if(iuart_txcount>=30)   // 30ms
     {
         iuart_txcount=0; 

         if(NOUSE_LEN_TX==0)  //257
            iputbytes1((uint8_t *)TESTTXDATA, TXDATALEN);
         else if(NOUSE_LEN_TX==1)  //256
            iputbytes1((uint8_t *)TESTTXDATA + NOUSE_LEN_TX, TXDATALEN - NOUSE_LEN_TX);
         else  
            iputbytes1((uint8_t *)TESTTXDATA + NOUSE_LEN_TX, TXDATALEN - NOUSE_LEN_TX);
        
     }  
		 
		if( iuart_txledcount >=1000)
		{	
			 iuart_txledcount=0;		 

			 LL_GPIO_TogglePin( LED_GPIO_Port, LED_Pin);
		}		 
		 
#else

		if( iuart_rxcount >=1000)
		{	
			 uint8_t igap =0;
			
			 iuart_rxcount=0;
			
			 //uart rx
			 if (flg_rx==1)
			 {
					 flg_rx=0;

					 //iputs1("rx in\r\n");

					 iputbytes1((uint8_t *)irxdata2,irxnum2);
				 
//				   if(irxnum2==257)
//						 igap=3;
//					 else if(irxnum2==256)
//						 igap=2; 					 
//					 else if(irxnum2==255)
//						 igap=1; 
//					 else if(irxnum2==254)
//						 igap=0; 
             
				   if(irxnum2==257)
						 igap=NOUSE_LEN_RX;
					 else if(irxnum2==256)
						 igap=NOUSE_LEN_RX-1; 		//NOUSE_LEN>=1			 
					 else if(irxnum2==255)
						 igap=NOUSE_LEN_RX-2; 		//NOUSE_LEN>=2		 
					 else if(irxnum2==254)
						 igap=NOUSE_LEN_RX-3; 		//NOUSE_LEN>=3		 
					 
					 //Send_Data_To_UART1(irxnum2);

					 //if(memcmp((void*)TESTRXDATA+3, (void*)irxdata2+igap, RXDATALEN-3) ==0)
					 if(memcmp((void*)TESTRXDATA + NOUSE_LEN_RX, (void*)irxdata2+igap, RXDATALEN - NOUSE_LEN_RX) ==0)
					 {
						 //iputs1("dataok\r\n");
						 flg_dataerr=0;
					 }
					 else
					 {
						 //iputs1("dataerr\r\n");
						 flg_dataerr=1;
					 }
					 
					 
						if( iuart_rxledcount >=1000)
						{	
							 iuart_rxledcount=0;		 
							
							 if(flg_dataerr==0)
								 LL_GPIO_TogglePin( LED_GPIO_Port, LED_Pin);
														
						}			
					 
			 }    
		 }	


		
#endif  		 
 
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
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(48000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
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
