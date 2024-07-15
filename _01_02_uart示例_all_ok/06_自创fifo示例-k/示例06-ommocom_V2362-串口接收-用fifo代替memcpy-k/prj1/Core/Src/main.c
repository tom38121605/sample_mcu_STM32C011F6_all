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



//#define TXMODE
#define RXMODE

extern volatile uint8_t flg_rx;
extern volatile uint8_t irxdata[2][TXRXDATALEN+2];
extern volatile uint8_t irxdata2[TXRXDATALEN+2];
extern volatile uint32_t irxnum;
extern volatile uint8_t icurrentindex;

extern volatile uint8_t flg_rxin1;
extern volatile uint8_t flg_rxin2;
extern volatile uint8_t irxcount;
extern volatile uint8_t irxcount2;


extern volatile uint8_t timer1_flag;
extern volatile uint32_t itimecount;
 
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


uint8_t TESTDATA2[TXRXDATALEN]=
{
 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D
};


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
     static uint32_t itxcount=0;  
     uint8_t stxout[TXRXDATALEN+2]={0};
   
      //uart tx
      if(itimecount>=1)   //1* 1ms =1ms
      {
         itimecount=0; 
        
        //Send_Data_To_UART1(uhADCxConvertedData);
        
         #if (defined TXMODE)    
        
            itxcount++;
        
            stxout[0]=CMDHEADER1;
            memcpy( stxout+1, TESTDATA2, TXRXDATALEN);
            //stxout[2]=itxcount;  //--test         
         
            stxout[TXRXDATALEN+1]=get_checksum_8(stxout,TXRXDATALEN+1);
         
            iputbytes1((uint8_t *)stxout,TXRXDATALEN+2);
        
          #endif  
       
      }  
     
      #if (defined TXMODE) 
      if(itxcount >=1000)  //1s
      {
         itxcount=0;
             
         LL_GPIO_TogglePin( LED_GPIO_Port, LED_Pin);
         LL_GPIO_TogglePin( GPIOB, LL_GPIO_PIN_7);
           
      }        
      #endif  
}


void dowith_uart_rx(void)
{  
     static uint8_t flg_dataerr =0;   
     static uint32_t irxcount=0;  
     uint8_t ichksum =0;   
     uint8_t iuserindex =0;   
   
     //uart rx     
     if (flg_rx==1)
     {
        flg_rx=0;
        
        irxcount++;        
        
        if (icurrentindex==1)
           iuserindex=0;
        else
           iuserindex=1;        

        //iputbytes1((uint8_t *)irxdata2,TXRXDATALEN+2);
        //iputbytes1((uint8_t *)irxdata[iuserindex],TXRXDATALEN+2);
           
        //ichksum = get_checksum_8((uint8_t *)irxdata2,TXRXDATALEN+1);
        ichksum = get_checksum_8( (uint8_t *)irxdata[iuserindex],TXRXDATALEN+1);

         //if( ichksum == irxdata2[TXRXDATALEN+1] )
         if( ichksum == irxdata[iuserindex][TXRXDATALEN+1] )
         {
           iputs1("ok");
           //flg_dataerr=0;
         }
         else
         {
            iputs1("er");
            flg_dataerr=1;
         }       
        
     }     
 
      if(irxcount >=1000)  //1s
      {
         irxcount=0;
         
         if(flg_dataerr==0)     
         {            
            LL_GPIO_TogglePin( LED_GPIO_Port, LED_Pin);
            LL_GPIO_TogglePin( GPIOB, LL_GPIO_PIN_7);
         }
            
         flg_dataerr=0;            
      }  
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_SYSCLK, LL_RCC_MCO1_DIV_4);   //12Mhz = 48Mhz/4
  
  iputs1("start...");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  

  while (1)
  {
    //LL_mDelay(1000);
     
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    dowith_uart_tx();

    dowith_uart_rx();     
 
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
