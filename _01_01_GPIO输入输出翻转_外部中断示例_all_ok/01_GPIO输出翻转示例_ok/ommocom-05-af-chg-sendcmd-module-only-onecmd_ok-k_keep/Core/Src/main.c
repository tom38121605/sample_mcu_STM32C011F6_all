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
#include <stdio.h>           
#include "string.h"           
#include "spi.h"              

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//uint8_t ireportdata[32];
uint8_t ireportdata[TXDATALEN];

#define DATA_HEADER_POINT  0
#define DATA_MMC0_POINT  2
#define DATA_MMC1_POINT  9
#define DATA_ICM0_POINT  16
#define DATA_TIME_POINT  28
#define DATA_ENDER_POINT 30

uint8_t mastercmd[72]={0};    


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

  
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


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

void delay_nop(uint32_t nCount) 
{ 
   for(; nCount != 0; nCount--)  
   { 
      __NOP(); 
   } 
}

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


////void mastersendcmd(void)
////{
////   //switch to tx 
////   //delay_nop(WAIT2TE_MASTER);  
////   switch2send(USART1); 

////   iputbytes1((uint8_t *)mastercmd,4);     //{0x55, 0xAA, 0x0D, 0x0A};   

////   //switch to rx
////   delay_nop(WAIT2RE0_MASTER);    
////   switch2read(USART1);   
////}   

void mastersendcmd(uint8_t *scmd, uint32_t ilen)
{
   //switch to tx 
   //delay_nop(WAIT2TE_MASTER);  
   switch2send(USART1); 

   //iputbytes1((uint8_t *)scmd,ilen);     
   iputbytes1(scmd,ilen);     

   //switch to rx
   delay_nop(WAIT2RE0_MASTER);    
   switch2read(USART1);   
}   
 
 

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
   //uint8_t ivalue=0;

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

   //switch to tx 
   switch2send(USART1); 
   iputs1("start...");  
   
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


   //LL_GPIO_SetOutputPin( LED_GPIO_Port, LED_Pin);  

   while (1)
   { 
      
      static uint32_t iuart_txcount=0;
      static uint32_t iuart_txledcount=0;        

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      if(flg_timer1==1)
      {
         flg_timer1=0;
         
         iuart_txcount++;
         iuart_txledcount++;   
      }      
       
      if(iuart_txcount>=100)
      {    
         iuart_txcount=0;  
         mastercmd[0]=0x55;
         mastercmd[1]=0xAA;
         mastercmd[2]=0x0D;
         mastercmd[3]=0x0A;      

         //if (flg_send ==1)         
           mastersendcmd(mastercmd,4);
      }            
      
		if( iuart_txledcount >=1000)
		{	
			 iuart_txledcount=0;
			 LL_GPIO_TogglePin( LED_GPIO_Port, LED_Pin);
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
