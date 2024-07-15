/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_rgbled.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

void Send_Data_To_UART1 (uint8_t c)
{
   USART1->TDR = c;
   while( (USART1-> ISR & (1<<7) ) ==0 );  //USART_ISR_TC  //USART_ISR_TXE_TXFNF
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

void uartsendcmd(uint8_t *scmd, uint32_t ilen)
{
   //switch to tx 
   delay_nop(WAIT2TE_SLAVE); //delay_nop(WAIT2TE_MASTER);  
   switch2send(USART1); 

   //iputbytes1((uint8_t *)scmd,ilen);     
   iputbytes1(scmd,ilen);     

   //switch to rx
   delay_nop(WAIT2RE_SLAVE);  //delay_nop(WAIT2RE0_MASTER);    
   switch2read(USART1);   
}   

void uartsendstr(uint8_t *str1)
{
   //switch to tx 
   delay_nop(WAIT2TE_SLAVE); //delay_nop(WAIT2TE_MASTER);  
   switch2send(USART1); 

   iputs1((char*)str1);   

   //switch to rx
   delay_nop(WAIT2RE_SLAVE);  //delay_nop(WAIT2RE0_MASTER);    
   switch2read(USART1);   
}   

void spi_transmitbytes(uint8_t *sdata, uint16_t inum)
{
   for(uint16_t i =0; i< inum; i++)
   {
      LL_SPI_TransmitData8(SPI1, sdata[i]);
      while(!LL_SPI_IsActiveFlag_TXE(SPI1));
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
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

   uartsendstr( (uint8_t*)"start...");
   
   LL_SPI_Enable(SPI1);   
   
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1)
   {  
      static uint32_t imaincount=0;
      static uint32_t printcount=0;
      imaincount++;      

      if(imaincount>10000000)
      {
         imaincount=0;
         
         rgbled_task();	
         
         if(printcount<8) printcount++;         
         //if(printcount<8)
            move_color();

         
         LL_GPIO_TogglePin( DATA_EN1_GPIO_Port, DATA_EN1_Pin);  
      }      
      
      //LL_mDelay(1000);	
      
      
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PREDIV_DIV_1);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(48000000);
  LL_SetSystemCoreClock(48000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
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
