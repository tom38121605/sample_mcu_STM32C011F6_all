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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
   while( (USART1-> ISR & (1<<7) ) ==0 );
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



uint8_t i2c_writereg(uint16_t idevaddr, uint8_t ireg,uint8_t data)
{
   if((HAL_I2C_Mem_Write(&hi2c1,idevaddr,ireg,I2C_MEMADD_SIZE_8BIT,&data,1,1000)) == HAL_OK)
   {
      return 0;
   }
   else
   {
      return 1;
   }
}

uint8_t i2c_readreg(uint16_t idevaddr,uint8_t ireg)
{
   uint8_t res;
   HAL_I2C_Mem_Read(&hi2c1,idevaddr,ireg,I2C_MEMADD_SIZE_8BIT,&res,1,1000);
   return res;
}

uint8_t i2c_writebytes(uint16_t idevaddr, uint8_t ireg, uint8_t *sbuf,uint16_t ilen)
{
   if((HAL_I2C_Mem_Write(&hi2c1,idevaddr,ireg,I2C_MEMADD_SIZE_8BIT,sbuf,ilen,1000)) == HAL_OK)
   {
      return 0;
   }
   else
   {
      return 1;
   }
}

uint8_t i2c_readbytes(uint16_t idevaddr,uint8_t ireg,uint8_t *sbuf,uint16_t ilen)
{
   if((HAL_I2C_Mem_Read(&hi2c1,idevaddr,ireg,I2C_MEMADD_SIZE_8BIT,sbuf,ilen,1000)) == HAL_OK)
   {
      return 0;
   }
   else
   {
      return 1;
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

   LL_mDelay(10);  //20
   iputs1( "start..." );     //start  
   
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
   LL_mDelay(1000);    
     
//-------------test 24c02 onebyte----------------------
     
//uartsendcmd((uint8_t*)"run\r\n",5); 
     
   uint8_t itemp=0;
   uint32_t idelay=50000;
        
   i2c_writereg( 0xA0,0x03,6);   
   delay_nop(idelay);        
   itemp=i2c_readreg( 0xA0,0x03); 

   iputbytes1( (uint8_t*)&itemp,1);      
        
   i2c_writereg( 0xA0,0x03,8);
   delay_nop(idelay);     
   itemp=i2c_readreg( 0xA0,0x03);    
   iputbytes1( (uint8_t*)&itemp,1);  
  
//-------------test 24c02 onebyte----end------------------

     
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
  LL_SetSystemCoreClock(48000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
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
