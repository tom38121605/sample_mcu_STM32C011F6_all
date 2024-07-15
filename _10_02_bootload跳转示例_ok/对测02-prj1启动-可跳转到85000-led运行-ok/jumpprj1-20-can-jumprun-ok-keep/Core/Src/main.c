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


extern volatile uint8_t flg_rx;
extern volatile uint8_t irxdata[RXDATALEN+2];
extern volatile uint8_t irxdata2[RXDATALEN+2];

extern volatile uint32_t irxcount;
extern volatile uint32_t irxcount2;

extern volatile uint8_t flg_timer1;

 
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

typedef void (*iapfun)(void);
iapfun jump2app;

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


void delay_count(uint32_t icount)
{
   uint32_t inum = icount;
   while (inum--) ;
}


//__asm void MSR_MSP(uint32_t addr)
//{
//   MSR MSP, r0  
//   BX r14
//}

void MSR_MSP(uint32_t addr)
{   
   __ASM("MSR MSP, r0");  
   __ASM(" BX r14");  
   
//   __ASM("msr msp, r0");  
//   __ASM("bx lr");  
}

//void MSR_MSP(uint32_t addr)
//{
//   __set_PSP(*(__IO uint32_t*)addr);
//   __set_CONTROL(0);
//   __set_MSP(*(__IO uint32_t*) addr);  

//           __ASM volatile ("MSR psp, %0" : : "r" (addr) : );
//           __ASM volatile ("MSR control, %0" : : "r" (0) : "memory");
//           __ISB();
//           __ASM volatile ("MSR msp, %0" : : "r" (addr) : );

//}


////void iap_load_app(uint32_t appxaddr)  //0x08050000
////{
////   iputs1("call\r\n");   
////   
////   if(((*(__IO uint32_t*)appxaddr)&0x2FFE0000)==0x20000000)  
////   {      
////      iputs1("yes\r\n");    
////      
////      jump2app=(iapfun)*(__IO uint32_t*)(appxaddr+4);  
////      MSR_MSP(*(__IO uint32_t*)appxaddr);  
////      jump2app();  
////   }
////   else
////     iputs1("no\r\n");      
////      
////}

void iap_load_app(uint32_t appxaddr)
{
   
   if(((*(__IO uint32_t*)appxaddr)&0x2FFE0000)==0x20000000)  
   {     
      
////      while(1)
////      {
////         LL_GPIO_TogglePin( LED_GPIO_Port, LED_Pin); ;
////         LL_mDelay(2000);      
////      }         
      
      jump2app=(iapfun)*(__IO uint32_t*)(appxaddr+4);  
      //MSR_MSP(*(__IO uint32_t*)appxaddr);  
      jump2app();  
      
      LL_GPIO_SetOutputPin( LED_GPIO_Port, LED_Pin);  //led off
      
   }
    
      
}


/* USER CODE END 0 */



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
////  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
////  LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_SYSCLK, LL_RCC_MCO1_DIV_4);   //12Mhz = 48Mhz/4

  iputs1("start2...\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  
  while (1)
  {
      __IO uint32_t data32 = 0;  

      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */  
      LL_mDelay(200);
     
      //data32 = *(__IO uint32_t *)0x8003000;     

      //if(data32 ==0x200006A8)  
      //   LL_GPIO_TogglePin( LED_GPIO_Port, LED_Pin);   //led toggle
      //else
      //   LL_GPIO_SetOutputPin( LED_GPIO_Port, LED_Pin);  //led off
      
      iap_load_app(0x8005000);
 
  }
  

   while(1)
   {
      iputs1("abc2...\r\n");  
      LL_mDelay(1000);  
      
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */

      iap_load_app(FLASH_SAVE_ADDR);  //0x08030000
      
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
