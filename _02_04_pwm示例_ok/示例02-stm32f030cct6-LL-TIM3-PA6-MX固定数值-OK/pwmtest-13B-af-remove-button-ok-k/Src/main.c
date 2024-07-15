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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Number of output compare modes */
#define TIM_DUTY_CYCLES_NB 11

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Duty cycles: D = T/P * 100%                                                */
/* where T is the pulse duration and P  the period of the PWM signal          */
static uint32_t aDutyCycle[TIM_DUTY_CYCLES_NB] = {
  0,    /*  0% */
  10,   /* 10% */
  20,   /* 20% */
  30,   /* 30% */
  40,   /* 40% */
  50,   /* 50% */
  60,   /* 60% */
  70,   /* 70% */
  80,   /* 80% */
  90,   /* 90% */
  100,  /* 100% */
};

/* Duty cycle index */
static uint8_t iDutyCycle = 0;

/* Measured duty cycle */
__IO uint32_t uwMeasuredDutyCycle = 0;

/* TIM2 Clock */
//static uint32_t TimOutClock = 1;
////uint32_t timxPrescaler = 0;
////uint32_t timxPeriod = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
__STATIC_INLINE void Configure_DutyCycle(uint32_t D);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* - Set the pre-scaler value to have TIM3 counter clock equal to 10 kHz  */
  /* - Set the auto-reload value to have a counter frequency of 100 Hz        */
  /* TIM3CLK = SystemCoreClock / (APB prescaler & multiplier)               */
////  TimOutClock = SystemCoreClock/1;
////  timxPrescaler = __LL_TIM_CALC_PSC(SystemCoreClock, 10000);        //tim 10k
////  timxPeriod = __LL_TIM_CALC_ARR(TimOutClock, timxPrescaler, 100);  //pwm  100Hz
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //LL_TIM_EnableIT_CC1(TIM3);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableCounter(TIM3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
     //uint32_t itimcount;
     uint32_t iarr, iduty ;
        
     //itimcount = LL_TIM_GetCounter(TIM3);
     iarr = LL_TIM_GetAutoReload(TIM3);  
     iduty=LL_TIM_OC_GetCompareCH1(TIM3);
        
     if(iduty<iarr)
        iduty++;
     else
        iduty=0;
    
     LL_TIM_OC_SetCompareCH1(TIM3, iduty);     
     
     LL_mDelay(100);	
          
     
     
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
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Changes the duty cycle of the PWM signal.
  *         D = (T/P)*100
  *           where T is the pulse duration and P is the PWM signal period
  * @param  D Duty cycle
  * @retval None
  */
__STATIC_INLINE void Configure_DutyCycle(uint32_t D)
{
  uint32_t P;    /* Pulse duration */
  uint32_t T;    /* PWM signal period */
  
  /* PWM signal period is determined by the value of the auto-reload register */
  T = LL_TIM_GetAutoReload(TIM3) + 1;
  
  /* Pulse duration is determined by the value of the compare register.       */
  /* Its value is calculated in order to match the requested duty cycle.      */
  P = (D*T)/100;
  LL_TIM_OC_SetCompareCH1(TIM3, P);
}

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/
/**
  * @brief  User button interrupt processing
  * @note   When the user key button is pressed the PWM duty cycle is updated. 
  * @param  None
  * @retval None
  */
void UserButton_Callback(void)
{
  /* Set new duty cycle */
  iDutyCycle = (iDutyCycle + 1) % TIM_DUTY_CYCLES_NB;

  /* Change PWM signal duty cycle */
  Configure_DutyCycle(aDutyCycle[iDutyCycle]);
}

/**
  * @brief  Timer capture/compare interrupt processing
  * @param  None
  * @retval None
  */
void TimerCaptureCompare_Callback(void)
{
  uint32_t CNT, ARR;
  CNT = LL_TIM_GetCounter(TIM3);
  ARR = LL_TIM_GetAutoReload(TIM3);
  
  if (LL_TIM_OC_GetCompareCH1(TIM3) > ARR )
  {
    /* If capture/compare setting is greater than autoreload, there is a counter overflow and counter restarts from 0.
       Need to add full period to counter value (ARR+1)  */
    CNT = CNT + ARR + 1;
  }
  uwMeasuredDutyCycle = (CNT * 100) / ( ARR + 1 );
}

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
