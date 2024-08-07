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


//#define TXMODE
#define RXMODE

extern volatile uint8_t flg_rx;
extern volatile uint8_t irxdata[256];
extern volatile uint8_t irxdata2[256];
extern volatile uint32_t irxnum;

extern volatile uint8_t timer1_flag;
extern  volatile uint32_t itimecount;
 
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

uint8_t TESTDATA[256]=
{
 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F, 
 0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F,0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F, 
 0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x5B,0x5C,0x5D,0x5E,0x5F, 
 0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F,0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x7B,0x7C,0x7D,0x7E,0x7F, 
 0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8A,0x8B,0x8C,0x8D,0x8E,0x8F,0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0x9B,0x9C,0x9D,0x9E,0x9F, 
 0xA0,0xA1,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8,0xA9,0xAA,0xAB,0xAC,0xAD,0xAE,0xAF,0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xBB,0xBC,0xBD,0xBE,0xBF, 
 0xC0,0xC1,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xCB,0xCC,0xCD,0xCE,0xCF,0xD0,0xD1,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0xDB,0xDC,0xDD,0xDE,0xDF, 
 0xE0,0xE1,0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xEB,0xEC,0xED,0xEE,0xEF,0xF0,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,0xF9,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF 
};

uint8_t TESTDATA2[TXRXDATALEN]=
{
 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D
};

uint8_t txstr[TXRXDATALEN+2]={0};


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//uint8_t ommo_mode = DEFAULT_MODE;
//uint8_t ommo_status = DEFAULT_STATUS;

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

/**
  * @brief  Perform ADC activation procedure to make it ready to convert
  *         (ADC instance: ADC1).
  * @param  None
  * @retval None
  */
//void ADC_Activate(void)
//{
//  __IO uint32_t wait_loop_index = 0U;
//  __IO uint32_t backup_setting_adc_dma_transfer = 0U;
//  #if (USE_TIMEOUT == 1)
//  uint32_t Timeout = 0U; /* Variable used for timeout management */
//  #endif /* USE_TIMEOUT */

//  /*## Operation on ADC hierarchical scope: ADC instance #####################*/

//  /* Note: Hardware constraint (refer to description of the functions         */
//  /*       below):                                                            */
//  /*       On this STM32 series, setting of these features is conditioned to  */
//  /*       ADC state:                                                         */
//  /*       ADC must be disabled.                                              */
//  /* Note: In this example, all these checks are not necessary but are        */
//  /*       implemented anyway to show the best practice usages                */
//  /*       corresponding to reference manual procedure.                       */
//  /*       Software can be optimized by removing some of these checks, if     */
//  /*       they are not relevant considering previous settings and actions    */
//  /*       in user application.                                               */
//  if (LL_ADC_IsEnabled(ADC1) == 0)
//  {
//    /* Enable ADC internal voltage regulator */
//    LL_ADC_EnableInternalRegulator(ADC1);

//    /* Delay for ADC internal voltage regulator stabilization.                */
//    /* Compute number of CPU cycles to wait for, from delay in us.            */
//    /* Note: Variable divided by 2 to compensate partially                    */
//    /*       CPU processing cycles (depends on compilation optimization).     */
//    /* Note: If system core clock frequency is below 200kHz, wait time        */
//    /*       is only a few CPU processing cycles.                             */
//    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
//    while(wait_loop_index != 0)
//    {
//      wait_loop_index--;
//    }

//    /* Disable ADC DMA transfer request during calibration */
//    /* Note: Specificity of this STM32 series: Calibration factor is          */
//    /*       available in data register and also transferred by DMA.          */
//    /*       To not insert ADC calibration factor among ADC conversion data   */
//    /*       in DMA destination address, DMA transfer must be disabled during */
//    /*       calibration.                                                     */
//    backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC1);
//    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);

//    /* Run ADC self calibration */
//    LL_ADC_StartCalibration(ADC1);

//    /* Poll for ADC effectively calibrated */
//    #if (USE_TIMEOUT == 1)
//    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
//    #endif /* USE_TIMEOUT */

//    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
//    {
//    #if (USE_TIMEOUT == 1)
//      /* Check Systick counter flag to decrement the time-out value */
//      if (LL_SYSTICK_IsActiveCounterFlag())
//      {
//        if(Timeout-- == 0)
//        {
//          /* Error: Time-out */
//          Error_Handler();
//        }
//      }
//    #endif /* USE_TIMEOUT */
//    }

//    /* Restore ADC DMA transfer request after calibration */
//    LL_ADC_REG_SetDMATransfer(ADC1, backup_setting_adc_dma_transfer);

//    /* Delay between ADC end of calibration and ADC enable.                   */
//    /* Note: Variable divided by 2 to compensate partially                    */
//    /*       CPU processing cycles (depends on compilation optimization).     */
//    wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
//    while(wait_loop_index != 0)
//    {
//      wait_loop_index--;
//    }

//    /* Enable ADC */
//    LL_ADC_Enable(ADC1);

//    /* Poll for ADC ready to convert */
//    #if (USE_TIMEOUT == 1)
//    Timeout = ADC_ENABLE_TIMEOUT_MS;
//    #endif /* USE_TIMEOUT */

//    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
//    {
//    #if (USE_TIMEOUT == 1)
//      /* Check Systick counter flag to decrement the time-out value */
//      if (LL_SYSTICK_IsActiveCounterFlag())
//      {
//        if(Timeout-- == 0)
//        {
//          /* Error: Time-out */
//          Error_Handler();
//        }
//      }
//    #endif /* USE_TIMEOUT */
//    }

//    /* Note: ADC flag ADRDY is not cleared here to be able to check ADC       */
//    /*       status afterwards.                                               */
//    /*       This flag should be cleared at ADC Deactivation, before a new    */
//    /*       ADC activation, using function "LL_ADC_ClearFlag_ADRDY()".       */
//  }

//  /*## Operation on ADC hierarchical scope: ADC group regular ################*/
//  /* Note: No operation on ADC group regular performed here.                  */
//  /*       ADC group regular conversions to be performed after this function  */
//  /*       using function:                                                    */
//  /*       "LL_ADC_REG_StartConversion();"                                    */

//  /*## Operation on ADC hierarchical scope: ADC group injected ###############*/
//  /* Note: Feature not available on this STM32 series */

//}

///**
//  * @brief  Perform ADC group regular conversion start, poll for conversion
//  *         completion.
//  *         (ADC instance: ADC1).
//  * @note   This function does not perform ADC group regular conversion stop:
//  *         intended to be used with ADC in single mode, trigger SW start
//  *         (only 1 ADC conversion done at each trigger, no conversion stop
//  *         needed).
//  *         In case of continuous mode or conversion trigger set to
//  *         external trigger, ADC group regular conversion stop must be added
//  *         after polling for conversion data.
//  * @param  None
//  * @retval None
//  */
//void ConversionStartPoll_ADC_GrpRegular(void)
//{
//  #if (USE_TIMEOUT == 1)
//  uint32_t Timeout = 0U; /* Variable used for timeout management */
//  #endif /* USE_TIMEOUT */

//  /* Start ADC group regular conversion */
//  /* Note: Hardware constraint (refer to description of the function          */
//  /*       below):                                                            */
//  /*       On this STM32 series, setting of this feature is conditioned to    */
//  /*       ADC state:                                                         */
//  /*       ADC must be enabled without conversion on going on group regular,  */
//  /*       without ADC disable command on going.                              */
//  /* Note: In this example, all these checks are not necessary but are        */
//  /*       implemented anyway to show the best practice usages                */
//  /*       corresponding to reference manual procedure.                       */
//  /*       Software can be optimized by removing some of these checks, if     */
//  /*       they are not relevant considering previous settings and actions    */
//  /*       in user application.                                               */
//  if ((LL_ADC_IsEnabled(ADC1) == 1)               &&
//      (LL_ADC_IsDisableOngoing(ADC1) == 0)        &&
//      (LL_ADC_REG_IsConversionOngoing(ADC1) == 0)   )
//  {
//    LL_ADC_REG_StartConversion(ADC1);
//  }
//  else
//  {
//    /* Error: ADC conversion start could not be performed */
//    Error_Handler();
//  }

//  #if (USE_TIMEOUT == 1)
//  Timeout = ADC_UNITARY_CONVERSION_TIMEOUT_MS;
//  #endif /* USE_TIMEOUT */

//  while (LL_ADC_IsActiveFlag_EOC(ADC1) == 0)
//  {
//  #if (USE_TIMEOUT == 1)
//    /* Check Systick counter flag to decrement the time-out value */
//    if (LL_SYSTICK_IsActiveCounterFlag())
//    {
//      if(Timeout-- == 0)
//      {
//        Error_Handler();
//      }
//    }
//  #endif /* USE_TIMEOUT */
//  }

//  /* Clear flag ADC group regular end of unitary conversion */
//  /* Note: This action is not needed here, because flag ADC group regular   */
//  /*       end of unitary conversion is cleared automatically when          */
//  /*       software reads conversion data from ADC data register.           */
//  /*       Nevertheless, this action is done anyway to show how to clear    */
//  /*       this flag, needed if conversion data is not always read          */
//  /*       or if group injected end of unitary conversion is used (for      */
//  /*       devices with group injected available).                          */
//  LL_ADC_ClearFlag_EOC(ADC1);

//}


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
  //LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_SYSCLK, LL_RCC_MCO1_DIV_4);   //12Mhz = 48Mhz/4
  LL_RCC_ConfigMCO2(LL_RCC_MCO2SOURCE_SYSCLK, LL_RCC_MCO2_DIV_4);   //12Mhz = 48Mhz/4
  
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
     static uint32_t itxcount=0;     
     static uint32_t irxcount=0;     
          
     
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

     
      //uart tx
      if(itimecount>=1)   //1* 1ms =1ms
      {
         itimecount=0; 
        
        //Send_Data_To_UART1(uhADCxConvertedData);
        
         #if (defined TXMODE)    
        
            itxcount++;
        
         
            iputbytes1((uint8_t *)TESTDATA2,TXRXDATALEN);
        
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
     
     //uart rx     
     if (flg_rx==1)
     {
        flg_rx=0;
        
        irxcount++;

         //iputs1("rx in\r\n");
         //iputbytes1((uint8_t *)irxdata2,TXRXDATALEN);

         if(memcmp((void*)TESTDATA2, (void*)irxdata2, TXRXDATALEN) ==0)
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
  LL_RCC_ConfigMCO(LL_RCC_MCO2SOURCE_SYSCLK, LL_RCC_MCO2_DIV_1);
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
