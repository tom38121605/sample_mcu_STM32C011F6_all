/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32c0xx_ll_rcc.h"
#include "stm32c0xx_ll_bus.h"
#include "stm32c0xx_ll_system.h"
#include "stm32c0xx_ll_exti.h"
#include "stm32c0xx_ll_cortex.h"
#include "stm32c0xx_ll_utils.h"
#include "stm32c0xx_ll_pwr.h"
#include "stm32c0xx_ll_dma.h"
#include "stm32c0xx_ll_spi.h"
#include "stm32c0xx_ll_tim.h"
#include "stm32c0xx_ll_usart.h"
#include "stm32c0xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
////#include "adc.h"
////#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
 
#define RXDATALEN 20 

#define CMDHEADER1 0xAA 
#define CMDHEADER2 0x55 
#define CMDEND1    0x0D 
#define CMDEND2    0x0A 



/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define TX_MODE 0
#define RX_MODE 1
#define DEFAULT_MODE RX_MODE

#define STATUS_ON 0
#define STATUS_OFF 1
#define DEFAULT_STATUS STATUS_ON
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void iputs1(char *msg);
void iputbytes1(uint8_t *msg, uint32_t ilen);
void Send_Data_To_UART1 (uint8_t c);
uint8_t get_checksum_8(uint8_t *ival, uint32_t inum);
void dowith_uart_rx(void);

void delay_count(uint32_t icount);
uint8_t SPI_transfer(uint8_t data);

//mmc0
uint8_t mmc0_read_byte(uint8_t addr);
void mmc0_write_byte(uint8_t addr,  uint8_t value);
void mmc0_read_bytes(uint8_t addr ,uint8_t length ,uint8_t *buffer);
void mmc0_readfromspi( uint8_t iaddr, uint8_t ilen, uint8_t *idata);
void mmc0_init(void);
void mmc0_init2(void);
void mmc0_read_data(uint8_t *sdata, uint8_t ilen);
void mmc0_read_data2(uint8_t *sdata, uint8_t ilen);

//mmc1
uint8_t mmc1_read_byte(uint8_t addr);
void mmc1_write_byte(uint8_t addr,  uint8_t value);
void mmc1_read_bytes(uint8_t addr ,uint8_t length ,uint8_t *buffer);
void mmc1_readfromspi( uint8_t iaddr, uint8_t ilen, uint8_t *idata);
void mmc1_init(void);
void mmc1_init2(void);
void mmc1_read_data(uint8_t *sdata, uint8_t ilen);
void mmc1_read_data2(uint8_t *sdata, uint8_t ilen);

//icm0
uint8_t icm0_read_byte(uint8_t addr);
void icm0_write_byte(uint8_t addr,  uint8_t value);
void icm0_read_bytes(uint8_t addr ,uint8_t length ,uint8_t *buffer);
void icm0_readfromspi( uint8_t iaddr, uint8_t ilen, uint8_t *idata);
void icm0_init(void);    
void icm0_read_data(uint8_t *sdata, uint8_t ilen); 

   
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define nRST_Pin LL_GPIO_PIN_2
#define nRST_GPIO_Port GPIOF
#define IMU_READY_0_Pin LL_GPIO_PIN_0
#define IMU_READY_0_GPIO_Port GPIOA
#define IMU_READY_0_EXTI_IRQn EXTI0_1_IRQn
#define IMU_READY_1_Pin LL_GPIO_PIN_1
#define IMU_READY_1_GPIO_Port GPIOA
#define IMU_READY_1_EXTI_IRQn EXTI0_1_IRQn
#define SPI1_CS2_Pin LL_GPIO_PIN_2
#define SPI1_CS2_GPIO_Port GPIOA
#define SPI1_CS0_Pin LL_GPIO_PIN_3
#define SPI1_CS0_GPIO_Port GPIOA
#define SPI1_CS1_Pin LL_GPIO_PIN_4
#define SPI1_CS1_GPIO_Port GPIOA
#define LED_Pin LL_GPIO_PIN_6
#define LED_GPIO_Port GPIOB
#define IMU_READY_2_Pin LL_GPIO_PIN_7
#define IMU_READY_2_GPIO_Port GPIOB
#define IMU_READY_2_EXTI_IRQn EXTI4_15_IRQn
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
