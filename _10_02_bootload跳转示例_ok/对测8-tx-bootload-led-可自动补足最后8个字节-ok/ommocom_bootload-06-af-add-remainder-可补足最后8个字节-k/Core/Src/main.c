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
#include "crc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>           
#include "string.h"

#include "stm32c0xx_hal_flash.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define LOCALSLAVEADDR  1
uint8_t islaveaddrin=0;

uint8_t slavecmd[TXDATALEN]={0}; 

#define DATA_MMC0_POINT  5
#define DATA_MMC1_POINT  12
#define DATA_ICM0_POINT  19
#define DATA_TIME_POINT  31


uint32_t idataoff =0;
uint32_t irestlen =0;

#define PAGE_DATA_SIZE  256

uint32_t icrctempdata[PAGE_DATA_SIZE/4+1]={0}; 

uint8_t FLASHTEMPDATA[PAGE_DATA_SIZE]= {0}; 
uint32_t ialldatalen=0;
uint32_t istepdatalen=0;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

  
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


//-------------dataflash-------------

#define ADDR_FLASH_PAGE_0     ((uint32_t)0x08000000) /* Base @ of Page 0, 2 Kbytes */
#define ADDR_FLASH_PAGE_1     ((uint32_t)0x08000800) /* Base @ of Page 1, 2 Kbytes */
#define ADDR_FLASH_PAGE_2     ((uint32_t)0x08001000) /* Base @ of Page 2, 2 Kbytes */
#define ADDR_FLASH_PAGE_3     ((uint32_t)0x08001800) /* Base @ of Page 3, 2 Kbytes */
#define ADDR_FLASH_PAGE_4     ((uint32_t)0x08002000) /* Base @ of Page 4, 2 Kbytes */
#define ADDR_FLASH_PAGE_5     ((uint32_t)0x08002800) /* Base @ of Page 5, 2 Kbytes */
#define ADDR_FLASH_PAGE_6     ((uint32_t)0x08003000) /* Base @ of Page 6, 2 Kbytes */
#define ADDR_FLASH_PAGE_7     ((uint32_t)0x08003800) /* Base @ of Page 7, 2 Kbytes */
#define ADDR_FLASH_PAGE_8     ((uint32_t)0x08004000) /* Base @ of Page 8, 2 Kbytes */
#define ADDR_FLASH_PAGE_9     ((uint32_t)0x08004800) /* Base @ of Page 9, 2 Kbytes */
#define ADDR_FLASH_PAGE_10    ((uint32_t)0x08005000) /* Base @ of Page 10, 2 Kbytes */
#define ADDR_FLASH_PAGE_11    ((uint32_t)0x08005800) /* Base @ of Page 11, 2 Kbytes */
#define ADDR_FLASH_PAGE_12    ((uint32_t)0x08006000) /* Base @ of Page 12, 2 Kbytes */
#define ADDR_FLASH_PAGE_13    ((uint32_t)0x08006800) /* Base @ of Page 13, 2 Kbytes */
#define ADDR_FLASH_PAGE_14    ((uint32_t)0x08007000) /* Base @ of Page 14, 2 Kbytes */
#define ADDR_FLASH_PAGE_15    ((uint32_t)0x08007800) /* Base @ of Page 15, 2 Kbytes */
#define ADDR_FLASH_PAGE_16    ((uint32_t)0x08008000)  


//#define DATA_64                 ((uint64_t)0x12345678aa345678)


#define FLASH_BASE_ADDR          ((uint32_t)0x08000000)  
#define FLASH_PAGESIZE           ((uint32_t)0x800)           //2K

#define FLASH_START_PAGE         6  
#define FLASH_START_ADDR    ( (uint32_t)(FLASH_BASE_ADDR + FLASH_START_PAGE * FLASH_PAGESIZE) )  //+3000

#define FLASH_BOOTFLAG_PAGE         14  
#define FLASH_BOOTFLAG_ADDR  ( (uint32_t)(FLASH_BASE_ADDR + FLASH_BOOTFLAG_PAGE * FLASH_PAGESIZE) )  //+7000

static FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t iflashaddr = 0;
uint32_t PageError = 0;


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
 

void dowithuart(void)
{   
   uint16_t itimnum=0;
   uint32_t ipackdatalen=0;
   uint32_t icrc=0;
   uint32_t icrclen=0;
   uint32_t icrcin=0;  
   static uint32_t icrcstep=0;
   
   
   switch (irxdata2[3])   //command
   {
      
      case 0xA1:
           
            slavecmd[0]=CMDHEADER1;    //0xAA;
            slavecmd[1]=CMDHEADER2;    //0x55;
            slavecmd[2]=0x1F;         
            slavecmd[3]=0xB1;          
            slavecmd[4]=LOCALSLAVEADDR;          
         
            itimnum = LL_TIM_GetCounter(TIM17);
            slavecmd[ DATA_TIME_POINT] = (uint8_t)(itimnum & 0xff);
            slavecmd[ DATA_TIME_POINT+1] = (uint8_t)( (itimnum>>8) & 0xff );   
      
            slavecmd[33]=get_checksum_8(slavecmd,33);
            uartsendcmd(slavecmd,34);  
           
            break;      
      
      case 0xC1:  
           
            // ------------- erease flash -----------------
      
            HAL_FLASH_Unlock();

            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

            EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
            EraseInitStruct.Page        = FLASH_START_PAGE;          //6,        //3000
            EraseInitStruct.NbPages     = 4;                         //4 x 800   //5000-

            if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
            {
               uartsendcmd( (uint8_t*)"erase flash err\r\n", 17);     
            } 
            HAL_FLASH_Lock();      
            
            // ------------- erease flash ---end--------------

            
            //response command D1
            slavecmd[0]=CMDHEADER1;  //0xAA;
            slavecmd[1]=CMDHEADER2;  //0x55;
            slavecmd[2]=0x03;        //len
            slavecmd[3]=0xD1;        //command
            slavecmd[4]=LOCALSLAVEADDR;   //local addr         
            slavecmd[5]=get_checksum_8(slavecmd,5);
            uartsendcmd(slavecmd,6);     
            
            
            iflashaddr = FLASH_START_ADDR;   //0800 3000
            idataoff =0;    
            istepdatalen=0;
            icrcstep=LL_CRC_DEFAULT_CRC_INITVALUE; 
            ialldatalen=0;       
      
            break;

      case 0xC2:  
      
            ipackdatalen=irxdata2[2]-3;
      
            if( (idataoff+ipackdatalen) > PAGE_DATA_SIZE)
            {
               uartsendcmd( (uint8_t*)"data err\r\n", 10);  
               break;               
            }
      
            //copy to array
            for( uint32_t i=0; i<ipackdatalen; i++)
            {
               FLASHTEMPDATA[idataoff+i]=irxdata2[5+i];
            }
            
            idataoff += ipackdatalen; 
            
            //------------------save to flash--------------------
            
            if( idataoff == PAGE_DATA_SIZE)
            {      
               //make step crc 
               istepdatalen=idataoff;
               icrclen=byte2uint32( (uint8_t *)FLASHTEMPDATA,icrctempdata,istepdatalen);   
               
               //icrc = stm32_crc32(icrctempdata, icrclen);  
               icrc = stm32_crc32ex(icrctempdata, icrclen, icrcstep); 
                
               HAL_FLASH_Unlock();               

               for(uint32_t i=0; i<istepdatalen; i+=8)
               {
                  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, iflashaddr, *(uint64_t*)(&FLASHTEMPDATA[i]) ) == HAL_OK)
                  {
                     iflashaddr += 8;  //move 8 bytes
                  }
                  else
                  {
                     ;//uartsendcmd( (uint8_t*)"save flash err\r\n", 16);  
                  }               
               }           

               HAL_FLASH_Lock();  

               icrcstep=icrc;
               ialldatalen+=istepdatalen;  //idataoff
               idataoff=0;
            }              
            
            //------------------save to flash-----end---------------
            
            
            //response command D2
            slavecmd[0]=CMDHEADER1;  //0xAA;
            slavecmd[1]=CMDHEADER2;  //0x55;
            slavecmd[2]=0x03;       
            slavecmd[3]=0xD2;      
            slavecmd[4]=LOCALSLAVEADDR;         
            slavecmd[5]=get_checksum_8(slavecmd,5);
            uartsendcmd(slavecmd,6);                 
            
            break;


      case 0xC3:
         
            //------------------save to flash--------------------
            
            if( idataoff < PAGE_DATA_SIZE)
            {      
               
               //if((PAGE_DATA_SIZE-idataoff)>=8)
               //   memset(FLASHTEMPDATA,0xff,8);
               //else
               //   memset(FLASHTEMPDATA,0xff,PAGE_DATA_SIZE-idataoff); 
               uint8_t iremainder=0;
               iremainder= idataoff % 8;   
               if(iremainder!=0)
                   memset(FLASHTEMPDATA+idataoff,0xff,8-iremainder);                 
               
               //make step crc 
               istepdatalen=idataoff;
               icrclen=byte2uint32( (uint8_t *)FLASHTEMPDATA,icrctempdata,istepdatalen);   
               
               //icrc = stm32_crc32(icrctempdata, icrclen);  
               icrc = stm32_crc32ex(icrctempdata, icrclen, icrcstep); 
                
               HAL_FLASH_Unlock();               

               for(uint32_t i=0; i<istepdatalen; i+=8)
               {
                  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, iflashaddr, *(uint64_t*)(&FLASHTEMPDATA[i]) ) == HAL_OK)
                  {
                     iflashaddr += 8;  //move 8 bytes
                  }
                  else
                  {
                     ;//uartsendcmd( (uint8_t*)"save flash err\r\n", 16);  
                  }               
               }           

               HAL_FLASH_Lock();  

               icrcstep=icrc;
               ialldatalen+=istepdatalen;  //idataoff
               idataoff=0;
            }                      
            //------------------save to flash-----end---------------   
      
      
            //check crc
            icrcin+=(uint32_t)(irxdata2[5]<<24);
            icrcin+=(uint32_t)(irxdata2[6]<<16);
            icrcin+=(uint32_t)(irxdata2[7]<<8);
            icrcin+=(uint32_t)(irxdata2[8]);
      
            if(icrcstep==icrcin)   
            {
               slavecmd[0]=CMDHEADER1;  //0xAA;
               slavecmd[1]=CMDHEADER2;  //0x55;
               slavecmd[2]=0x03;       
               slavecmd[3]=0xD3;      
               slavecmd[4]=LOCALSLAVEADDR;         
               slavecmd[5]=get_checksum_8(slavecmd,5);
               uartsendcmd(slavecmd,6);     
            }  
            else
            {
               slavecmd[0]=CMDHEADER1;  //0xAA;
               slavecmd[1]=CMDHEADER2;  //0x55;
               slavecmd[2]=0x03;       
               slavecmd[3]=0xE3;      
               slavecmd[4]=LOCALSLAVEADDR;         
               slavecmd[5]=get_checksum_8(slavecmd,5);
               uartsendcmd(slavecmd,6);   

               break;               
            } 

////--test
//uartsendcmd( (uint8_t*)(&ialldatalen), 4);             
            
            //-------------------read out the flash data -------------------            
            
            memset(FLASHTEMPDATA,0,PAGE_DATA_SIZE);  
            iflashaddr = FLASH_START_ADDR;        //0800 3000
            icrcstep=LL_CRC_DEFAULT_CRC_INITVALUE; 
                 
            //uint32_t ipage=0;            
            while( ialldatalen>0 )    
            {        
               if(ialldatalen >= PAGE_DATA_SIZE)
                  istepdatalen = PAGE_DATA_SIZE;
               else
                  istepdatalen = ialldatalen;  
                              
               for(uint32_t j=0; j<istepdatalen; j+=4)      
                  *(__IO uint32_t *)( &FLASHTEMPDATA[j] ) = *(__IO uint32_t *)(iflashaddr+j); 
               
               //make step crc 
               icrclen=byte2uint32( (uint8_t *)FLASHTEMPDATA,icrctempdata,istepdatalen);   
               icrc = stm32_crc32ex(icrctempdata, icrclen, icrcstep);  
               //uartsendcmd( FLASHTEMPDATA, istepdatalen);       //--test log             
               
               //ipage++;
               icrcstep=icrc;
               ialldatalen -= istepdatalen;
               iflashaddr += istepdatalen;
               istepdatalen = 0;  
            }           
                             
            if(icrc!=icrcin) 
                uartsendcmd( (uint8_t*)"crc err\r\n", 9);       

            //-------------------read out the flash data ---end----------------   
            
            
//---test log ---         
//uartsendcmd( FLASHTEMPDATA, ialldatalen);             

delay_nop(WAIT2TE_SLAVE); 
switch2send(USART1);
Send_Data_To_UART1(icrc>>24);
Send_Data_To_UART1(icrc>>16);
Send_Data_To_UART1(icrc>>8);
Send_Data_To_UART1(icrc>>0);       
delay_nop(WAIT2RE_SLAVE);  
switch2read(USART1);     
             
//uartsendcmd( (uint8_t*)(&icrc), 4); 
            
            //clear the bootflag in the flash 
            HAL_FLASH_Unlock();

            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

            EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
            EraseInitStruct.Page        = FLASH_BOOTFLAG_PAGE;   //14
            EraseInitStruct.NbPages     = 1;                     //1 x 800    

            if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
            {
               ;//iputbytes1( (uint8_t*)"erase flash err\r\n", 17);     
            } 
            
            HAL_FLASH_Lock();     

            //run the new firmware            
            LL_mDelay(10000);        //--test       
            USART1->CR1 |=(1<<2);
            USART1->CR1 |=(1<<3);
            iap_load_app(FLASH_START_ADDR);  //0x8003000

            break;

      default:         
            break;
   } 
       
}

uint32_t byte2uint32(uint8_t *strin, uint32_t *strout, uint32_t ilen)
{

   uint32_t i = 0;
   uint32_t iret = 0;

   //get crc adder
   for ( i=0; i<(ilen/4); i++)
   {
      strout[i] = (uint32_t)((strin[4*i] << 24) | (strin[4*i+1] << 16) | (strin[4*i+ 2] << 8) | strin[4*i+3]);
   }

   //last word
   if ( (ilen%4) == 3)
      strout[i] = (uint32_t)((strin[4*i] << 24) | (strin[4*i+1] << 16) | (strin[4*i+ 2] << 8) );
   if ( (ilen%4) == 2) 
      strout[i] = (uint32_t)((strin[4*i] << 24) | (strin[4*i+1] << 16) );
   if ( (ilen%4) == 1)
      strout[i] = (uint32_t)((strin[4*i] << 24) );
   
   
   //get crc len
   if( (ilen%4) ==0)
      iret =  ilen/4;
   else
      iret = ilen/4 +1;
   
   return iret;
}


uint32_t calcu_crc32(uint32_t *idatain, uint32_t len)
{
   uint32_t icrc_poly = 0x04C11DB7;  
   uint32_t icrc_out = 0xffffffff;  

   for(uint32_t i = 0; i < len; i++)
   {
      icrc_out ^= idatain[i];  

      for (uint8_t j = 0; j < 32; j++)
      {
         if (icrc_out & 0x80000000)
            icrc_out = (icrc_out << 1) ^ icrc_poly;
         else
            icrc_out <<= 1;
      }
   }
   
   return (icrc_out);
}


uint32_t calcu_crc32ex(uint32_t *idatain, uint32_t len, uint32_t icrc_in)
{
   uint32_t icrc_poly = 0x04C11DB7;  
   uint32_t icrc_out = icrc_in;  

   for(uint32_t i = 0; i < len; i++)
   {
      icrc_out ^= idatain[i];        

      for (uint8_t j = 0; j < 32; j++)
      {
         if (icrc_out & 0x80000000)
            icrc_out = (icrc_out << 1) ^ icrc_poly;
         else
            icrc_out <<= 1;
      }
   }
   
   return (icrc_out);
}

uint32_t stm32_crc32(uint32_t *idatain, uint32_t len)
{   
   LL_CRC_SetInitialData(CRC, LL_CRC_DEFAULT_CRC_INITVALUE);

   for(uint32_t i = 0; i < len; i++)
   {
      LL_CRC_FeedData32(CRC,  idatain[i]);
   }

   return (LL_CRC_ReadData32(CRC));
}

uint32_t stm32_crc32ex(uint32_t *idatain, uint32_t len, uint32_t icrc_in)
{   
   LL_CRC_SetInitialData(CRC, icrc_in);

   for(uint32_t i = 0; i < len; i++)
   {
      LL_CRC_FeedData32(CRC,  idatain[i]);
   }

   return (LL_CRC_ReadData32(CRC));
}

//---------------------bootload ---------------------------

#define STM32_FLASH_BASE 0x08000000   //bootload hex
#define FLASH_SAVE_ADDR 0x00805000    //app hex

typedef void (*iapfun)(void);
iapfun jump2app;


void MSR_MSP(uint32_t addr)
{   
   __ASM("MSR MSP, r0");  
   __ASM(" BX r14");  
   
//   __ASM("msr msp, r0");  
//   __ASM("bx lr");  
}


void iap_load_app(uint32_t appxaddr)
{ 
   if(((*(__IO uint32_t*)appxaddr)&0x2FFE0000)==0x20000000)  
   {  
      jump2app=(iapfun)*(__IO uint32_t*)(appxaddr+4);  
      //MSR_MSP(*(__IO uint32_t*)appxaddr);  
      jump2app();  
   }      
}

//---------------------bootload ----end-----------------------

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
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
    

   LL_mDelay(10);  //20
   uartsendcmd( (uint8_t*)"...",3);     //start
   

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  
   //read flash 
   __IO uint32_t idata32 = 0;  
   idata32 = *(__IO uint32_t *)(FLASH_BOOTFLAG_ADDR);       //0x8007000  
   //uartsendcmd( (uint8_t*)(&idata32), 4);  
   
  
   if(idata32!=GOBOOTLOAD)  //no bootload flag, run old
   {
      USART1->CR1 |=(1<<2);
      USART1->CR1 |=(1<<3);
      
      iap_load_app(FLASH_START_ADDR);   //0x8003000
   }
   
   while (1)
   { 
      static uint32_t iuart_rxledcount=0;
      static uint32_t isensorcount=0;
      uint8_t ichecksum=0;      

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      if(flg_timer1==1)
      {
         flg_timer1=0;
       
         iuart_rxledcount++;	
         isensorcount++;
      }             
      
      //uart rx
      if (flg_rx==1)
      {
         flg_rx=0;   
         //iputs1("rx in\r\n");

         islaveaddrin=irxdata2[4];

         //checksum
         ichecksum = get_checksum_8((uint8_t *)irxdata2,irxcount2-1);

         if(ichecksum == irxdata2[irxcount2-1])  
            if(islaveaddrin==LOCALSLAVEADDR)            
               dowithuart(); 
          
      }  
 
		if( iuart_rxledcount >=1000)
		{	
			 iuart_rxledcount=0;		 
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
