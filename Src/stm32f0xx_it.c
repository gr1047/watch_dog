/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"
#include "main.h"


/* USER CODE BEGIN 0 */
#include "main.h"
#include "crc.h"
#include "usbd_cdc_if.h"
#include "stm32f0xx_hal.h"

extern uint32_t TMBUF;
extern uint8_t iRX, iTX;
extern uint8_t BUF_TX[32];
extern uint8_t BUF_RX[32];

extern uint16_t TO;
extern uint32_t TO_TICK;

extern uint16_t TO_RES_LEN;

void analiz(void);
void writeFlash( uint32_t ADDR, uint32_t F);


uint16_t crc_;
uint16_t crcG ;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;

/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  // Отработка таймера работы с буфером
	if(TMBUF > 0) TMBUF --;
	if(TO_TICK > 0) TO_TICK --;
	if(TO_RES_LEN >0) TO_RES_LEN --;
	
	if(TO_RES_LEN > 0 ) 
		 HAL_GPIO_WritePin(ResetPC_GPIO_Port, ResetPC_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(ResetPC_GPIO_Port, ResetPC_Pin, GPIO_PIN_RESET);
	
	if(TMBUF == 1) 
	{
		analiz();
  	iRX = 0; // Нулим приёмный буфер
	}	
	
  if(TO_TICK==1) // Считает не больше чем до переполнения.
	{
		writeFlash( FLASHADDR, 0 ); // Заполняем факт сработки ватчдога 0 - есть сработка. Не 0 (FF) нет.
		 TO_RES_LEN = RESET_LEN; // Ресетим таймер длительности сброса
	}	
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles USB global Interrupt / USB wake-up interrupt through EXTI line 18.
*/
void USB_IRQHandler(void)
{
  /* USER CODE BEGIN USB_IRQn 0 */

  /* USER CODE END USB_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_IRQn 1 */

  /* USER CODE END USB_IRQn 1 */
}

/* USER CODE BEGIN 1 */
	void analiz( void )
{
	// Frame receive
  // analitics
	
	Crc16_ini();
//	uint16_t *ccrc;
	
	
	crc_ = (BUF_RX[BUF_RX[1]+3]<< 8 | BUF_RX[BUF_RX[1]+2]);
	crcG = Crc16((uint8_t *)BUF_RX, BUF_RX[1]+2);
	if (crcG!= crc_)
	{

		BUF_TX[0]=0xff;
		 CDC_Transmit_FS((uint8_t *)BUF_TX, 1);

		return;
	}	
	
//****************************	
	switch (BUF_RX[0])
	{
    case 0x01: // Я жив
		{	
		 
		 Crc16_ini();
     BUF_TX[0]=0x01;
	   BUF_TX[1]=0x01;
		 TO = BUF_RX[2];
		 TO_TICK = TO*MINTOSYS;	// Запускаем таймер WatchDog
		 BUF_TX[2]=TO;

			crcG = Crc16((uint8_t *)BUF_TX, 3);
			BUF_TX[3] = crcG & 0xff; BUF_TX[4] = crcG >> 8 & 0xff;	

			CDC_Transmit_FS((uint8_t *)BUF_TX, 5);

			// После запроса "я жив", сбрасывается признак ранее сброса
     if(*(uint32_t *)(FLASHADDR)==0)
			                writeFlash(FLASHADDR, 0xff );
		}
		break;

		case 0x02: // Стоп
		{	
		 TO_TICK = 0;	//Останавливаем таймер WatchDog
		 Crc16_ini();
     BUF_TX[0]=0x02;
	   BUF_TX[1]=0x01;
		 BUF_TX[2]= 0;


			
		  crcG = Crc16((uint8_t *)BUF_TX, 3);
			BUF_TX[3] = crcG & 0xff; BUF_TX[4] = crcG >> 8 & 0xff;	
			
		 CDC_Transmit_FS((uint8_t *)BUF_TX, 5);
		}
			break;
		case 0x03: // Рестарт. Тут требуется подать сброс на PC. 
		{
		 Crc16_ini();
			
		 TO_TICK = 2; // Даём 2 такта на сброс. 	// Запускаем таймер WatchDog
		 BUF_TX[0]=0x03;
	   BUF_TX[1]=0x01;
		 BUF_TX[2]=TO;
		 
		 crcG = Crc16((uint8_t *)BUF_TX, 3);
     BUF_TX[3] = crcG & 0xff; BUF_TX[4] = crcG >> 8 & 0xff;	

			CDC_Transmit_FS((unsigned char*)BUF_TX, 5);
		}
			break;
    // Версия
		case 0x04:
		{
		 Crc16_ini();
     BUF_TX[0]=0x04;
	   BUF_TX[1]=0x01;
		 BUF_TX[2]=VER;

  	 crcG = Crc16((uint8_t *)BUF_TX, 3);
     BUF_TX[3] = crcG & 0xff; BUF_TX[4] = crcG >> 8 & 0xff;	
		
			CDC_Transmit_FS((uint8_t *)BUF_TX, 5);
		}
		break;
		
		//Статус
		case 0x05: // Статус 
		{
		 Crc16_ini();
     BUF_TX[0]=0x05;
	   BUF_TX[1]=0x01;
		 BUF_TX[2]=(*(uint32_t *)(FLASHADDR)) & 0xff;

		 crcG = Crc16((uint8_t *)BUF_TX, 3);
     BUF_TX[3] = crcG & 0xff; BUF_TX[4] = crcG >> 8 & 0xff;	

			CDC_Transmit_FS((uint8_t *)BUF_TX, 5);
		}
		break;
		
		
		default:
		{
			BUF_TX[0]=0xff;
			CDC_Transmit_FS((uint8_t *)BUF_TX, 1);
		}
		break;
	}
}

void writeFlash( uint32_t ADDR, uint32_t F)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PAGEError = 0;
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = ADDR;
    EraseInitStruct.NbPages     = 1;
    HAL_FLASH_Unlock();   // Разблокируем флеш память
    HAL_FLASHEx_Erase(&EraseInitStruct,&PAGEError);   
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, ADDR, F );   // Записываем значение переменной isTimeWorkL на 63 странице флеш памяти
    HAL_FLASH_Lock();   // Блокируем флеш память
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
