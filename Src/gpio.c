/**
  ******************************************************************************
  * File Name          : gpio.c
  * Date               : 24/01/2015 00:31:17
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "gpio.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
GPIO_InitTypeDef gpio_init_struct ;	//for: CE, CSN, and IRQ pins in RFM73 module


/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();		// For SPI1 and other RFM73 pins
  __GPIOC_CLK_ENABLE();		// For SPI3 and other RFM73 pins
	__GPIOG_CLK_ENABLE(); 	// For LEDs

	
	/**  RFM73 - others pin configuration - MODULE 1
		CE
		CSN 
		IRQ
		*/
	
		// PIN: CE, CSN
		gpio_init_struct.Pin = MOD1_CE | MOD1_CSN ;
		gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP ;
		gpio_init_struct.Pull = GPIO_NOPULL ;
		gpio_init_struct.Speed = GPIO_SPEED_MEDIUM ;
		HAL_GPIO_Init(MOD1_ADF_PORT, &gpio_init_struct );
		
		// PIN IRQ - interrupt
		gpio_init_struct.Pin = MOD1_IRQ ;
		gpio_init_struct.Mode = GPIO_MODE_IT_FALLING ;
		gpio_init_struct.Pull = GPIO_PULLUP ;
		HAL_GPIO_Init(MOD1_IRQ_PORT, &gpio_init_struct );
		
		// Set NVIC for IRQ - SPI1
		HAL_NVIC_SetPriority(EXTI3_IRQn, 0x0F, 0x00);
		HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	
		
		
		/**  RFM73 - others pin configuration - MODULE 2
		CE
		CSN 
		IRQ
		*/
		// PIN CE, CSN
		gpio_init_struct.Pin = MOD2_CE | MOD2_CSN ;
		gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP ;
		gpio_init_struct.Pull = GPIO_NOPULL ;
		gpio_init_struct.Speed = GPIO_SPEED_MEDIUM ;
		HAL_GPIO_Init(MOD2_ADF_PORT, &gpio_init_struct );
		
		// PIN IRQ - interrupt
		gpio_init_struct.Pin = MOD2_IRQ ;
		gpio_init_struct.Mode = GPIO_MODE_IT_FALLING ;
		gpio_init_struct.Pull = GPIO_PULLUP ;
		HAL_GPIO_Init(MOD2_IRQ_PORT, &gpio_init_struct );
		

		// Set NVIC for IRQ - SPI3
		HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0x0E, 0x00);
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
		
		
		
		// Init green LED
		gpio_init_struct.Pin = GREEN_LED_PIN;
		gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
		gpio_init_struct.Pull = GPIO_NOPULL;
		gpio_init_struct.Speed = GPIO_SPEED_LOW ;
		HAL_GPIO_Init(GREEN_LED_PORT, &gpio_init_struct);
		
		// Init red LED
		gpio_init_struct.Pin = RED_LED_PIN;
		gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
		gpio_init_struct.Pull = GPIO_NOPULL;
		gpio_init_struct.Speed = GPIO_SPEED_LOW ;
		HAL_GPIO_Init(RED_LED_PORT, &gpio_init_struct);
		
		
		
		// User button - hardware pulled-down - active when HIGH
		gpio_init_struct.Pin = BLUE_SW_PIN;
		gpio_init_struct.Mode = GPIO_MODE_INPUT;
		gpio_init_struct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(BLUE_SW_PORT, &gpio_init_struct);
		
		
}

/* USER CODE BEGIN 2 */
		
		
	

/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
