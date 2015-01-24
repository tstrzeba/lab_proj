/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @date    24/01/2015 00:31:18
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "gpio.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

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

  /* USER CODE END SysTick_IRQn 1 */
}

/**
* @brief This function handles RCC global interrupt.
*/
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/** 
* @brief For RFM73 IRQ - MODULE 1 ( connected with SPI1 )
*
*/
void EXTI2_IRQHandler( void ) {
	
	// Check if that was interrupt from MOD1_IRQ pin
	if(__HAL_GPIO_EXTI_GET_IT(MOD1_IRQ) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(MOD1_IRQ);		// Clear INTERRUPT
		
		// TO DO STH. 
		// ************
  }

}



/**
*	@brief For RFM73 IRQ - MODULE 2 ( connected with SPI3 )
*/
void EXTI15_10_IRQHandler( void ) {
	
	// Check if that was interrupt from MOD1_IRQ pin
	if(__HAL_GPIO_EXTI_GET_IT(MOD2_IRQ) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(MOD2_IRQ);		// Clear INTERRUPT
		
		// TO DO STH. 
		// ************
  }
	
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
