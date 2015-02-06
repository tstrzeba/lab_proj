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
#include "radio_lib.h"
/* USER CODE BEGIN 0 */


// external - global - variables

// structures for radio RFM73 modules:
extern struct Radio_TypeDef radio1;
extern struct Radio_TypeDef radio2;


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
void EXTI3_IRQHandler( void ) {
	
	// Check if that was interrupt from MOD1_IRQ pin
	if(__HAL_GPIO_EXTI_GET_IT(MOD1_IRQ) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(MOD1_IRQ);		// Clear INTERRUPT
		
		// Inform main program that RFM73 module 1 has something to say: 
		radio1.status |= RFM73_IRQ_OCR_MASK ;
  }

}



/**
*	@brief For RFM73 IRQ - MODULE 2 ( connected with SPI3 )
*/
void EXTI9_5_IRQHandler( void ) {
	
	// Check if that was interrupt from MOD1_IRQ pin
	if(__HAL_GPIO_EXTI_GET_IT(MOD2_IRQ) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(MOD2_IRQ);		// Clear INTERRUPT
		
		
		// Inform main program that RFM73 module 2 has something to say: 
		radio2.status |= RFM73_IRQ_OCR_MASK ;
		
  }
	
}







/** 
* @brief For RFM73 IRQ - MODULE 1 SPI - interrupt
*/
void SPI1_IRQHandler(void) {
	
	
	
}


/** 
* @brief For RFM73 IRQ - MODULE 2 SPI - interrupt
*/
void SPI3_IRQHandler(void) {
	
	if ( (radio2.status & RFM73_D_READING_MASK) && 
			 !(radio2.status & RFM73_D_READY_MASK)
	) {
		
		// Copy just read data to buffer
		radio2.buffer[ radio2.buffer_cpos ] = radio2.spi_inst->Instance->DR ;
		
		radio2.buffer_cpos++ ;
		
		// All bytes was read? 
		if( radio2.buffer_cpos >= radio2.buffer_maxl ) {
			// Inform main that whole data was read
			radio2.status |= RFM73_D_READY_MASK ;
			
			RFM73_CSN( &radio2, 1 ); 				// End of transmition
			
			// Mask source of this interrpt ( disable )
			radio2.spi_inst->Instance->CR2 &= ~(SPI_CR2_RXNEIE) ;
		}
		else {
			// still some bytes are waitng in RFM73 RX FIFO
			
			// send infomation to RFM73 to send next byte
			// wait for empty TX buffer
			while( !(radio2.spi_inst->Instance->SR & SPI_SR_TXE) ) ;			
			radio2.spi_inst->Instance->DR = 0;				// received data will triger that interrupt
			
		}
	} 
	// else option is hard fault?  Could it happen?

}



/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
