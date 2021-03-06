/** \file gpio.h  */
/**
  ******************************************************************************
  * File Name          : gpio.h
  * Date               : 24/01/2015 00:31:18
  * Description        : This file contains all the functions prototypes for 
  *                      the gpio  
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/** MODULE 1 - RFM73 additional pins definition - compatible with STM HAL
* 
* 
* \warning Macro must be as mask not as bit number
* \warning Pins: CE and CSN must be at the same port!
*/
#define MOD1_CE GPIO_PIN_2    /**< mask for CE pin in port: MOD1_ADF_PORT */
#define MOD1_CSN GPIO_PIN_3   /**< mask for CSN pin in port: MOD1_ADF_PORT */
#define MOD1_ADF_PORT GPIOG   /**< Address of port where MOD1_CE and MOD1_CSN are connected */

#define MOD1_IRQ GPIO_PIN_3   /**< mask for IRQ pin in port: MOD1_IRQ_PORT */
#define MOD1_IRQ_PORT GPIOC   /**< Address of port where MOD1_IRQ is connected */

	 
/** MODULE 2 - RFM73 additional pins definition - compatible with STM HAL
* 
* \warning Macro must be as mask not as bit number
* \warning Pins: CE and CSN must be at the same port!
*/
#define MOD2_CE GPIO_PIN_7    /**< mask for CE pin in port: MOD2_ADF_PORT */
#define MOD2_CSN GPIO_PIN_6   /**< mask for CSN pin in port: MOD2_ADF_PORT */
#define MOD2_ADF_PORT GPIOC   /**< Address of port where MOD2_CE and MOD2_CSN are connected */
	 
#define MOD2_IRQ GPIO_PIN_8   /**< mask for IRQ pin in port: MOD2_IRQ_PORT */
#define MOD2_IRQ_PORT GPIOC   /**< Address of port where MOD2_IRQ is connected */


	 
	 
// Additional #defines

#define GPIO_SET(PORT, PIN) (((PORT)->BSRRL |= (PIN)))
#define GPIO_CLEAR(PORT, PIN) (((PORT)->BSRRH |= (PIN)))

// For LED ( fitted in discovery board ) - H level on LED pin means shining
#define GREEN_LED_PIN GPIO_PIN_13
#define GREEN_LED_PORT GPIOG
#define RED_LED_PIN GPIO_PIN_14
#define RED_LED_PORT GPIOG


// For User switch:
#define BLUE_SW_PIN GPIO_PIN_0
#define BLUE_SW_PORT GPIOA

/**  
* Performs gpio initialization
*/
void MX_GPIO_Init(void);
#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
