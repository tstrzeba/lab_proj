/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 24/01/2015 00:31:19
  * Description        : Main program body
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
#include "spi.h"
#include "gpio.h"
#include "radio_lib.h"

#ifdef __DBG_ITM
#include "stdio.h"
#endif
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

#ifdef __DBG_ITM
	// For debuging option
	int fputc(int c, FILE *f) {
		return(	ITM_SendChar(c));
	}
#endif

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// Declare Radio_TypeDef structure for each RFM73 module
struct Radio_TypeDef radio1;
struct Radio_TypeDef radio2;


// SPI hal instances from spi.c
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;



/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
	// Main function variables: 
	uint8_t temp8 ;
	
	// for testing:
	uint8_t pipe, length, r2buff[20];
	uint32_t coreclock ;
	
	// Define Radio_TypeDef structs for each RFM73 module
	
	radio1.spi_inst = &hspi1 ;
	radio1.spi_irqn = SPI1_IRQn ;
	radio1.A_SPI_CE_pin = MOD1_CE ;
	radio1.A_SPI_CSN_pin = MOD1_CSN ;
	radio1.A_SPI_gpio_port = MOD1_ADF_PORT ;
	radio1.buff_stat = 0 ;
	radio1.status = 0 ;
	
	radio2.spi_inst = &hspi3 ;
	radio2.spi_irqn = SPI3_IRQn ;
	radio2.A_SPI_CE_pin = MOD2_CE ;
	radio2.A_SPI_CSN_pin = MOD2_CSN ;
	radio2.A_SPI_gpio_port = MOD2_ADF_PORT ;
	radio2.buff_stat = 0 ;
	radio2.status = 0 ;
	
	// End Define Radio_TypeDef struct for each RFM73 module
	
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();

  /* USER CODE BEGIN 2 */
	
	// Init radio RFM73 module
	rfm73_init( &radio1 );
	rfm73_init( &radio2 );
  
	
	/* USER CODE END 2 */

	
	// Check that radio RFM73 module is connected

	if ( rfm73_is_present(&radio1) ) {
		// turn on led
		GPIO_SET(GREEN_LED_PORT, GREEN_LED_PIN);
	} else {
		// turn off led
		GPIO_CLEAR(GREEN_LED_PORT, GREEN_LED_PIN);
	}

	// Check that radio RFM73 module is connected
	if ( rfm73_is_present(&radio2) ) {
		// turn on led
		GPIO_SET(RED_LED_PORT, RED_LED_PIN);
	} else {
		// turn off led
		GPIO_CLEAR(RED_LED_PORT, RED_LED_PIN);
	}
		
	
	

	rfm73_mode_transmit(&radio1);
	rfm73_transmit_message(&radio1, (const unsigned char *)"aabbccyyiiooaakklld", 19);
	rfm73_mode_receive(&radio1);
	
	
	// Test module2 if are some data to read:
	/*
	temp8 = 0;
	temp8 = rfm73_register_read(&radio2, RFM73_REG_STATUS);
	if ( temp8 & (1<<6) )
		rfm73_receive( &radio2, &pipe, r2buff, &length );
	
	// Clear status
	rfm73_register_write( &radio2, RFM73_REG_STATUS, temp8 ); //clear interrupts in RFM73
	*/
	
	/*
	SystemCoreClockUpdate();
	coreclock = SystemCoreClock;
	printf("CoreClock: %i", coreclock);
	*/
	
	
  /* Infinite loop */
  while (1)
  {
		/*
		GPIO_SET(GREEN_LED_PORT, GREEN_LED_PIN);
		rfm73_wait_ms(1000);
		GPIO_CLEAR(GREEN_LED_PORT, GREEN_LED_PIN);
		rfm73_wait_ms(1000);
		*/
		
		
		// if IRQ interrupt was received
		/// *******// Probably good idea is to test IRQpin if it has low level ( becouse probably rfm73 can keep low level
		// when have even one interrupt flag set ( in his status register )
		if ( (radio2.status & RFM73_IRQ_OCR_MASK) ) {
			
			// Perform test his status register
			rfm73_analyze( &radio2 );
			
		}
		
		
		// if data from RFM73 module was read properly
		if ( radio2.status & RFM73_D_READY_MASK ) {
			
		#ifdef __DBG_ITM
			for ( temp8 = 0 ; temp8 <= radio2.buffer_maxl ; temp8++ ) 
				ITM_SendChar( radio2.buffer[temp8] ) ;
		#endif
			
			// Clear library flags - ready for another data
			radio2.status &= ~( RFM73_D_READY_MASK | RFM73_D_READING_MASK );
			
			// Clear int. flags in module: 
			rfm73_register_write( &radio2, RFM73_REG_STATUS, 0x70 );  //clear ints
			
		}
		
  }


}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
