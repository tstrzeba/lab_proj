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
#include "rfm73_callbacks.h"
#include "system_status.h"
#include "adc.h"
#include "sys_connect.h"
#include "dac.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// Declare SYSTEM structure - contain system state information ( e.g. connected / not connected )
struct SysStat_TypeDef system ;
	
	
// Declare Radio_TypeDef structure for each RFM73 module
struct Radio_TypeDef radio1;
struct Radio_TypeDef radio2;


// SPI hal instances from spi.c
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern ADC_HandleTypeDef hadc3;
extern DAC_HandleTypeDef hdac;

// RFM73 pipes address
extern const unsigned char RX0_Address[] ;
extern const unsigned char RX1_Address[] ;
extern const unsigned char RX2_Address[] ;


TIM_HandleTypeDef htim2;
static void MX_TIM2_Init(void);


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
	// Main function variables: 
	uint8_t temp8 ;
	
	// for testing:
	uint8_t pipe, length, r2buff[20];
	uint32_t coreclock ;

	// Define SYSTEM STATUS
	system.conn_status = 0 ;
	
	// Define Radio_TypeDef structs for each RFM73 module
	
	radio1.spi_inst = &hspi1 ;
	radio1.spi_irqn = SPI1_IRQn ;
	radio1.A_SPI_CE_pin = MOD1_CE ;
	radio1.A_SPI_CSN_pin = MOD1_CSN ;
	radio1.A_SPI_gpio_port = MOD1_ADF_PORT ;
	radio1.A_IRQ_gpio_port = MOD1_IRQ_PORT ;
	radio1.A_SPI_IRQ_pin = MOD1_IRQ ;
	radio1.tx_buffer = NULL ;
	radio1.tx_buff_size = 0 ;
	radio1.buff_stat = 0 ;
	radio1.status = 0 ;
	radio1._data_ready_handler = &disc_rcv_data_callback ;
	radio1._max_retransmission_handler = &disc_cant_send_callback ;
	radio1._packet_sent_handler = &disc_packet_sent_callback ;
	/*
	_radioH->_data_ready_handler = &data_ready_callback ;
	_radioH->_max_retransmission_handler = &cant_send_callback ;
	_radioH->_packet_sent_handler = &packet_sent_callback ;
	*/
	
	
	radio2.spi_inst = &hspi3 ;
	radio2.spi_irqn = SPI3_IRQn ;
	radio2.A_SPI_CE_pin = MOD2_CE ;
	radio2.A_SPI_CSN_pin = MOD2_CSN ;
	radio2.A_SPI_gpio_port = MOD2_ADF_PORT ;
	radio2.A_IRQ_gpio_port = MOD2_IRQ_PORT ;
	radio2.A_SPI_IRQ_pin = MOD2_IRQ ;
	radio2.tx_buffer = NULL ;
	radio2.tx_buff_size = 0 ;
	radio2.buff_stat = 0 ;
	radio2.status = 0 ;
	radio2._data_ready_handler = &disc_rcv_data_callback ;
	radio2._max_retransmission_handler = &disc_cant_send_callback ;
	radio2._packet_sent_handler = &disc_packet_sent_callback ;
	
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
  MX_ADC3_Init();
	MX_DAC_Init();
	MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
	// Init radio RFM73 module
	rfm73_init( &radio1 );
	rfm73_init( &radio2 );
  
	
	/* USER CODE END 2 */

	
	// Check that radio RFM73 module is connected
	/*
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
	*/


		//rfm73_mode_standby( &radio2 );
	
	/*
		r2buff[0]='\n'; r2buff[1]='T' ;r2buff[2]='E'; r2buff[3]='S'; r2buff[4]='T'; r2buff[5]=' ';
		
		rfm73_set_Tpipe( &radio1, RX2_Address );
		rfm73_mode_transmit( &radio1 ) ;
	
		rfm73_transmit_message( &radio1, r2buff, 6 ) ;
		rfm73_transmit_message( &radio1, r2buff, 6 ) ;
		rfm73_transmit_message( &radio1, r2buff, 6 ) ;
	*/
		/*
		rfm73_transmit_address( &radio1, RX2_AddressT ) ;
		rfm73_receive_address_p0( &radio1, RX2_AddressT ) ;
		
		rfm73_mode_transmit( &radio1 ) ;
		*/
	//	rfm73_mode_receive( &radio1 );
		
	//	rfm73_init_sendingNB( &radio1, r2buff, 6, RX2_AddressT ) ;
		
	
	/*
	SystemCoreClockUpdate();
	coreclock = SystemCoreClock;

	*/
	
	/*
		GPIO_SET(GREEN_LED_PORT, GREEN_LED_PIN);
		rfm73_wait_ms(1000);
		GPIO_CLEAR(GREEN_LED_PORT, GREEN_LED_PIN);
		rfm73_wait_ms(1000);
		*/
	

  /* Infinite loop */
  while (1)
  {
		adc_data_ready();
		dac_data_ready(); 
		
		
		// Check status rfm73 module
		rfm73_check( &radio1 ) ;
		
		
		// Check status rfm73 module
		rfm73_check( &radio2 ) ;
			
			
			
		// Perform connection procedure - master mode:
		if ( (HAL_GPIO_ReadPin( BLUE_SW_PORT, BLUE_SW_PIN ) == GPIO_PIN_SET) &&
				!(system.conn_status & SYSTEM_CONNECTED_MASK)
			 ) {
			
				 // Is need to disable interrupts?
				if( try_connect(&radio1, RX1_Address, RX2_Address) == RET_M_CONNECTED )
					coreclock = HAL_GetTick() ;	// for sending testing message
					
				
		} 
		// Perform disconnection procedure only in master mode:
		else if ( (HAL_GPIO_ReadPin( BLUE_SW_PORT, BLUE_SW_PIN ) == GPIO_PIN_RESET) &&
							(system.conn_status & SYSTEM_CONNECTED_MASK) && 
							(system.conn_status & SYSTEM_MASTER_MODE_MASK )
						) { 
			// Is need to disable interrupts?
			master_disconnect_main( &radio1, RX1_Address ) ;
		}


		// Timeout procedure for SLAVE when connected but it not receiving data anymore
		if( (system.conn_status & SYSTEM_CONNECTED_MASK) && 
				!(system.conn_status & SYSTEM_MASTER_MODE_MASK) && 
			( (HAL_GetTick() - system.s_timelast) >= SYSTEM_SLAVE_TIMEOUT_VAL )
		  ) {

			slave_disconnect(&radio2) ;
		}
		

		
		
		/** Send one message after 2s since estabilished connection **/
		if ( ((uint32_t)( HAL_GetTick() - coreclock ) >= 2000 ) &&
					(system.conn_status & SYSTEM_CONNECTED_MASK)
			 ) {
				 
				 coreclock = HAL_GetTick() ; // reset timer
				// rfm73_transmit_message( &radio1, (const uint8_t *)"TEST\n", 5 );
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);

}

void MX_TIM2_Init(void)
{
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1428 ;		// Give as 95,2 us bettwen interrupts
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);
	TIM2->DIER |= TIM_DIER_UIE;
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
  /* User can add his own implementation to report the file name and line number,*/
  
  /* USER CODE END 6 */

}

#endif

/*
		// Connected in master (transmitter) mode 
		if ( (system.conn_status & SYSTEM_CONNECTED_MASK) && 
				 (system.conn_status & SYSTEM_MASTER_MODE_MASK)
			 ) {}
		// Connected in slave (receiver) mode 
		else if ( (system.conn_status & SYSTEM_CONNECTED_MASK) && 
							!(system.conn_status & SYSTEM_MASTER_MODE_MASK)
						){ }
		// Not connected - idle
		else 
		{		}
		*/


/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
