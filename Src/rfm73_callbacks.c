#include "rfm73_callbacks.h"
#include "gpio.h"
#include "system_status.h"
#include "sys_connect.h"
#include "adc.h"
#include "dac.h"

// For FIR purposes
#include "arm_math.h"
//#include "math_helper.h"


// SYSTEM STATUS
extern struct SysStat_TypeDef system ;

// FIR structure
extern arm_fir_instance_f32 S_FIR;

/**
* 			Callback functions in DISCONNECTED state
*/

void disc_rcv_data_callback( struct Radio_TypeDef * _radioH ) {
			
		// Check for connect message:
		if ( (_radioH->buffer[0] == 'C') && (_radioH->buffer_maxl == 1) && (_radioH->pipe == 1) ) {
			
			// Set the connection slave mode flag:
			system.conn_status |= SYSTEM_CONNECTED_MASK ;
			
			// Turn on red led
			GPIO_SET(RED_LED_PORT, RED_LED_PIN) ;
			
			// Init DAC - buffers, etc.
			dac_buff_init() ;
					
						// Init FIR 
						fir_init();
			
			
			// Reset timeout counter:
			system.s_timelast = HAL_GetTick() ;


			// Set callback functions for module in that state - connected SLAVE mode ( receiver )
			_radioH->_data_ready_handler = &Sconn_rcv_data_callback ;
			_radioH->_max_retransmission_handler = &Sconn_cant_send_callback ; 
			_radioH->_packet_sent_handler = &Sconn_packet_sent_callback ;
		}
			
}

void disc_packet_sent_callback( struct Radio_TypeDef * _radioH ) {}

uint8_t disc_cant_send_callback( struct Radio_TypeDef * _radioH ) {
	// return non 0 value to flush TX FIFO in that case
	return 1;
}






/**
* 			Callback functions in CONNECTED state, SLAVE ( receiver ) mode
*/

void Sconn_rcv_data_callback( struct Radio_TypeDef * _radioH ) {
	
		// Test for DISCONNECT message - it must appear on pipe 1
		if ( _radioH->pipe == 1 ) {
			// Test for DISCONNECT message:
			if ( (_radioH->buffer_maxl == 1) && ( _radioH->buffer[0] == 'D' ) ) {
				slave_disconnect( _radioH ) ;
			}
		} 
		// Valid data
		else {

			// Reset timeout counter:
			system.s_timelast = HAL_GetTick();

			/* *** Place here not too long functions to handle received data
						 Otherwise you should set proper flag and handle your function
						 in main. This function shouldn't be to long because the incoming
						 data could be missed.
			*** */
			
			// Convert received data to DAC buffer
			// second parameter shoud be:  _radioH->buffer_maxl but dac_append() only works for 30Bs
			
			dac_buff_append( (uint8_t*)_radioH->buffer, 30);


		}
}
	
void Sconn_packet_sent_callback( struct Radio_TypeDef * _radioH ) {}

uint8_t Sconn_cant_send_callback( struct Radio_TypeDef * _radioH ) {
	// return non 0 value to flush TX FIFO in that case
	return 1;
}






/**
* 			Callback functions in CONNECTED state - MASTER mode
*/


void Mconn_rcv_data_callback( struct Radio_TypeDef * _radioH ) {}
	
void Mconn_packet_sent_callback( struct Radio_TypeDef * _radioH ) {
	// Clear counter of not confirmed packets
	system.m_missing_cnt = 0;
}

uint8_t Mconn_cant_send_callback( struct Radio_TypeDef * _radioH ) {
	
	#ifdef __DBG_ITM
			ITM_SendChar( '-' ) ;
			ITM_SendChar( 'M' ) ;
			ITM_SendChar( 'D' ) ;
			ITM_SendChar( 'C' ) ;
			ITM_SendChar( 'A' ) ;
			ITM_SendChar( 'L' ) ;
			ITM_SendChar( '-' ) ;
	#endif
	
	
	// The receiver don't respond properly - perform disconnecting steps withou sending disconnect message
	
	// Count not confirmed packets
	system.m_missing_cnt++ ;
	if( system.m_missing_cnt >= SYSTEM_NOT_CONFIRMED_MAX ) {
		system.m_missing_cnt = 0 ;
		master_disconnect_main( _radioH, NULL ) ; 	// Without sending disconnect message
	}
	
	
	// return non 0 value to flush TX FIFO in that case
	return 1;
}






/** 
* 			Callbacks functions for (MASTER) connection procedure
*/



void master_rcv_data_callback( struct Radio_TypeDef * _radioH ) {}
	
uint8_t master_cant_conn_callback( struct Radio_TypeDef * _radioH ) {
	
	// clear connection status
	system.conn_status &= ~(SYSTEM_CONNECTED_MASK) ;
	// inform that connection process was stopped
	system.conn_status |= SYSTEM_CANT_CONNECT_MASK ;
	
	// turn off leds
	GPIO_CLEAR(GREEN_LED_PORT, GREEN_LED_PIN);
	GPIO_CLEAR(RED_LED_PORT, RED_LED_PIN);
	
	// Set appriopriate callback functions for DISCONNECTED state
	_radioH->_data_ready_handler = &disc_rcv_data_callback ;
	_radioH->_max_retransmission_handler = &disc_cant_send_callback ;
	_radioH->_packet_sent_handler = &disc_packet_sent_callback ;
	
	
	return 1; 	// it flushes TX FIFO - no retransmission will be performed
}



void master_connected_callback( struct Radio_TypeDef * _radioH ) {
	
	// Set appriopriate callback functions for CNNECTED state - MASTER mode
	_radioH->_data_ready_handler = &Mconn_rcv_data_callback ;
	_radioH->_max_retransmission_handler = &Mconn_cant_send_callback ;
	_radioH->_packet_sent_handler = &Mconn_packet_sent_callback ;
	
	// Clear counter of not confirmed packets
	system.m_missing_cnt = 0;

	// Turn on green led
	GPIO_SET(GREEN_LED_PORT, GREEN_LED_PIN);
	
	// set connected status
	system.conn_status |= SYSTEM_CONNECTED_MASK ;
	// set connection status for master mode
	system.conn_status |= SYSTEM_MASTER_MODE_MASK ;
	adc_ON();
	
}	








/**			!!!!!!!! NOT USED NOW !!!!!!!!
* 			Callbacks function for main program - the first one
*/

/**
* 	That function must return !=0 to FLUSH TX FIFO!
*/
void data_ready_callback( struct Radio_TypeDef * _radioH ) {
	
	#ifdef __DBG_ITM
	ITM_SendChar( '\n' );
	
	uint8_t i ;
	for ( i = 0 ; i < _radioH->buffer_maxl ; i++ ) 
		ITM_SendChar( _radioH->buffer[i] ) ;
	
	ITM_SendChar( '\n' );
	#endif

}

uint8_t cant_send_callback ( struct Radio_TypeDef * _radioH ) {
	
	ITM_SendChar( '-' ) ;
	ITM_SendChar( 'R' ) ;
	ITM_SendChar( '-' ) ;
	
	return 1 ;
}

void packet_sent_callback( struct Radio_TypeDef * _radioH ) {
	
	ITM_SendChar( '-' ) ;
	ITM_SendChar( 'S' ) ;
	ITM_SendChar( '-' ) ;
	
}
