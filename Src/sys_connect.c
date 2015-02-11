#include "sys_connect.h"
#include "system_status.h"
#include "gpio.h"
#include "rfm73_callbacks.h"
#include "adc.h"
// SYSTEM STATUS
extern struct SysStat_TypeDef system ;



uint8_t try_connect( 
		struct Radio_TypeDef * _radioH, 
		const uint8_t * const _pipe_conn_addr, 
		const uint8_t * const _pipe_data_addr
		)
{
	
	uint8_t temp = 100 ;
	
	// A first clear old connection status flags
	system.conn_status &= ~( SYSTEM_CONNECTED_MASK | SYSTEM_CANT_CONNECT_MASK | SYSTEM_MASTER_MODE_MASK ) ;

	
	// Set proper call back functions for response from RFM73 before transmit
	_radioH->_data_ready_handler = &master_rcv_data_callback ;
	_radioH->_max_retransmission_handler = &master_cant_conn_callback ;
	_radioH->_packet_sent_handler = &master_connected_callback ;
	
	// Select connection pipe
	rfm73_set_Tpipe( _radioH, _pipe_conn_addr );
	// Switch to transmit mode 
	rfm73_mode_transmit( _radioH ) ;
	// Transmit message to next device
	rfm73_transmit_message( _radioH, (const unsigned char *)"C", 1 ) ;
	
	
/*** need to add some timeout here!? ***/
	while( !(system.conn_status & SYSTEM_CONNECTED_MASK) &&
				 !(system.conn_status & SYSTEM_CANT_CONNECT_MASK)
	) {
		rfm73_check( _radioH ) ;
	}
			
	// Last steps of connection procedure:
	if ( system.conn_status & SYSTEM_CONNECTED_MASK ) {
		// set pipe for transmission data
		rfm73_set_Tpipe( _radioH, _pipe_data_addr ) ;
		// Return status:
		temp = RET_M_CONNECTED ;
	}
	else if( system.conn_status & SYSTEM_CANT_CONNECT_MASK ) {
		
		system.conn_status &= ~(SYSTEM_CANT_CONNECT_MASK) ;
		
		// back to receive mode
		rfm73_mode_receive( _radioH ) ;
		
		// Return status:
		temp = RET_M_CANT_CONNECT ;
	}
	//timeout: else if ( ) 
		
	
	return temp;
}




// It probably can be used in callbacks - but be careful
// If called from callback pass NULL in second parameter
void master_disconnect_main( 
		struct Radio_TypeDef * _radioH, 
		const uint8_t * const _pipe_conn_addr
		) {
			// for now only: send disconnect message, and switch to receive mode
			
			// Clear connections flags in system status 
			system.conn_status &= ~( SYSTEM_CONNECTED_MASK | SYSTEM_CANT_CONNECT_MASK | SYSTEM_MASTER_MODE_MASK ) ;
			
			
			// Turn off leds 
			GPIO_CLEAR(GREEN_LED_PORT, GREEN_LED_PIN);
			//	GPIO_CLEAR(RED_LED_PORT, RED_LED_PIN);
			
			// Set appropriate callback functions for DISCONNECTED state:
			_radioH->_data_ready_handler = &disc_rcv_data_callback ;
			_radioH->_max_retransmission_handler = &disc_cant_send_callback ; 
			_radioH->_packet_sent_handler = &disc_packet_sent_callback ;
			
			// Do it if this is not in callback function
			if ( NULL != _pipe_conn_addr ) {
				// Flush TX FIFO
				rfm73_register_write( _radioH, RFM73_CMD_FLUSH_TX, 0);
				// Select connection pipe for transmitter
				rfm73_set_Tpipe( _radioH, _pipe_conn_addr );
				// Transmit message to 'slave' device
				rfm73_transmit_message( _radioH, (const unsigned char *)"D", 1 ) ;
			
				//while( _radioH->A_IRQ_gpio_port->IDR & _radioH->A_SPI_IRQ_pin ) ;
				// Get time to send packet properly
				while( HAL_GPIO_ReadPin( _radioH->A_IRQ_gpio_port, _radioH->A_SPI_IRQ_pin ) == GPIO_PIN_SET );
				
				
				// Switch to receive mode
				rfm73_mode_receive( _radioH ) ;
								
				// Flush TX FIFO 
				rfm73_register_write( _radioH, RFM73_CMD_FLUSH_TX, 0);
				
				// Clear ints in RFM73 module:
				rfm73_register_write( _radioH, RFM73_REG_STATUS, 0x70 ) ;
			}
			
			// Clear possible IRQ in modulse status - ?
			_radioH->status &= ~(RFM73_IRQ_OCR_MASK) ;
			
			adc_OFF();
			adc_buff_clear();
			// Disable all used peripherials, clear buffer and their 'iterators', etc ...
			//...
}



// Can be called from callback functions
void slave_disconnect( struct Radio_TypeDef * _radioH ) {
		
		// Turn off RED led:
		GPIO_CLEAR(RED_LED_PORT, RED_LED_PIN) ;
		
		// Clear connection flags in system status:
		system.conn_status &= ~( SYSTEM_CONNECTED_MASK | SYSTEM_CANT_CONNECT_MASK | SYSTEM_MASTER_MODE_MASK ) ;
		
		// Reset RX buffer params in module struct:
		_radioH->pipe = _radioH->buffer_maxl = _radioH->buffer_cpos = 0 ;
	
		// Set appropriate callback functions for DISCONNECTED state:
		_radioH->_data_ready_handler = &disc_rcv_data_callback ;
		_radioH->_max_retransmission_handler = &disc_cant_send_callback ; 
		_radioH->_packet_sent_handler = &disc_packet_sent_callback ;
	
		// Disable all used peripherials, clear buffer and their 'iterators', etc ...
		//...
}
