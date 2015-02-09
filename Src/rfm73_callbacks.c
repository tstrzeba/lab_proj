#include "rfm73_callbacks.h"


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
