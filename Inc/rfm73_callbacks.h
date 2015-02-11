#ifndef _RFM73_CALLBACKS_
#define _RFM73_CALLBACKS_

#include <stdint.h>
#include "radio_lib.h"


	/*	Callback functions for DISCONNECTED state */
void disc_rcv_data_callback( struct Radio_TypeDef * _radioH ) ;
void disc_packet_sent_callback( struct Radio_TypeDef * _radioH ) ;
uint8_t disc_cant_send_callback( struct Radio_TypeDef * _radioH ) ;



/**	\breif	Callback functions for CONNECTED state - SLAVE (receiver) mode
* 	 
*		 Place in Sconn_rcv_data_callback() not too long functions to handle received data
*		 Otherwise you should set proper flag and handle your function
*		 in main. This function shouldn't be to long because the incoming
*		 data could be missed.
*/
void Sconn_rcv_data_callback( struct Radio_TypeDef * _radioH ) ;
void Sconn_packet_sent_callback( struct Radio_TypeDef * _radioH ) ;
uint8_t Sconn_cant_send_callback( struct Radio_TypeDef * _radioH ) ;



	/*	Callback functions for CONNECTED state - MASTER (receiver) mode */
void Mconn_rcv_data_callback( struct Radio_TypeDef * _radioH ) ;
void Mconn_packet_sent_callback( struct Radio_TypeDef * _radioH ) ;
uint8_t Mconn_cant_send_callback( struct Radio_TypeDef * _radioH ) ;



	/*	Callback functions for connection procedure */
void master_rcv_data_callback( struct Radio_TypeDef * ) ;
uint8_t master_cant_conn_callback( struct Radio_TypeDef * ) ;
void master_connected_callback( struct Radio_TypeDef * ) ;	



	/* 		!!!!! NOT USED NOW !!!!!
				Callback functions for main program 
	*/
void data_ready_callback( struct Radio_TypeDef * _radioH ) ;
uint8_t cant_send_callback ( struct Radio_TypeDef * _radioH ) ;
void packet_sent_callback( struct Radio_TypeDef *) ;


#endif
