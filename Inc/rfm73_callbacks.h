#ifndef _RFM73_CALLBACKS_
#define _RFM73_CALLBACKS_

#include <stdint.h>
#include "radio_lib.h"


/** \file rfm73_callbacks.h rfm73_callbacks.c
*	
*	These files contains declarations and definitions callback functions
*	used by RFM73 library. 
*
*	\sa Radio_TypeDef
*/


/**	\brief Callback function for DISCONNECTED state 
*
* 	Function called when data was received from RFM73 module
*	\n \n
*	Tests data from packet if they are 'connect' data
*	\li if yes, sets proper flags in SysStat_TypeDef::conn_status and turns on red led.
*	After that device goes to CONNECTED as Slave state.
*	\li if no, device remains in DISCONNECTED state
*	
*	\note Received data contain Radio_TypeDef structure.
*	\param _radioH pointer to structure which descries RFM73 module
*/
void disc_rcv_data_callback( struct Radio_TypeDef * _radioH ) ;



/**	\brief Callback function for DISCONNECTED state 
*
* 	Function called when packet was sent and delivered properly
*	
*	\note In that state of connectoin it isn't relevant.
*	\param _radioH pointer to structure which descries RFM73 module
*/
void disc_packet_sent_callback( struct Radio_TypeDef * _radioH ) ;



/**	\brief Callback function for DISCONNECTED state 
*
* 	Function called when packet couldn't be delivered properly
*	
*	\note In that state of connectoin it isn't relevant.
*	\param _radioH pointer to structure which descries RFM73 module
*	\return uint8_t value - to flush or not TX buffer in RFM73
*/
uint8_t disc_cant_send_callback( struct Radio_TypeDef * _radioH ) ;




/* ***************************************************************************** */
/* ***************************************************************************** */
/* ***************************************************************************** */




/**	\brief	Callback functions for CONNECTED state - SLAVE (receiver) mode
* 	
* 	Function called when data was received from RFM73 module	 
* 	\n \n
*	Place here not too long code or function to handle received data
*	Otherwise you should set proper flag and handle your function
*	in main. This function shouldn't be to long because the incoming
*	data could be missed.
* 	\n \n
*	That function also check for 'disconnect' packet, when it will be received
* 	then the disconnection procedure will be performed.
*	\n \n
*	Also resets time between two received packets with data:
*	SysStat_TypeDef::s_timelast
*	
*/
void Sconn_rcv_data_callback( struct Radio_TypeDef * _radioH ) ;




/**	\brief	Callback functions for CONNECTED state - SLAVE (receiver) mode
* 	
* 	Function called when packet was sent and delivered properly	 
*
*	\note In that state of connectoin it isn't relevant.
*	\param _radioH pointer to structure which descries RFM73 module
*/
void Sconn_packet_sent_callback( struct Radio_TypeDef * _radioH ) ;



/**	\brief Callback functions for CONNECTED state - SLAVE (receiver) mode
*
* 	Function called when packet couldn't be delivered properly
*
*	\note In that state of connectoin it isn't relevant.
*	\param _radioH pointer to structure which descries RFM73 module
*	\return uint8_t value - to flush or not TX buffer in RFM73
*/
uint8_t Sconn_cant_send_callback( struct Radio_TypeDef * _radioH ) ;






/* ***************************************************************************** */
/* ***************************************************************************** */
/* ***************************************************************************** */






/**	\brief	Callback functions for CONNECTED state - MASTER (transmitter) mode
* 	
* 	Function called when data was received from RFM73 module	 
* 	\n \n
*	\note In that state of connectoin it isn't relevant.
* 	\param _radioH pointer to structure which descries RFM73 module
*/
void Mconn_rcv_data_callback( struct Radio_TypeDef * _radioH ) ;



/**	\brief	Callback functions for CONNECTED state - MASTER (transmitter) mode
* 	
* 	Function called when packet was sent and delivered properly	 
* 	\n
*	Resets SysStat_TypeDef::m_missing_cnt when value is less than
*	SYSTEM_NOT_CONFIRMED_MAX otherwise disconnection procedure will be performed
*
*	\param _radioH pointer to structure which descries RFM73 module
*/
void Mconn_packet_sent_callback( struct Radio_TypeDef * _radioH ) ;





/**	\brief	Callback functions for CONNECTED state - MASTER (transmitter) mode
* 	
* 	Function called when packet couldn't be delivered properly
* 	\n
*	Increments SysStat_TypeDef::m_missing_cnt
*
*	\param _radioH pointer to structure which descries RFM73 module
* 	\return uint8_t value - to flush or not TX buffer in RFM73
*/
uint8_t Mconn_cant_send_callback( struct Radio_TypeDef * _radioH ) ;








/* ***************************************************************************** */
/* ***************************************************************************** */
/* ***************************************************************************** */








/**	\brief	Callback functions for connection procedure when called try_connect()
* 	
* 	Function called when data was received from RFM73 module	 
*
*	\note In that state of connectoin it isn't relevant.
* 	\param _radioH pointer to structure which descries RFM73 module
*/
void master_rcv_data_callback( struct Radio_TypeDef * ) ;





/**	\brief	Callback functions for connection procedure when called try_connect()
* 	
* 	Function called when packet was sent and delivered properly. That will
*	happen when device try to be Master (transmitter) and it indicates that 
*	connection has been established. Device goes to CONNECTED as Master state.
* 	\n \n
*	That function do:
* 	\li sets proper bits in SysStat_TypeDef::conn_status
*	\li turns on green LED
*
* 	\param _radioH pointer to structure which descries RFM73 module
*/
void master_connected_callback( struct Radio_TypeDef * ) ;	






/**	\brief	Callback functions for connection procedure when called try_connect()
* 	
* 	Function called when packet couldn't be delivered properly.
* 	\n \n
*	Makes sure to stay in disconnect state.
*
* 	\param _radioH pointer to structure which descries RFM73 module
*	\return uint8_t value - to flush or not TX buffer in RFM73
*/
uint8_t master_cant_conn_callback( struct Radio_TypeDef * ) ;













	/* 		!!!!! NOT USED NOW !!!!!
				Callback functions for main program 
	*/
void data_ready_callback( struct Radio_TypeDef * _radioH ) ;
uint8_t cant_send_callback ( struct Radio_TypeDef * _radioH ) ;
void packet_sent_callback( struct Radio_TypeDef *) ;


#endif
