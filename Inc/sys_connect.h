#ifndef _SYS_CONNECT_H
#define _SYS_CONNECT_H

#include "radio_lib.h"

/** \file sys_connec.h sys_connect.c
*	
*	These files contain defines and fnctions which allow to:
* 	\li establish connection between two devices
* 	\li disconnection
*
*/


/** 
* 	Defines which describes possible values returned by try_connect() 
*/
#define RET_M_CONNECTED (uint8_t)0		/**< When connection was established */
#define RET_M_CANT_CONNECT (uint8_t)1	/**< When connection couldn't be established */
#define RET_M_TIMEOUT (uint8_t)2 		/**< When connection procedure lasted too long */




/** 
* 	Function for Master device - it tries to connect to another device
*
*	\param struct Radio_TypeDef * pointer to radio structure which describes RFM73 module
*	\param _pipe_conn_addr five bytes table which contain address for first pipe
* 	\param _pipe_data_addr five bytes table which contain address for second pipe
*	
*	\return it returns one of values desribed by: RET_M_CONNECTED, RET_M_CANT_CONNECT, RET_M_TIMEOUT
*/
uint8_t try_connect( 
	struct Radio_TypeDef *, 
	const uint8_t * const _pipe_conn_addr, 
	const uint8_t * const _pipe_data_addr
	) ;


/** \brief Disconnection function only for Master device
*	
*	It can be used in RFM73 module callbacks
*
*	\note If called from callback pass NULL in second parameter
* 	\param struct Radio_TypeDef * pointer to radio structure which describes RFM73 module
* 	\param _pipe_conn_addr five bytes table which contain address for first pipe
*/
void master_disconnect_main( 
	struct Radio_TypeDef *, 
	const uint8_t * const _pipe_conn_addr
	) ;
	
	
/**	\brief Disconnection function only for Slave device
*	\param struct Radio_TypeDef * pointer to radio structure which describes RFM73 module
*/
void slave_disconnect( struct Radio_TypeDef * _radioH ) ;
	
#endif
