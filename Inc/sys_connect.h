#ifndef _SYS_CONNECT_H
#define _SYS_CONNECT_H

#include "radio_lib.h"


/* Possible returned value from try_connect() */
#define RET_M_CONNECTED (uint8_t)0
#define RET_M_CANT_CONNECT (uint8_t)1
#define RET_M_TIMEOUT (uint8_t)2




/** 
* 	\brief Function for master - it try to connect to another device
*/
uint8_t try_connect( 
	struct Radio_TypeDef *, 
	const uint8_t * const _pipe_conn_addr, 
	const uint8_t * const _pipe_data_addr
	) ;


/** 
* 	\brief Function for master - disconnecting
*  It probably can be used in callbacks - but be careful
*  If called from callback pass NULL in second parameter
*/
void master_disconnect_main( 
	struct Radio_TypeDef *, 
	const uint8_t * const _pipe_conn_addr
	) ;
	
	
/**
*		\brief Function for slave - disconnecting
*/
void slave_disconnect( struct Radio_TypeDef * _radioH ) ;
	
#endif
