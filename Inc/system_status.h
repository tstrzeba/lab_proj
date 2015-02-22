#ifndef _SYSTEM_STATUS_H_
#define _SYSTEM_STATUS_H_

#include "stdint.h"

/** \file system_status.h
* 	This file contain: 
* 	\li definiton of #SysStat_TypeDef structure (which is base of 
*	connection system)
* 	\li defines bits used in #SysStat_TypeDef
* 	\li defines values that are used to detect connection lost
*
*/

/** 
* 	It describes size of SysStat_TypeDef::conn_status variable
*/
typedef uint8_t conn_stat_TypeDef ;


/** \brief It contains information about connection status
*
*	\note That stucture is the base of that project.
*	\n To determine status of connection check SYSTEM_CONNECTED_BIT
*	and SYSTEM_MASTER_MODE_BIT in SysStat_TypeDef::conn_status
*
*/
struct SysStat_TypeDef {

	/** Contain information/status about connection. \n
	*	See SYSTEM_CONNECTED_BIT, SYSTEM_CANT_CONNECT_BIT, SYSTEM_MASTER_MODE_BIT
	* 	to get meaning bits.
	*/
	volatile conn_stat_TypeDef conn_status ;	

	/** Used to detect connection lost by Master device. \n
	*	Count when receiver is out of range - not confirmed packets
	*/
	uint8_t m_missing_cnt ;

	/** Used to detect connection lost by Slave device. \n
	*	'Time' of last received packet - using for estimate timeout
	* 	when master not send data anymore and non disconnection
	* 	message was received
	*/
	volatile uint32_t s_timelast;
} ;



/** If 1 it means that connection was established */
#define SYSTEM_CONNECTED_BIT 0
#define SYSTEM_CONNECTED_MASK (conn_stat_TypeDef)(1 << SYSTEM_CONNECTED_BIT)


/** If 1 it means that there is no proper device on the range of transmitter*/
#define SYSTEM_CANT_CONNECT_BIT 1
#define SYSTEM_CANT_CONNECT_MASK (conn_stat_TypeDef)(1 << SYSTEM_CANT_CONNECT_BIT)

/** If 1 it means that that device is in master mode, 0 - in slave mode */
#define SYSTEM_MASTER_MODE_BIT 2
#define SYSTEM_MASTER_MODE_MASK (conn_stat_TypeDef)(1 << SYSTEM_MASTER_MODE_BIT)



/**	Used as threshold value to detect connection lost by Master device.
*	\n
*	How many packets in a row could be send without confirmation - MASTER mode
* 	When counter riches that value the disconnection function will be called.
*/
#define SYSTEM_NOT_CONFIRMED_MAX 20


/** Used as threshold value to detect connection lost by Slave device.
* 	\n
*	Time in [ms] after which SLAVE device call disconnect function.
*/
#define SYSTEM_SLAVE_TIMEOUT_VAL 1000


#endif
