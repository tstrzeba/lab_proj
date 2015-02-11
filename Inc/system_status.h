#ifndef _SYSTEM_STATUS_H_
#define _SYSTEM_STATUS_H_

#include "stdint.h"

typedef uint8_t conn_stat_TypeDef ;

struct SysStat_TypeDef {
	volatile conn_stat_TypeDef conn_status ;
} ;


// If 1 it means that connection was established
#define SYSTEM_CONNECTED_BIT 0
#define SYSTEM_CONNECTED_MASK (conn_stat_TypeDef)(1 << SYSTEM_CONNECTED_BIT)

// If 1 it means that there is no proper device on the range of transmitter
#define SYSTEM_CANT_CONNECT_BIT 1
#define SYSTEM_CANT_CONNECT_MASK (conn_stat_TypeDef)(1 << SYSTEM_CANT_CONNECT_BIT)

// If 1 it means that that device is in master mode, 0 - in slave mode
#define SYSTEM_MASTER_MODE_BIT 2
#define SYSTEM_MASTER_MODE_MASK (conn_stat_TypeDef)(1 << SYSTEM_MASTER_MODE_BIT)


#endif
