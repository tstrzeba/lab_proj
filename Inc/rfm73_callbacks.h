#ifndef _RFM73_CALLBACKS_
#define _RFM73_CALLBACKS_

#include <stdint.h>
#include "radio_lib.h"

void data_ready_callback( struct Radio_TypeDef * _radioH ) ;
uint8_t cant_send_callback ( struct Radio_TypeDef * _radioH ) ;
void packet_sent_callback( struct Radio_TypeDef *) ;


#endif
