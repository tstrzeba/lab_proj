#ifndef __dac_H
#define __dac_H
#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"
	 
#define DAC_BUFF_SIZE 66

extern DAC_HandleTypeDef hdac;

void MX_DAC_Init(void);

struct DAC_BUFF {
	volatile uint8_t buff_max;
	volatile uint16_t* volatile p_buff;
	volatile uint16_t* volatile p_buff_dac;
	volatile uint8_t i;
	volatile uint16_t dac_wheel_buffer[DAC_BUFF_SIZE];
	volatile uint8_t is_dac_on;
	volatile uint8_t buff_overflow;
} ;

void dac_buff_init(void);
void dac_buff_append(uint8_t * const _rcv_data, uint8_t _adc_buff_size);
void dac_buff_clear(void);
void dac_data_ready(void);
void dac_ON(void);
void dac_OFF(void);



// Only for testing dac
void dac_buff_imit_append( void ) ;

#endif
