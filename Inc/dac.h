#ifndef __dac_H
#define __dac_H
#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include "adc.h"

#define DAC_BUFF_SIZE 60

extern DAC_HandleTypeDef hdac;

void MX_DAC_Init(void);

struct DAC_BUFF {
	volatile uint8_t buff_max;
	volatile uint8_t buff_it;
	volatile uint16_t* volatile p_buff;
	volatile uint16_t* volatile p_buff_ready;
	volatile uint8_t data_ready;
	volatile uint8_t buff_in_use;
	volatile uint16_t dac_buffer0[DAC_BUFF_SIZE];
	volatile uint16_t dac_buffer1[DAC_BUFF_SIZE];
	volatile uint8_t i;
} ;

void dac_buff_init(void);
void dac_buff_append(uint8_t * const _rcv_data, uint8_t _adc_buff_size);
void dac_buff_clear(void);
void dac_data_ready(void);
void dac_ON(void);
void dac_OFF(void);
#endif
