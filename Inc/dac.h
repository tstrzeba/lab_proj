#ifndef __dac_H
#define __dac_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
	
/// Maximum DAC buffer size	
#define DAC_BUFF_SIZE 180

/// For HAL drivers
extern DAC_HandleTypeDef hdac;

/**
*Struct contained DAC variables and DAC wheel buffer. That struct containes:
*- buff_max - maximum DAC buffer size defined by DAC_BUFF_SIZE
*- p_buff - pointer to buffer element in which store actual sample received from RF module
*- p_buff_dac - pointer to buffer element which send to DAC
*- i - used in for loop
*- dac_wheel_buffer - buffer which stores samples received from RF module
*- is_dac_on - is timer on or off
*- buff_overflow - buffer overflow flag
*/
struct DAC_BUFF {
	volatile uint8_t buff_max;
	volatile uint16_t* volatile p_buff;
	volatile uint16_t* volatile p_buff_dac;
	volatile uint8_t i;
	volatile uint16_t dac_wheel_buffer[DAC_BUFF_SIZE];
	volatile uint8_t is_dac_on;
	volatile uint8_t buff_overflow;
} ;

/// Initialize DAC
void MX_DAC_Init(void);

/// Initialize DAC buffer
void dac_buff_init(void);	

/// Convert data received from RF module (8 bits to 12 bits) and place them into DAC buffer
void dac_buff_append(uint8_t * const _rcv_data, uint8_t _adc_buff_size);

/// Used in main, check if data in DAC buffer are ready to send (check if p_buff is equal to p_buff_dac)
/// and if not enables TIM2
void dac_data_ready(void);

/// Enable TIM2
inline void dac_ON(void);

/// Disable TIM2
inline void dac_OFF(void);

#endif
