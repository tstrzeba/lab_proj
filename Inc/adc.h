#ifndef __adc_H
#define __adc_H
	 
#include "stdint.h"
#include "stm32f4xx_hal.h"

/// Maximum ADC buffer size	
#define ADC_BUFF_SIZE 30

/// For HAL drivers
extern ADC_HandleTypeDef hadc3;

/**
*Struct contained ADC variables and ADC buffers. That struct containes:
*- buff_max - maximum ADC buffer size defined by ADC_BUFF_SIZE
*- p_buff - pointer to buffer element in which store actual sample received from ADC module
*- p_buff_ready - pointer to buffer element which send to RF module
*- buff_it, buff_i - used in for loop
*- buff_full - buffer_full flag
* buff_in_use - which buffer (buffer0 or buffer1) is used to store data from ADC module
*- buffer0, buffer1 - buffers which store samples received from ADC module, one module is used to store data
*received from ADC and second to store data to send to RF module
*/ 
struct ADC_BUFF {
	volatile uint8_t buff_max;
	volatile uint8_t* volatile p_buff;
	volatile uint8_t* volatile p_buff_ready;
	volatile uint8_t buff_it;
	volatile uint8_t buff_full;
	volatile uint8_t buff_in_use;
	volatile uint8_t buffer0[ADC_BUFF_SIZE];
	volatile uint8_t buffer1[ADC_BUFF_SIZE];
	volatile uint8_t i;
} ;

/// Initialize ADC
void MX_ADC3_Init(void);

/// Convert data received from ADC (12 bits to 8 bits) and send them to RF module
void adc_buff_append(uint16_t value);

/// Initialize ADC buffer
void adc_buff_init(void);

/// Used in main, check if data in ADC buffer are ready to send (RF module transmit 30B data)
void adc_data_ready(void);

/// Clear ADC buffer
void adc_buff_clear(void);

/// Enable ADC
inline void adc_ON(void);

/// Disable ADC
inline void adc_OFF(void);

#endif
