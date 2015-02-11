#ifndef __adc_H
#define __adc_H
#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stdint.h"
#include "stm32f4xx_hal.h"

#define ADC_BUFF_SIZE 30
	 
extern ADC_HandleTypeDef hadc3;

void MX_ADC3_Init(void);
	 
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

void adc_buff_append(uint16_t value);
void adc_buff_init(void);
void adc_data_ready(void);
void adc_buff_clear(void);
void adc_ON(void);
void adc_OFF(void);

#endif
