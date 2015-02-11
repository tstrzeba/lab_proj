#include "stdint.h"

#define ADC_BUFF_SIZE 30

typedef struct ADC_BUFF {
	volatile uint8_t buff_max;
	volatile uint8_t* volatile p_buff;
	volatile uint8_t* volatile p_buff_ready;
	volatile uint8_t buff_it;
	volatile uint8_t buff_full;
	volatile uint8_t buff_in_use;
	volatile uint8_t buffer0[ADC_BUFF_SIZE];
	volatile uint8_t buffer1[ADC_BUFF_SIZE];
	volatile uint8_t i;
} ADC_BUFF;

void adc_buff_append(uint16_t value);
void adc_buff_init();
void adc_data_ready(uint8_t buffer[]);