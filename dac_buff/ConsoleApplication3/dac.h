#include <stdint.h>

#define ADC_BUFF_SIZE 30
#define DAC_BUFF_SIZE 60

typedef struct DAC_BUFF {
	volatile uint8_t buff_max;
	volatile uint8_t buff_it;
	volatile uint16_t* volatile p_buff;
	volatile uint8_t dac_data_ready;
	volatile uint16_t dac_buffer[DAC_BUFF_SIZE];
	volatile uint8_t i;
} DAC_BUFF;

void dac_buff_init();
void dac_buff_append(uint8_t *p_adc_buff);