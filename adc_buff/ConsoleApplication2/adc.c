#include "adc.h"
#include "stdlib.h"

ADC_BUFF adc_buff;

void adc_buff_append(uint16_t value) {
	if (adc_buff.i == 0) {
		*adc_buff.p_buff = (uint8_t)(value >> 4);
		adc_buff.p_buff++;
		adc_buff.buff_it++;
		*adc_buff.p_buff = (uint8_t)(value << 4);
		adc_buff.i++;
	}
	else if (adc_buff.i == 1) {
		*adc_buff.p_buff |= (uint8_t)(value >> 8);
		adc_buff.p_buff++;
		adc_buff.buff_it++;
		*adc_buff.p_buff = (uint8_t)value;
		adc_buff.p_buff++;
		adc_buff.buff_it++;
		adc_buff.i = 0;
	}
	
	if (adc_buff.buff_it == adc_buff.buff_max) {
		adc_buff.buff_full = 1;
		adc_buff.buff_it = 0;
		adc_buff.p_buff_ready = adc_buff.p_buff;
		if (adc_buff.buff_in_use == 0) {
			adc_buff.p_buff = adc_buff.buffer1;
			adc_buff.buff_in_use++;
		}
		else {
			adc_buff.p_buff = adc_buff.buffer0;
			adc_buff.buff_in_use--;
		}
	}
};

void adc_data_ready() {
	if (adc_buff.buff_full == 1) {
		adc_buff.p_buff_ready--;
		printf("NOWY BUFOR\n");
		//RF_SEND (adc_buff.p_buff);
		for (adc_buff.i = adc_buff.buff_max; adc_buff.i > 0; adc_buff.i--, adc_buff.p_buff_ready--) {
			printf("%#x\n", *adc_buff.p_buff_ready);
			*adc_buff.p_buff_ready = 0;
		}
		adc_buff.buff_full = 0;
	}
}

void adc_buff_init() {
	adc_buff.buff_max = ADC_BUFF_SIZE;
	adc_buff.buff_it = 0;
	adc_buff.p_buff = adc_buff.buffer0;
	adc_buff.buff_full = 0;
	adc_buff.buff_in_use = 0;
	adc_buff.i = 0;
}

int main(void) {
	uint16_t a = 0;
	adc_buff_init();
	for (a; a < 60; a++)
	{
		printf("%#x\n", 2134 - a);
		adc_buff_append(2134-a);
		adc_data_ready();
	}
	system("PAUSE");
}