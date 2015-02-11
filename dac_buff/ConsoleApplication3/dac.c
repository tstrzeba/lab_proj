#include "dac.h"

DAC_BUFF dac_buff;

void dac_buff_init() {
	dac_buff.buff_it = 0;
	dac_buff.buff_max = DAC_BUFF_SIZE;
	dac_buff.p_buff = dac_buff.dac_buffer;
	dac_buff.dac_data_ready = 0;
	dac_buff.i = 0;
}

void dac_buff_append(uint8_t *p_adc_buff) {
	for (dac_buff.i = 0; dac_buff.i < ADC_BUFF_SIZE / 3; dac_buff.i++) {
		*dac_buff.p_buff = ((uint16_t)(*p_adc_buff)) << 4;
		p_adc_buff++;
		*dac_buff.p_buff |= (uint16_t)(*p_adc_buff >> 4);
		dac_buff.p_buff++;
		dac_buff.buff_it++;
		*dac_buff.p_buff = (((uint16_t)(*p_adc_buff)) << 8) & 0x0fff;
		p_adc_buff++;
		*dac_buff.p_buff |= *p_adc_buff;
		dac_buff.p_buff++;
		dac_buff.buff_it++;
		p_adc_buff++;
	}

	//printf("ititit %u ititit", dac_buff.buff_it);
	if (dac_buff.buff_it == dac_buff.buff_max) {
		dac_buff.p_buff--;
		//SEND TO DAC
		printf("NOWY BUFOR\n");
		dac_buff.buff_it = 0;
		for (dac_buff.i = dac_buff.buff_max; dac_buff.i > 0; dac_buff.i--, dac_buff.p_buff--) {
		//	printf("%#x\n", *dac_buff.p_buff);
			*dac_buff.p_buff = 0;
		}

	}
}

int main(void) {
	uint8_t a = 0;
	uint8_t tab[120];
	uint8_t *p_tab;

	dac_buff_init();
	for (a = 0; a < 60; a++) {
		tab[a] = 164-a;
		printf("%#x\n", tab[a]);
	}
	p_tab = tab;
	dac_buff_append(p_tab);
	dac_buff_append(p_tab+30);
	/*for (a; a < 100; a++)
	{
		printf("%#x\n", 2134 - a);
		dac_buff_append(2134 - a);
	}*/
	system("PAUSE");
}