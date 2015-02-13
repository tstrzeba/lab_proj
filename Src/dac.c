#include "dac.h"

struct DAC_BUFF dac_buff;

// For HAL drivers:
DAC_HandleTypeDef hdac;

// For proper DAC sampling frequency
extern TIM_HandleTypeDef htim2 ;


void dac_buff_init( void ) {
	dac_buff.buff_it = 0;
	dac_buff.buff_max = DAC_BUFF_SIZE;
	dac_buff.p_buff = dac_buff.dac_buffer0;
	dac_buff.data_ready = 0;
	dac_buff.i = 0;
	dac_buff.buff_in_use = 0;
}

/* Not need
void dac_buff_clear(void) {
	dac_buff.buff_it = 0;
	dac_buff.p_buff = dac_buff.dac_buffer0;
	dac_buff.data_ready = 0;
	dac_buff.i = 0;
	dac_buff.buff_in_use = 0;
	
}
*/

void dac_buff_append(uint8_t * const _rcv_data, uint8_t _adc_buff_size ) {
	uint8_t *p_adc_buff = _rcv_data;

	for (dac_buff.i = 0; dac_buff.i < (_adc_buff_size/3); dac_buff.i++) {
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

	if (dac_buff.buff_it >= DAC_BUFF_SIZE) {
		dac_buff.data_ready = 1;
		dac_buff.buff_it = 0;
		dac_buff.p_buff_ready = dac_buff.p_buff - DAC_BUFF_SIZE;
		if (dac_buff.buff_in_use == 0) {
			dac_buff.p_buff = dac_buff.dac_buffer1;
			dac_buff.buff_in_use++;
		}
		else {
			dac_buff.p_buff = dac_buff.dac_buffer0;
			dac_buff.buff_in_use--;
		}
	}
}

void dac_data_ready(void) {
	if (dac_buff.data_ready == 1) {
		dac_buff.p_buff_ready--;
		
		
		dac_ON();

		/*
		for (dac_buff.i = dac_buff.buff_max; dac_buff.i > 0; dac_buff.i--, dac_buff.p_buff_ready--) {
			*dac_buff.p_buff_ready = 0;
		}
		*/
		dac_buff.data_ready = 0;
	}
}

void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  HAL_DAC_Init(&hdac);

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);
}

void dac_ON(void){
	DAC->CR |= DAC_CR_EN1;
	// Enable timer too
	htim2.Instance->CR1 |= TIM_CR1_CEN;
}
void dac_OFF(void){
	DAC->CR &= ~DAC_CR_EN1;
	// Disable timer too
	htim2.Instance->CR1 &= ~TIM_CR1_CEN;
}
