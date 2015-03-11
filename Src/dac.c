#include "dac.h"

struct DAC_BUFF dac_buff;

/// For HAL drivers
DAC_HandleTypeDef hdac;

/// For proper DAC sampling frequency
extern TIM_HandleTypeDef htim2 ;


void dac_buff_init( void ) {
	dac_buff.buff_max = DAC_BUFF_SIZE;
	dac_buff.p_buff = dac_buff.dac_wheel_buffer;			/// p_buff points to beginnig of buffer
	dac_buff.p_buff_dac = dac_buff.dac_wheel_buffer;	/// p_buff_dac points to beginnig of buffer
	dac_buff.i = 0;
	dac_buff.is_dac_on = 0;	/// disable TIM2
	dac_buff.buff_overflow = 0;
	DAC->CR |= DAC_CR_EN1;	/// enable DAC
}

void dac_buff_append(uint8_t * const _rcv_data, uint8_t _adc_buff_size ) {
	uint8_t *p_adc_buff = _rcv_data;
	/// Convert data from 8 bits to 12 bits
	for (dac_buff.i = 0; dac_buff.i < (_adc_buff_size/3); dac_buff.i++) {
		*dac_buff.p_buff = ((uint16_t)(*p_adc_buff)) << 4;
		p_adc_buff++;
		*dac_buff.p_buff |= (uint16_t)(*p_adc_buff >> 4);
		dac_buff.p_buff++;
		*dac_buff.p_buff = (((uint16_t)(*p_adc_buff)) << 8) & 0x0fff;
		p_adc_buff++;
		*dac_buff.p_buff |= *p_adc_buff;
		p_adc_buff++;
		dac_buff.p_buff++;
		
	/// If p_buff points to end of buffer change it to point to beginning of buffer
		if (dac_buff.p_buff >= (dac_buff.dac_wheel_buffer + (DAC_BUFF_SIZE - 1))) {
			dac_buff.p_buff = dac_buff.dac_wheel_buffer;
		} 
	}
}

void dac_data_ready(void) {
	if (dac_buff.p_buff_dac != dac_buff.p_buff) {
		if (dac_buff.is_dac_on == 0) {
			dac_ON();
			dac_buff.is_dac_on = 1;
		}
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
	TIM2->CR1 |= TIM_CR1_CEN;
	//htim2.Instance->CR1 |= TIM_CR1_CEN;
}
void dac_OFF(void){
	TIM2->CR1 &= ~TIM_CR1_CEN ;
	//htim2.Instance->CR1 &= ~TIM_CR1_CEN;
}
