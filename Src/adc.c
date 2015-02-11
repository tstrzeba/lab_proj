#include "adc.h"

struct ADC_BUFF adc_buff;

ADC_HandleTypeDef hadc3;

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

void adc_data_ready(void) {
	if (adc_buff.buff_full == 1) {
		adc_buff.p_buff_ready--;
		//RF_SEND (adc_buff.p_buff);
		for (adc_buff.i = adc_buff.buff_max; adc_buff.i > 0; adc_buff.i--, adc_buff.p_buff_ready--) {
			*adc_buff.p_buff_ready = 0;
		}
		adc_buff.buff_full = 0;
	}
}

void adc_buff_init(void) {
	adc_buff.buff_max = ADC_BUFF_SIZE;
	adc_buff.buff_it = 0;
	adc_buff.p_buff = adc_buff.buffer0;
	adc_buff.p_buff_ready = 0;
	adc_buff.buff_full = 0;
	adc_buff.buff_in_use = 0;
	adc_buff.i = 0;
}

void adc_buff_clear(void) {
	adc_buff.buff_it = 0;
	adc_buff.p_buff = adc_buff.buffer0;
	adc_buff.p_buff_ready = 0;
	adc_buff.buff_full = 0;
	adc_buff.buff_in_use = 0;
	for (adc_buff.i = 0; adc_buff.i < ADC_BUFF_SIZE; adc_buff.i++) {
		adc_buff.buffer0[adc_buff.i] = 0;
		adc_buff.buffer1[adc_buff.i] = 0;
	}
	adc_buff.i = 0;
	
}

void adc_ON(void){
	ADC3->CR2 |= ADC_CR2_ADON;
		ADC3->CR2 |= ADC_CR2_SWSTART;
}

void adc_OFF(void){
	ADC3->CR2 &= ~ADC_CR2_ADON;
		ADC3->CR2 &= ~ADC_CR2_SWSTART;
}

void MX_ADC3_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV8;
  hadc3.Init.Resolution = ADC_RESOLUTION12b;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  //hadc3.Init.EOCSelection = EOC_SEQ_CONV;
  HAL_ADC_Init(&hadc3);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);
	ADC3->CR1 |= ADC_CR1_EOCIE;
}