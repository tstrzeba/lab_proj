#include "dac.h"

struct DAC_BUFF dac_buff;



// For HAL drivers:
DAC_HandleTypeDef hdac;

// For proper DAC sampling frequency
extern TIM_HandleTypeDef htim2 ;


void dac_buff_init( void ) {
	/*
	#ifdef __DBG_ITM
	// Init table with 0.3*sin(1kHz) + 1.65 sampled by 10kHz
dac_buff.dac_wheel_buffer[0] = 2253 ; dac_buff.dac_wheel_buffer[1] = 2494 ; dac_buff.dac_wheel_buffer[2] = 2642 ; dac_buff.dac_wheel_buffer[3] = 2642 ; dac_buff.dac_wheel_buffer[4] = 2494 ; dac_buff.dac_wheel_buffer[5] = 2253 ; dac_buff.dac_wheel_buffer[6] = 2012 ; dac_buff.dac_wheel_buffer[7] = 1863 ; dac_buff.dac_wheel_buffer[8] = 1863 ; dac_buff.dac_wheel_buffer[9] = 2012 ; dac_buff.dac_wheel_buffer[10] = 2253 ; 
dac_buff.dac_wheel_buffer[11] = 2253 ; dac_buff.dac_wheel_buffer[12] = 2494 ; dac_buff.dac_wheel_buffer[13] = 2642 ; dac_buff.dac_wheel_buffer[14] = 2642 ; dac_buff.dac_wheel_buffer[15] = 2494 ; dac_buff.dac_wheel_buffer[16] = 2253 ; dac_buff.dac_wheel_buffer[17] = 2012 ; dac_buff.dac_wheel_buffer[18] = 1863 ; dac_buff.dac_wheel_buffer[19] = 1863 ; dac_buff.dac_wheel_buffer[20] = 2012 ; dac_buff.dac_wheel_buffer[21] = 2253 ; 
dac_buff.dac_wheel_buffer[22] = 2253 ; dac_buff.dac_wheel_buffer[23] = 2494 ; dac_buff.dac_wheel_buffer[24] = 2642 ; dac_buff.dac_wheel_buffer[25] = 2642 ; dac_buff.dac_wheel_buffer[26] = 2494 ; dac_buff.dac_wheel_buffer[27] = 2253 ; dac_buff.dac_wheel_buffer[28] = 2012 ; dac_buff.dac_wheel_buffer[29] = 1863 ; dac_buff.dac_wheel_buffer[30] = 1863 ; dac_buff.dac_wheel_buffer[31] = 2012 ; dac_buff.dac_wheel_buffer[32] = 2253 ; 
dac_buff.dac_wheel_buffer[33] = 2253 ; dac_buff.dac_wheel_buffer[34] = 2494 ; dac_buff.dac_wheel_buffer[35] = 2642 ; dac_buff.dac_wheel_buffer[36] = 2642 ; dac_buff.dac_wheel_buffer[37] = 2494 ; dac_buff.dac_wheel_buffer[38] = 2253 ; dac_buff.dac_wheel_buffer[39] = 2012 ; dac_buff.dac_wheel_buffer[40] = 1863 ; dac_buff.dac_wheel_buffer[41] = 1863 ; dac_buff.dac_wheel_buffer[42] = 2012 ; dac_buff.dac_wheel_buffer[43] = 2253 ; 
dac_buff.dac_wheel_buffer[44] = 2253 ; dac_buff.dac_wheel_buffer[45] = 2494 ; dac_buff.dac_wheel_buffer[46] = 2642 ; dac_buff.dac_wheel_buffer[47] = 2642 ; dac_buff.dac_wheel_buffer[48] = 2494 ; dac_buff.dac_wheel_buffer[49] = 2253 ; dac_buff.dac_wheel_buffer[50] = 2012 ; dac_buff.dac_wheel_buffer[51] = 1863 ; dac_buff.dac_wheel_buffer[52] = 1863 ; dac_buff.dac_wheel_buffer[53] = 2012 ; dac_buff.dac_wheel_buffer[54] = 2253 ; 
dac_buff.dac_wheel_buffer[55] = 2253 ; dac_buff.dac_wheel_buffer[56] = 2494 ; dac_buff.dac_wheel_buffer[57] = 2642 ; dac_buff.dac_wheel_buffer[58] = 2642 ; dac_buff.dac_wheel_buffer[59] = 2494 ;
dac_buff.dac_wheel_buffer[60] = 2253 ; dac_buff.dac_wheel_buffer[61] = 2012 ; dac_buff.dac_wheel_buffer[62] = 1863 ; dac_buff.dac_wheel_buffer[63] = 1863 ; dac_buff.dac_wheel_buffer[64] = 2012 ; dac_buff.dac_wheel_buffer[65] = 2253 ;
#endif
	*/

	dac_buff.buff_max = DAC_BUFF_SIZE;
	dac_buff.p_buff = dac_buff.dac_wheel_buffer;
	dac_buff.p_buff_dac = dac_buff.dac_wheel_buffer;
	dac_buff.i = 0;
	dac_buff.is_dac_on = 0;
	dac_buff.buff_overflow = 0;
	
	
//#ifdef __DBG_ITM
	DAC->CR |= DAC_CR_EN1;
//#endif 
	
	
}

void dac_buff_append(uint8_t * const _rcv_data, uint8_t _adc_buff_size ) {
	uint8_t *p_adc_buff = _rcv_data;

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
		if (dac_buff.p_buff >= (dac_buff.dac_wheel_buffer + (DAC_BUFF_SIZE - 1))) {
			dac_buff.p_buff = dac_buff.dac_wheel_buffer;
			//dac_buff.buff_overflow = 1;
		}
	}
}

void dac_data_ready(void) {
	//if (dac_buff.buff_overflow == 1) {
	if (dac_buff.p_buff_dac != dac_buff.p_buff) {
		if (dac_buff.is_dac_on == 0) {
			dac_ON();
			dac_buff.is_dac_on = 1;
		}
	}
//}
	/*else {
		if (dac_buff.p_buff_dac < dac_buff.p_buff) {
		if (dac_buff.is_dac_on == 0) {
			dac_ON();
			dac_buff.is_dac_on = 1;
		}
	}
	}*/
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
	//DAC->CR |= DAC_CR_EN1;
	// Enable timer too
	htim2.Instance->CR1 |= TIM_CR1_CEN;
}
void dac_OFF(void){
	//DAC->CR &= ~DAC_CR_EN1;
	// Disable timer too
	htim2.Instance->CR1 &= ~TIM_CR1_CEN;
}



//#ifdef __DBG_ITM

		void dac_buff_imit_append (void) {
			
			// Only move pointer - like that while data received are converting
			
			if ( (dac_buff.p_buff + 30) <= ( dac_buff.dac_wheel_buffer + (DAC_BUFF_SIZE-1)) ) {
				dac_buff.p_buff += 30 ;
			} else {
				dac_buff.p_buff = dac_buff.dac_wheel_buffer +( 30 - ( (dac_buff.dac_wheel_buffer + (DAC_BUFF_SIZE-1)) - dac_buff.p_buff) );
			//	dac_buff.buff_overflow = 1;
			}
			
		}
//#endif
