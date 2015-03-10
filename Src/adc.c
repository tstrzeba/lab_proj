#include "adc.h"
#include "radio_lib.h"

#include "arm_math.h"

struct ADC_BUFF adc_buff;

/// For HAL drivers
ADC_HandleTypeDef hadc3;
extern struct Radio_TypeDef radio1;


// FILTERING purposes:

struct ADC_PRE_FILTER adc_prefilter ;

arm_fir_instance_f32 S; 
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1]; 
const float32_t firCoeffs32[NUM_TAPS] = { 
-0.010428676528349761f, -0.014151231757741725f, -0.017327915499945123f, -0.014858220527324346f
-0.005513416855661676f, 0.008913322592290924f, 0.02372045041887954f, 0.03298137191056509f,
0.03232763753912525f, 0.021461516631862024f, 0.004968471560022535f, -0.009309014167126947f, 
-0.013898057578515671f, -0.005711199641298502f, 0.011667570170512261f, 0.028947736378385252f,
0.03534076152106504f, 0.024283882672566745f, -0.002081273597635951f, -0.03218167145783216f,
-0.04896984622174031f, -0.037242059403343006f, 0.008838956658702643f, 0.08158720324594303f, 
0.1612623331347346f, 0.22295189937665133f, 0.24614999500478163f, 0.22295189937665133f, 
0.1612623331347346f, 0.08158720324594303f, 0.008838956658702643f, -0.037242059403343006f, 
-0.04896984622174031f, -0.03218167145783216f, -0.0020812735976359505f, 0.024283882672566745f, 
0.03534076152106504f, 0.028947736378385252f, 0.011667570170512261f, -0.005711199641298502f, 
-0.013898057578515671f, -0.009309014167126947f, 0.004968471560022535f, 0.021461516631862028f, 
0.03232763753912525f, 0.03298137191056509f, 0.02372045041887954f, 0.008913322592290924f, 
-0.0055134168556616805f, -0.014858220527324353f, -0.017327915499945123f, -0.014151231757741725f, 
-0.010428676528349761f
}; 
static float32_t outbuffer[ BLOCK_SIZE ] ;


/* FILTERING PURPOSES */
void adc_filtering_check( void ) {
  
	static float32_t * _fir_in ;
	static float32_t * _fir_out = &outbuffer[0] ;
	
	uint8_t clear_mask = 0 ;
	
	uint8_t i ;
	
	// get pointer of filled buffer
	if( adc_prefilter.status & ADC_PRE_FILTER_FULL0_MASK ) {
			_fir_in = (float32_t*) adc_prefilter.buffers[0] ;
			clear_mask = ADC_PRE_FILTER_FULL0_MASK ;
	}
	else if ( adc_prefilter.status & ADC_PRE_FILTER_FULL1_MASK ) {
			_fir_in = (float32_t*) adc_prefilter.buffers[1] ;
			clear_mask = ADC_PRE_FILTER_FULL1_MASK ;
	}
		
	
	if ( clear_mask != 0 ) {
			// filter data
		  arm_fir_f32( &S, (float32_t*)_fir_in, (float32_t*)_fir_out, BLOCK_SIZE ) ;
			
			// Release buffer
			adc_prefilter.status &= ~clear_mask ;
			
			// Select at 4th sample and convert it
			for ( i = 0; i < adc_prefilter.buffers_size ; i += 4 )
				adc_buff_append( (uint16_t)outbuffer[i] ) ;
	}
	
}










void adc_buff_append(uint16_t value) {
	/// Convert data from 12 bits to 8 bits
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
	
	/// If p_buff point to end of buffer switch buffers and set buff_full flag
	if (adc_buff.buff_it >= adc_buff.buff_max) {
		adc_buff.buff_it = 0;
		adc_buff.buff_full = 1;
		adc_buff.p_buff_ready = adc_buff.p_buff;
		if (adc_buff.buff_in_use == 0) {
			adc_buff.p_buff = adc_buff.buffer1;
			adc_buff.buff_in_use = 1;
		}
		else {
			adc_buff.p_buff = adc_buff.buffer0;
			adc_buff.buff_in_use = 0;
			
		}
	}
}


void adc_data_ready(void) {
	/// If buffer full send buffer to RF module
	if (adc_buff.buff_full == 1) {
		adc_buff.p_buff_ready--;
		if (adc_buff.buff_in_use == 1) {
			rfm73_transmit_message( &radio1, (const uint8_t*)adc_buff.buffer0, 30 );

		}
		else {
			rfm73_transmit_message( &radio1, (const uint8_t*)adc_buff.buffer1, 30 );
		}
		adc_buff.buff_full = 0;	/// Clear buff_full flag
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
	
	/// Clear all samples
	for (adc_buff.i = 0; adc_buff.i < ADC_BUFF_SIZE; adc_buff.i++) {
		adc_buff.buffer0[adc_buff.i] = 0;
		adc_buff.buffer1[adc_buff.i] = 0;
	}
	adc_buff.i = 0;
	
}

void adc_ON(void) {
	
	// Init filter and reset ADC prefiltering buffers:
	arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], BLOCK_SIZE); 
	adc_prefilter.status = 0;
	adc_prefilter.nr_used_buff = 0 ;
	adc_prefilter.used_buff = adc_prefilter.buffers[0] ;
	
	ADC3->CR2 |= ADC_CR2_ADON;
	ADC3->CR2 |= ADC_CR2_SWSTART;
	
}

void adc_OFF(void) {
	ADC3->CR2 &= ~ADC_CR2_ADON;
	ADC3->CR2 &= ~ADC_CR2_SWSTART;
}

void MX_ADC3_Init(void) {

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2; // should be about 44 kHz // ADC_CLOCKPRESCALER_PCLK_DIV8;
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
	
	/* Enable NVIC */ 
	
	NVIC_ClearPendingIRQ( ADC_IRQn );
	NVIC_EnableIRQ(ADC_IRQn);
	NVIC_SetPriority(ADC_IRQn, 0x01 );
	
	ADC3->CR1 |= ADC_CR1_EOCIE;
	adc_buff_init();
}
