#include "dac.h"
#include "arm_math.h"
//#include "math_helper.h"


struct DAC_BUFF dac_buff;

/// For HAL drivers
DAC_HandleTypeDef hdac;

/// For proper DAC sampling frequency
extern TIM_HandleTypeDef htim2 ;



// FIR properties:
#define NUM_TAPS 73
static double reverseFIRtabs[NUM_TAPS] = { 
-0.079972f, -0.052498f, 0.077378f, 0.001472f, 0.024929f, 0.021913f, -0.000833f,
0.025820f, -0.011167f, 0.022260f, -0.011762f, 0.015442f, -0.006030f, 0.007370f,
0.003099f, -0.001323f, 0.012813f, -0.009883f, 0.019657f, -0.017006f, 0.020517f,
-0.021668f, 0.012300f, -0.022428f, -0.005672f, -0.018692f, -0.032674f, -0.010907f,
-0.065558f, -0.000399f, -0.099585f, 0.010666f, -0.129362f, 0.019838f, -0.149651f,
0.025073f, 0.843147f, 0.025073f, -0.149651f, 0.019838f, -0.129362f, 0.010666f, 
-0.099585f, -0.000399f, -0.065558f, -0.010907f, -0.032674f, -0.018692f, -0.005672f,
-0.022428f, 0.012300f, -0.021668f, 0.020517f, -0.017006f, 0.019657f, -0.009883f, 
0.012813f, -0.001323f, 0.003099f, 0.007370f, -0.006030f, 0.015442f, -0.011762f,
0.022260f, -0.011167f, 0.025820f, -0.000833f, 0.021913f, 0.024929f, 0.001472f,
0.077378f, -0.052498f, -0.079972f 
};


/*
#define NUM_TAPS 33
static float32_t reverseFIRtabs[NUM_TAPS] = {
	-0.1412439347965169f, 	-0.07655592872244435f, 	0.058953566613785535f, 	-0.04463578556661513f,
	0.03263402942737146f, 	-0.020970723409016102f,	0.00879470619417877f,	0.004103636301491229f,
	-0.018179600521947906f, 	0.033401843502147345f, 	-0.04901508280002529f, 	0.0642438203548715f,
	-0.07838018026049243f, 	0.0905414997331043f, 	-0.09991894640255063f,	0.10591689831632427f,
	0.8919963276796102f, 	0.10591689831632427f, 	-0.09991894640255063f, 	0.0905414997331043f,
	-0.07838018026049243f, 	0.0642438203548715f, 	-0.04901508280002529f, 	0.03340184350214733f,
	-0.018179600521947906f,  	0.004103636301491229f,	0.008794706194178784f, 	-0.020970723409016102f,
	0.03263402942737146f, 	-0.04463578556661513f, 	0.05895356661378551f, 	-0.07655592872244435f,
	-0.1412439347965169f
};
*/

/*
#define NUM_TAPS 45
static float32_t reverseFIRtabs[NUM_TAPS] = {  -0.063427f, 0.169791f, 0.948440f,
0.364134f, -0.677113f, 0.083894f, -0.240385f, -0.322288f, 0.026128f, -0.459502f,
0.034045f, -0.407411f, -0.104666f, -0.257717f, -0.315474f, -0.059806f, -0.553790f,
0.147636f, -0.776610f, 0.322792f, -0.935617f, 0.423466f, 5.316944f, 0.423466f,
-0.935617f, 0.322792f, -0.776610f, 0.147636f, -0.553790f, -0.059806f, -0.315474f,
-0.257717f, -0.104666f, -0.407411f, 0.034045f, -0.459502f, 0.026128f, -0.322288f,
-0.240385f, 0.083894f, -0.677113f, 0.364134f, 0.948440f, 0.169791f, -0.063427f
};
*/
#define BLOCK_SIZE 2
// output buffer
// static float32_t testOutput[BLOCK_SIZE];
// Declare State buffer of size (numTaps + blockSize - 1)
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];

// Describe a filter
arm_fir_instance_f32 S_FIR;

// FIR functons:
void fir_init( void ) {
	  arm_fir_init_f32(&S_FIR, NUM_TAPS, (float32_t *)&reverseFIRtabs[0], &firStateF32[0], BLOCK_SIZE);
}




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
	
	//FIR variables
	static float32_t inputfir[2] ;
	static float32_t outputfir[2] ;
	static float32_t *p_inputfir ;
	static float32_t *p_outputfir ;
	
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
		// If not then filter data
		else {
			// Filter samples before they will save to DAC buffer
			inputfir[0]  = (float32_t) (((*(dac_buff.p_buff - 2))*3.0f)/4096.0f);
			inputfir[1]  = (float32_t) (((*(dac_buff.p_buff - 1))*3.0f)/4096.0f);
			
			p_inputfir = (float32_t *)&inputfir[0] ;
			p_outputfir = &outputfir[0];
			
			arm_fir_f32( &S_FIR, p_inputfir, p_outputfir, 2 ) ;
			
			*(dac_buff.p_buff -2) = (uint16_t) ((outputfir[0] * 4096.0f)/3.0f) ;
			*(dac_buff.p_buff -1) = (uint16_t) ((outputfir[1] * 4096.0f)/3.0f) ;
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
	htim2.Instance->CR1 |= TIM_CR1_CEN;
}
void dac_OFF(void){
	htim2.Instance->CR1 &= ~TIM_CR1_CEN;
}
