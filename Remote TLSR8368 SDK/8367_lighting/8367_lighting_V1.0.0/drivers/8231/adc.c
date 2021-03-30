/********************************************************************************************************
 * @file     adc.c
 *
 * @brief    This is the ADC driver file for TLSR8231
 *
 * @author   Telink
 * @date     May 12, 2019
 *
 * @par      Copyright (c) 2018, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *           The information contained herein is confidential property of Telink
 *           Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *           of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *           Co., Ltd. and the licensee or the terms described here-in. This heading
 *           MUST NOT be removed from this file.
 *
 *           Licensees are granted free, non-transferable use of the information in this
 *           file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 * @par      History:
 * 			 1.initial release(DEC. 26 2018)
 *
 * @version  A001
 *
 *******************************************************************************************************/

#include "adc.h"
#include "analog.h"
#include "clock.h"
#include "timer.h"

volatile unsigned short	adc_code;

unsigned char ADC_Vref = 0;     		//ADC  Vref
unsigned char ADC_Pre_Scale = 1;		//ADC VBAT scale
unsigned char ADC_VBAT_Scale = 0;		//ADC pre scale

const unsigned char  Vref_tab[4] = {2,3,4,1};
const unsigned char  VBAT_Scale_tab[4] = {1,4,3,2};

#define    MISC_NUM          16

unsigned int Vbat = 0;

volatile signed short misc_dat_buf[MISC_NUM] = {0}; //must 16 byte
unsigned int Volt_B0 = 0;

GPIO_PinTypeDef ADC_GPIO_tab[10] = {

		GPIO_PC7,GPIO_PA7,
		GPIO_PB0,GPIO_PB1,
		GPIO_PB2,GPIO_PB3,
		GPIO_PB4,GPIO_PB5,
		GPIO_PB6,GPIO_PB7,

};

/**
 * @brief This function serves to ADC init.
 * @param[in]   none
 * @return none
 */
void adc_init(void ){

	/****** sar adc Reset ********/
	//reset whole digital adc module
	reg_rst2 = FLD_RST2_ADC;
	reg_rst2 = 0;

	/******power on sar adc********/
	//adc_power_on(1);

	/******enable signal of 24M clock to sar adc********/
	adc_clk_en(1);

	/******set adc clk as 4MHz******/
	adc_set_clk_div(5);

	/*********	Optimized the problem of ADC low temperature jitter *************/
	adc_set_vbat_divider_atb(ADC_VBAT_DIVIDER_OFF,ADC_SEL_ATB_1);

}

/**
 * @brief     This function is used for IO port configuration of ADC IO port voltage sampling.
 * @param[in] GPIO_PinTypeDef pin
 * @return    none
 */
void adc_base_pin_init(GPIO_PinTypeDef pin)
{
	//ADC GPIO Init
	gpio_set_func(pin, AS_GPIO);
	gpio_set_input_en(pin,0);
	gpio_set_output_en(pin,0);
	gpio_write(pin,0);
}

/**
 * @brief     This function is used for IO port configuration of ADC supply voltage sampling.
 * @param[in] GPIO_PinTypeDef pin
 * @return    none
 */
void adc_vbat_pin_init(GPIO_PinTypeDef pin)
{
	gpio_set_func(pin, AS_GPIO);
	gpio_set_input_en(pin,0);
	gpio_set_output_en(pin,1);
	gpio_write(pin,1);
}

/**
 * @brief     This function is used for IO port configuration of ADC IO port voltage sampling.
 * @param[in] GPIO_PinTypeDef pin
 * @return    none
 */
volatile unsigned char mode_select=0;
void adc_base_init(GPIO_PinTypeDef pin)
{
	unsigned char i,j;
	unsigned char gpio_no=0;
	mode_select=1;
	//set R_max_mc,R_max_c,R_max_s
	adc_set_misc_rns_capture_state_length(0xf0);						//max_mc
	adc_set_all_set_state_length(0x0a);									//max_s

	//set total length for sampling state machine and channel
	adc_set_chn_en_cnt(ADC_MISC_CHN,0x02);

	//set channel Vref
	adc_set_all_vref(ADC_MISC_CHN, ADC_VREF_1P2V);
	ADC_Vref = (unsigned char)ADC_VREF_1P2V;
	//set Vbat divider select,
	adc_set_vbat_divider_atb(ADC_VBAT_DIVIDER_OFF,ADC_SEL_ATB_1);

	ADC_VBAT_Scale = VBAT_Scale_tab[ADC_VBAT_DIVIDER_OFF];

	//set resolution channel mode and channel for normal
	adc_set_misc_resolution_input_mode(RES14,DIFFERENTIAL_MODE,ADC_MISC_CHN);

	//set normal mode
	adc_set_mode(NORMAL_MODE);

	for(i=0;i<10;i++)
	{
		if(pin == ADC_GPIO_tab[i])
		{
			gpio_no = i+1;
			break;
		}
	}
	adc_set_all_differential_p_n_ain(ADC_MISC_CHN, gpio_no, GND);

	//Number of ADC clock cycles in sampling phase
	adc_set_all_tsample_cycle(ADC_MISC_CHN, SAMPLING_CYCLES_6);
	//set Analog input pre-scaling and
	adc_set_all_ain_pre_scaler(ADC_PRESCALER_1F8);
	ADC_Pre_Scale = 1<<(unsigned char)ADC_PRESCALER_1F8;

	for(j = 0;j<MISC_NUM;j++)
		misc_dat_buf[j] = 0x5555;

	//use dfifo mode to get adc sample data
	adc_config_misc_channel_buf(misc_dat_buf, sizeof(misc_dat_buf));
}

/**
 * @brief     This function is used for ADC configuration of ADC supply voltage sampling.
 * @param[in] GPIO_PinTypeDef pin
 * @return    none
 */
void adc_vbat_init(GPIO_PinTypeDef pin)
{
	unsigned char i,j;
	unsigned char gpio_no=0;
	mode_select=2;
	//set R_max_mc,R_max_c,R_max_s
	adc_set_misc_rns_capture_state_length(0xf0);						//max_mc
	adc_set_all_set_state_length(0x0a);									//max_s

	//set total length for sampling state machine and channel
	adc_set_chn_en_cnt(ADC_MISC_CHN,0x02);

	//set channel Vref
	adc_set_all_vref(ADC_MISC_CHN, ADC_VREF_1P2V);
	ADC_Vref = (unsigned char)ADC_VREF_1P2V;
	//set Vbat divider select,
	adc_set_vbat_divider_atb(ADC_VBAT_DIVIDER_OFF,ADC_SEL_ATB_1);
	ADC_VBAT_Scale = VBAT_Scale_tab[ADC_VBAT_DIVIDER_OFF];

	//set resolution channel mode and channel for normal
	adc_set_misc_resolution_input_mode(RES14,DIFFERENTIAL_MODE,ADC_MISC_CHN);

	adc_vbat_pin_init(pin);
	for(i=0;i<10;i++)
	{
		if(pin == ADC_GPIO_tab[i])
		{
			gpio_no = i+1;
			break;
		}

	}
	adc_set_all_differential_p_n_ain(ADC_MISC_CHN, gpio_no, GND);

	//Number of ADC clock cycles in sampling phase
	adc_set_all_tsample_cycle(ADC_MISC_CHN, SAMPLING_CYCLES_6);
	//set Analog input pre-scaling and
	adc_set_all_ain_pre_scaler(ADC_PRESCALER_1F8);
	ADC_Pre_Scale = 1<<(unsigned char)ADC_PRESCALER_1F8;

	//set normal mode
	adc_set_mode(NORMAL_MODE);

	for(j = 0;j<MISC_NUM;j++)
		misc_dat_buf[j] = 0x5555;

	//use dfifo mode to get adc sample data
	adc_config_misc_channel_buf(misc_dat_buf, sizeof(misc_dat_buf));

}

/**
 * @brief     This function is used for ADC configuration of ADC supply voltage sampling.especially for RNG mode.
 * @param[in] GPIO_PinTypeDef pin
 * @return    none
 */
void adc_vbat_init_rng(GPIO_PinTypeDef pin)
{
	unsigned char i;
	unsigned char gpio_no=0;

	//set R_max_mc,R_max_c,R_max_s
	adc_set_misc_rns_capture_state_length(0xf0);		 //max_mc
	adc_set_all_set_state_length(0x0a);									//max_s

	//set total length for sampling state machine and channel
	adc_set_chn_en_cnt(ADC_MISC_CHN,0x02);

	//set channel Vref
	adc_set_all_vref(ADC_MISC_CHN, ADC_VREF_1P2V);

	//set Vbat divider select,
	adc_set_vbat_divider_atb(ADC_VBAT_DIVIDER_OFF,ADC_SEL_ATB_1);
	ADC_VBAT_Scale = VBAT_Scale_tab[ADC_VBAT_DIVIDER_OFF];

	//set channel mode and channel
	adc_vbat_pin_init(pin);
	for(i=0;i<10;i++)
	{
		if(pin == ADC_GPIO_tab[i])
		{
			gpio_no = i+1;
			break;
		}
	}

	//set resolution channel mode and channel for normal
	adc_set_misc_resolution_input_mode(RES14,DIFFERENTIAL_MODE,ADC_MISC_CHN);
	adc_set_all_differential_p_n_ain(ADC_MISC_CHN, gpio_no, GND);
	//Number of ADC clock cycles in sampling phase
	adc_set_all_tsample_cycle(ADC_MISC_CHN, SAMPLING_CYCLES_6);

	//set Analog input pre-scaling and
	adc_set_all_ain_pre_scaler(ADC_PRESCALER_1F8);

	//set RNG mode
	adc_set_mode(NORMAL_MODE);

}

/**
 * @brief       This function serves to set the channel reference voltage.
 * @param[in]   ch_n - enum variable of ADC input channel.
 * @param[in]   v_ref - enum variable of ADC reference voltage.
 * @return none
 */
void adc_set_all_vref(ADC_ChTypeDef ch_n, ADC_RefVolTypeDef v_ref)
{
	if(ch_n & ADC_MISC_CHN)
	{
		adc_set_misc_vref(v_ref);
	}

	if(v_ref == ADC_VREF_1P2V)
	{
		analog_write(anareg_ain_scale, (analog_read(anareg_ain_scale)&(0xC0)) | 0x3d );
	}
	else
	{
		analog_write(anareg_ain_scale, (analog_read(anareg_ain_scale)&(0xC0)) | 0x15 );
	}

}

/**
 * @brief     This function serves to set sample_cycle.
 * @param[in] ch_n - the enum variable of ADC input channel.
 * @param[in] adcST - the enum variable of ADC Sampling cycles.
 * @return    none
 */
void adc_set_all_tsample_cycle(ADC_ChTypeDef ch_n, ADC_SampCycTypeDef adcST)
{
	if(ch_n & ADC_MISC_CHN)
	{
		adc_set_misc_tsample_cycle(adcST);
	}
}

/**
 * @brief      This function serves to set input channel in differential_mode.
 * @param[in]  ch_n - enum variable of ADC input channel.
 * @param[in]  InPCH - enum variable of ADC analog positive input channel.
 * @param[in]  InNCH - enum variable of ADC analog negative input channel.
 * @return     none
 */
void adc_set_all_differential_p_n_ain(ADC_ChTypeDef ch_n, ADC_InputPchTypeDef InPCH,ADC_InputNchTypeDef InNCH)
{
	if(ch_n & ADC_MISC_CHN)
	{
		adc_set_channel(InNCH, InPCH);
	}
}

/**
 * @brief      This function serves to set state length.
 * @param[in]  R_max_mc - Value of length of "capture" state for RNS & MISC channel.
 * @param[in]  R_max_c - Value of length of "capture" state.
 * @param[in]  R_max_s - Value of length of "capture" state.
 * @return     none
 */
void adc_set_all_set_and_capture_state_length(unsigned short R_max_mc, unsigned short R_max_c,unsigned char R_max_s)
{
	unsigned char data[3]={0};
	if(R_max_mc&0x3ff)    //r_max_mc[9:0]serves to set length of state for RNS and Misc channel.
	{
		data[0] = (unsigned char)R_max_mc;
		data[2] = (unsigned char)(R_max_mc>>2)&0xc0;
	}
	if(R_max_c&0x3ff)     //r_max_c*9:0+ serves to set length of  state for left and right channel.
	{
		data[1] = (unsigned char)R_max_c;
		data[2] |= (unsigned char)(R_max_c>>4)&0x30;
	}
	if(R_max_s)     //r_max_s serves to set length of  state for left, right and Misc channel.
	{
		data[2] |= (unsigned char)(R_max_s&0x0f);
	}

	analog_write(anareg_r_max_mc, data[0]);
	analog_write(anareg_r_max_s,  data[2]);
}

/**
 * @brief     This function serves to set pre_scaling.
 * @param[in] v_scl - enum variable of ADC pre_scaling factor.
 * @return    none
 */
void adc_set_all_ain_pre_scaler(ADC_PreScalingTypeDef v_scl)
{
	analog_write(anareg_ain_scale, (analog_read(anareg_ain_scale)&(~FLD_SEL_AIN_SCALE)) | (v_scl<<6) );

	//setting adc_sel_atb ,if stat is 0,clear adc_sel_atb,else set adc_sel_atb[0]if(stat)
	unsigned char tmp;
	if(v_scl)
	{
		//ana_F9<4> must be 1
		tmp = analog_read(0xF9);
		tmp = tmp|0x10;                    //open tmp = tmp|0x10;
		analog_write (0xF9, tmp);
	}
	else
	{
		//ana_F9 <4> <5> must be 0
		tmp = analog_read(0xF9);
		tmp = tmp&0xef;
		analog_write (0xF9, tmp);
	}
}

/**
 * @brief     This function serves to set adc sampling and get results.
 * @param[in] none.
 * @return    the result of sampling.
 */
volatile signed short dat_buf[MISC_NUM];

_attribute_ram_code_ unsigned int adc_set_sample_and_get_result(void)
{

	unsigned short temp;
	unsigned short adc_sample[MISC_NUM] = {0};
	int i,j;

	adc_reset();
	aif_reset();

	unsigned int t0 = get_sys_tick();

	while(!timeout_us(t0, 25));                       //wait at least 2 sample cycle(f = 96K, T = 10.4us)

	//dfifo setting will lose in suspend/deep, so we need config it every time
	adc_aif_set_misc_buf((unsigned short *)dat_buf,MISC_NUM);  //size: ADC_SAMPLE_NUM*4
	adc_aif_set_m_chn_en(1);
	adc_aif_set_use_raw_data_en();

    // get adc sample data and sort these data
	for(i=0;i<MISC_NUM;i++){
		while((!dat_buf[i])&&(!timeout_us(t0,20)));  //wait for new adc sample data,
													 //When the data is not zero or more than 1.5 sampling times (when the data is zero),The default data is already ready.
		t0 = get_sys_tick();

		if(dat_buf[i] & BIT(13)){                    //14 bit resolution, BIT(13) is sign bit, 1 means negative voltage in differential_mode
			adc_sample[i] = 0;
		}
		else{
			adc_sample[i] = ((unsigned short)dat_buf[i] & 0x1FFF);  //BIT(12..0) is valid adc result
		}

		//insert sort
		if(i){
			if(adc_sample[i] < adc_sample[i-1]){
				temp = adc_sample[i];
				adc_sample[i] = adc_sample[i-1];
				for(j=i-1;j>=0 && adc_sample[j] > temp;j--){
					adc_sample[j+1] = adc_sample[j];
				}
				adc_sample[j+1] = temp;
			}
		}
	}

	adc_aif_set_m_chn_en(0);//misc channel data dfifo disable
	                        //get average value from raw data(abandon some small and big data ), then filter with history data.
	#if (MISC_NUM == 4)  	//use middle 2 data (index: 1,2)
		unsigned int adc_average = (adc_sample[1] + adc_sample[2])/2;
	#elif(MISC_NUM == 16) 	//use middle 4 data (index: 2,3,4,5)
		unsigned int adc_average = (adc_sample[4] + adc_sample[5] + adc_sample[6] + adc_sample[7] + adc_sample[8] + adc_sample[9] + adc_sample[10] + adc_sample[11])/8;
	#endif

	adc_code = adc_average;

	if(mode_select==1){
		if(adc_average>0x8000)
		{
				Volt_B0 = (0x10000-adc_code)*ADC_Pre_Scale*300*Vref_tab[ADC_Vref]/0x2000*63/64;
		}
		else
		{
				Volt_B0 = adc_code*ADC_Pre_Scale*300*Vref_tab[ADC_Vref]/0x2000*63/64;
		}
		return Volt_B0;
	}

	if(mode_select==2){
		if(ADC_VBAT_Scale == 1)
		{
			if(adc_average>0x2000)
			{
				Vbat = (0x8000-adc_code)*300*Vref_tab[ADC_Vref]*ADC_Pre_Scale/0x2000*63/64;
			}
			else
			{
				Vbat = adc_code*300*Vref_tab[ADC_Vref]*ADC_Pre_Scale/0x2000*63/64;
			}
		}
		else
		{
			if(adc_average>0x2000)
			{
				Vbat = (0x8000-adc_code)*300*Vref_tab[ADC_Vref]*ADC_VBAT_Scale/0x2000*63/64;
			}
			else
			{
				Vbat = adc_code*300*Vref_tab[ADC_Vref]*ADC_VBAT_Scale/0x2000*63/64;
			}
		}
		return Vbat;
	}
	return 0;
}



