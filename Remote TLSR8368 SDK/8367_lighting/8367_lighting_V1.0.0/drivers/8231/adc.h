/********************************************************************************************************
 * @file     adc.h
 *
 * @brief    This is the ADC driver header file for TLSR8231
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

#pragma once

#include "bsp.h"
#include "analog.h"
#include "register.h"
#include "gpio.h"

/**
 *  ADC reference voltage
 */
typedef enum{
	ADC_VREF_NONE   = 0,
	ADC_VREF_0P9V   = 1,
	ADC_VREF_1P2V   = 2,
	ADC_VREF_VBAT_N = 3,  //here N(1/2/3/4) is configed by interface adc_set_vbat_divider_atb
}ADC_RefVolTypeDef;

/**
 *  ADC Vbat divider
 */
typedef enum{
	ADC_VBAT_DIVIDER_OFF = 0,
	ADC_VBAT_DIVIDER_1F4,
	ADC_VBAT_DIVIDER_1F3,
	ADC_VBAT_DIVIDER_1F2
}ADC_VbatDivTypeDef;

typedef enum{
	ADC_SEL_ATB_NONE = 0,
	ADC_SEL_ATB_0,
	ADC_SEL_ATB_1,
	ADC_SEL_ATB_2
}ADC_SelAtbTypeDef;

typedef enum{
	ADC_SM_RESTART,
	ADC_SM_DISABLE,
} adc_sm_restart_t;

/**
 *	ADC analog input negative channel
 */
typedef enum {
	NOINPUTN,
	C7N,
	A7N,
	B0N,
	B1N,
	B2N,
	B3N,
	B4N,
	B5N,
	B6N,
	B7N,
	PGA0N,
	RSVD0_N,
	RSVD1_N,
	RSVD2_N,
	GND,
}ADC_InputNchTypeDef;

/**
 *	ADC analog input positive channel
 */
typedef enum {
	NOINPUTP,
	C7P,
	A7P,
	B0P,
	B1P,
	B2P,
	B3P,
	B4P,
	B5P,
	B6P,
	B7P,
	PGA0P,
	RSVD0_P,
	RSVD1_P,
	RSVD2_P,
	VBAT,
}ADC_InputPchTypeDef;

/**
 *	ADC resolution
 */
typedef enum{
	RES8,
	RES10,
	RES12,
	RES14
}ADC_ResTypeDef;

/**
 *	ADC channel input mode
 *  (in black hawk, only differential mode)
 */
typedef enum{
	DIFFERENTIAL_MODE = 1,  //differential mode
}ADC_InputModeTypeDef;

/**
 *  ADC Sampling cycles
 */
typedef enum{
	SAMPLING_CYCLES_3,
	SAMPLING_CYCLES_6,
	SAMPLING_CYCLES_9,
	SAMPLING_CYCLES_12,
	SAMPLING_CYCLES_15,
	SAMPLING_CYCLES_18,
	SAMPLING_CYCLES_21,
	SAMPLING_CYCLES_24,
	SAMPLING_CYCLES_27,
	SAMPLING_CYCLES_30,
	SAMPLING_CYCLES_33,
	SAMPLING_CYCLES_36,
	SAMPLING_CYCLES_39,
	SAMPLING_CYCLES_42,
	SAMPLING_CYCLES_45,
	SAMPLING_CYCLES_48,
}ADC_SampCycTypeDef;

/**
 * ADC input channel: Left/Right/MISC/RNS
 */
typedef enum{
	ADC_MISC_CHN	= BIT(2),
	ADC_RNS_CHN 	= BIT(3),
}ADC_ChTypeDef;

/**
 *	ADC Prescaler
 */
typedef enum{
	ADC_PRESCALER_1,
	ADC_PRESCALER_1F2,
	ADC_PRESCALER_1F4,
	ADC_PRESCALER_1F8
}ADC_PreScalingTypeDef;

/**
 *	ADC current trim
 */
typedef enum{
	 ADC_CUR_TRIM_PER75,
	 ADC_CUR_TRIM_PER100,
	 ADC_CUR_TRIM_PER125,
	 ADC_CUR_TRIM_PER150
}ADC_Cur_TrimTypeDef;

typedef enum{
	GAIN_STAGE_BIAS_PER75 = 0,
	GAIN_STAGE_BIAS_PER100,
	GAIN_STAGE_BIAS_PER125,
	GAIN_STAGE_BIAS_PER150,
}ADC_Gain_BiasTypeDef;

/**
 *	ADC mode: Normal mode/RNS mode
 */
typedef enum{
	NORMAL_MODE      = 0,
	RNS_MODE         = BIT(4),
}ADC_ModeTypeDef;

/**
 * ADC RNS channel source
 */
typedef enum {
	SAR_ADC_RNG_MODE   = 0,
	R_RNG_MODE_0       = 2,
	R_RNG_MODE_1       = 3,
	ADC_DAT12_RNG_MODE = 4,
	ADC_DAT5_RNG_MODE  = 6,
}ADC_RNS_SrcTypeDef;

typedef enum {
	READ_UPDATA        = BIT(3),
	CLOCLK_UPDATA      = BIT(4),
}ADC_RNS_UpdateTypeDef;

/**
 * @brief      This function sets sar_adc power.
 * @param[in]  on - 1 : power on; 0 : power off.
 * @return     none
 */
static inline void adc_power_on(unsigned char on_off)
{
	analog_write (anareg_adc_pga_ctrl, (analog_read(anareg_adc_pga_ctrl)&(~FLD_SAR_ADC_POWER_DOWN)) | (!on_off)<<5  );
}

/**
 * @brief      This function reset adc module
 * @param[in]  none.
 * @return     none.
 */
static inline void	adc_reset(void)
{
	reg_rst2 = FLD_RST2_ADC;
	reg_rst2 = 0;
}

/**
 * @brief      This function reset aif module
 * @param[in]  none.
 * @return     none.
 */
static inline void	aif_reset(void)
{
	reg_rst0= FLD_RST0_AIF;
	reg_rst0=0;
}

/**
 * @brief      This function enable adc source clock: external 24M
 * @param[in]  en - variable of source clock state 1: enable;  0: disable.
 * @return     none.
 */
static inline void adc_clk_en(unsigned int en)
{
	if(en)
	{
		analog_write(anareg_80, analog_read(anareg_80) | FLD_CLK_24M_TO_SAR_EN);
	}
	else{
		analog_write(anareg_80, analog_read(anareg_80) & ~FLD_CLK_24M_TO_SAR_EN);
	}
}

/**
 * @brief      This function sets adc sample clk. adc sample clk = 24M/(1+div)  div: 0~7.
 * @param[in]  div - the divider of adc sample clock.
 * @return     none
 */
static inline void adc_set_clk_div(unsigned char div)
{
	analog_write(anareg_adc_clk_div, (analog_read(anareg_adc_clk_div)&(~FLD_ADC_CLK_DIV)) | (div & 0x07) );
}

/**
 * @brief      This function service to get the data of adc MISC.
 * @param[in]  none.
 * @return     The result of adc MISC data.
 */
static inline unsigned short adc_get_misc_data(void){
	return analog_read(anareg_adc_misc_l) + (analog_read(anareg_adc_misc_h)<<8);
}

/**
 * @brief      This function sets ADC reference voltage for the MISC channel
 * @param[in]  v_ref - enum variable of adc reference voltage.
 * @return     none
 */
static inline void adc_set_misc_vref(ADC_RefVolTypeDef v_ref)
{
	analog_write(anareg_adc_vref, ((analog_read(anareg_adc_vref)&(~FLD_ADC_VREF_CHN_M)) | (v_ref<<4)) );
}

/**
 * @brief      This function service to set adc vabt divider and select ATB.
 * @param[in]  vbat_div - enum variable of Vbat voltage divider.
 * @param[in]  stat     - enum variable of ATB.
 * @return     none
 */
static inline void adc_set_vbat_divider_atb(ADC_VbatDivTypeDef vbat_div,ADC_SelAtbTypeDef stat)
{
	unsigned char value_h;
	value_h = ((analog_read(anareg_adc_vref_vbat_div)&(~FLD_ADC_SEL_ATB)) | (stat<<4));
	analog_write (anareg_adc_vref_vbat_div, ((analog_read(anareg_adc_vref_vbat_div)&(~FLD_ADC_VREF_VBAT_DIV)) | (vbat_div<<2)) | value_h);
}

/**
 * @brief      This function sets ADC analog negative & positive input channel for the MISC channel
 * @param[in]  v_ain - enum variable of ADC analog negative input.
 * @param[in]  v_ain - enum variable of ADC analog positive input.
 * @return     none
 */
static inline void adc_set_channel(ADC_InputNchTypeDef n_chn, ADC_InputPchTypeDef p_chn){
	analog_write(anareg_adc_ain_chn_misc, n_chn+(p_chn<<4));
}

/**
 * @brief      This function serves to set input channel in differential_mode.
 * @param[in]  ch_n - enum variable of ADC input channel.
 * @param[in]  InPCH - enum variable of ADC analog positive input channel.
 * @param[in]  InNCH - enum variable of ADC analog negative input channel.
 * @return     none
 */
void adc_set_all_differential_p_n_ain(ADC_ChTypeDef ch_n, ADC_InputPchTypeDef InPCH,ADC_InputNchTypeDef InNCH);

/**
 * @brief     This function serves to set pre_scaling.
 * @param[in] v_scl - enum variable of ADC pre_scaling factor.
 * @return    none
 */
void adc_set_all_ain_pre_scaler(ADC_PreScalingTypeDef v_scl);

/**
 * @brief      This function sets ADC resolution for the MISC channel and the input mode.
 * @param[in]  v_res - enum variable of ADC resolution.
 * @return     none
 */
static inline void adc_set_misc_resolution_input_mode(ADC_ResTypeDef v_res,ADC_InputModeTypeDef m_input,ADC_ChTypeDef ch_n)
{
	unsigned char value_six;
	if(m_input){
		value_six = analog_read(anareg_adc_res_m) | FLD_ADC_EN_DIFF_CHN_M;
	}
	if(ch_n & ADC_MISC_CHN)
	{
		analog_write(anareg_adc_res_m, ((analog_read(anareg_adc_res_m)&(~FLD_ADC_RES_M)) | (v_res)) | value_six );
	}
}

/**
 * @brief      This function sets ADC sample time(the number of adc clocks for each sample)
 * @param[in]  adcST - enum variable of adc sample time.
 * @return     none
 */
static inline void adc_set_misc_tsample_cycle(ADC_SampCycTypeDef adcST)
{
	analog_write(anareg_adc_tsmaple_m, (analog_read(anareg_adc_tsmaple_m)&(~FLD_ADC_TSAMPLE_CYCLE_CHN_M)) | (adcST) );
}

/**
 * @brief     This function serves to set sample_cycle.
 * @param[in] ch_n - the enum variable of ADC input channel.
 * @param[in] adcST - the enum variable of ADC Sampling cycles.
 * @return    none
 */
void adc_set_all_tsample_cycle(ADC_ChTypeDef ch_n, ADC_SampCycTypeDef adcST);

/**
 * @brief      This function sets length of set state for MISC channel.
 * @param[in]  r_max_s - variable of length of "set" state for MISC channel.
 * @return     none
 */
static inline void adc_set_all_set_state_length(unsigned char r_max_s)
{
	analog_write(anareg_r_max_s, (analog_read(anareg_r_max_s)&(~FLD_R_MAX_S)) | (r_max_s) );
}

/**
 * @brief      This function sets length of each captures for MISC channel.
 * @param[in]  r_max_mc - variable of length of "set" state for MISC channel.
 * @return     none
 */
static inline void adc_set_misc_rns_capture_state_length(unsigned short r_max_mc)
{
	analog_write(anareg_r_max_mc,  (r_max_mc & 0x0ff));
	analog_write(anareg_r_max_s,  ((analog_read(anareg_r_max_s)&(~FLD_R_MAX_MC1)) | (r_max_mc>>8)<<6 ));
}

/**
 * @brief      This function serves to set state length.
 * @param[in]  R_max_mc - Value of length of "capture" state for RNS & MISC channel.
 * @param[in]  R_max_c - Value of length of "capture" state.
 * @param[in]  R_max_s - Value of length of "capture" state.
 * @return     none
 */
void adc_set_all_set_and_capture_state_length(unsigned short R_max_mc, unsigned short R_max_c,unsigned char R_max_s);

/**
 * @brief      This function sets ADC input channel and states counts.
 * @param[in]  ad_ch - enum variable of ADC input channel.
 * @param[in]  s_cnt - sum of state index start with 0x0.
 * @return     none
 */
static inline void adc_set_chn_en_cnt(ADC_ChTypeDef ad_ch,unsigned char s_cnt)
{
	unsigned char val_h;
	val_h = (analog_read(anareg_adc_chn_en)&(~FLD_ADC_MAX_SCNT)) | ((s_cnt&0x07)<<4);
	analog_write(anareg_adc_chn_en, ((analog_read(anareg_adc_chn_en)&0xf0) | ad_ch) | val_h);
}

/**
 * @brief      This function serves to set mode for ADC.
 * @param[in]  adc_m - 0: normal mode; 1: RNS mode.
 * @return     none
 */
static inline void adc_set_mode(ADC_ModeTypeDef adc_m)
{
	analog_write (anareg_adc_pga_ctrl, (analog_read(anareg_adc_pga_ctrl)&(~FLD_ADC_MODE)) | adc_m);
}

/**
 * @brief      This function serves to set the source and mode of the random number generator.
 * @param[in]  stat - the value of ADC RNS channel source.
 * @param[in]  stat1 - ADC_RNS_UpdateTypeDef update_type.
 * @return none.
 */
static inline void adc_set_rns(ADC_RNS_SrcTypeDef stat,ADC_RNS_UpdateTypeDef stat1){

	unsigned datast = stat|stat1;
	analog_write(0x80+126,datast);			//Set
}

/**
 * @brief     This function serves to read the value of the random number generator.
 * @param[in] none.
 * @return    unsigned short RngValue random number.
 */
static inline unsigned short adc_get_rns_result(void){

	unsigned short tmp1,tmp2,RngValue;
	tmp1 = analog_read(0x80+118);  //read
	tmp2 = analog_read(0x80+117);
	RngValue = (tmp1<<8) + tmp2;
	return RngValue;
}

/**
 * @brief     This function serves to ADC init.
 * @param[in] none
 * @return    none
 */
void adc_init(void);

/**
 * @brief     This function is used for IO port configuration of ADC supply voltage sampling.
 * @param[in] GPIO_PinTypeDef pin
 * @return    none
 */
void adc_vbat_pin_init(GPIO_PinTypeDef pin);

/**
 * @brief     This function is used for IO port configuration of ADC IO port voltage sampling.
 * @param[in] GPIO_PinTypeDef pin
 * @return    none
 */
void adc_base_init(GPIO_PinTypeDef pin);

/**
 * @brief     This function is used for ADC configuration of ADC supply voltage sampling.
 * @param[in] GPIO_PinTypeDef pin
 * @return    none
 */
void adc_vbat_init(GPIO_PinTypeDef pin);

/**
 * @brief     This function is used for ADC configuration of ADC supply voltage sampling.especially for RNG mode.
 * @param[in] GPIO_PinTypeDef pin
 * @return    none
 */
void adc_vbat_init_rng(GPIO_PinTypeDef pin);

/**
 * @brief     This function serves to set adc sampling and get results.
 * @param[in] none.
 * @return    the result of sampling.
 */
unsigned int adc_set_sample_and_get_result(void);

/**
 * @brief      This function performs to set MISC channel.
 * @param[in]  pbuff - address in FIFO2.
 * @param[in]  size_buff - depth of FIFO2.
 * @return     none.
 */
static inline void adc_aif_set_misc_buf(unsigned short* pbuff,unsigned int size_buff)
{
	reg_dfifo_misc_chn_addr = (unsigned short)((unsigned int)pbuff);
	reg_dfifo_misc_chn_size = (size_buff>>3)-1;
	reg_aif_m_chn_wptr = 0;  //clear dfifo2 write pointer
}
/**
 * @brief      This function performs to enable audio input of DFIFO2.
 * @param[in]  en - the value 1 - enable.0 - disable.
 * @return     none.
 */
static inline void adc_aif_set_m_chn_en(unsigned char en)
{
	if(en)
	{
		reg_aif_m_chn_ctrl |= (FLD_M_CHANNEL_FIFO_EN|FLD_M_CHANNEL_WPTR_EN|FLD_M_CHANNEL_MONO);///FLD_M_CHANNEL_MONO:in short, in word(4byte)
	}
	else
	{
		reg_aif_m_chn_ctrl &= ~(FLD_M_CHANNEL_FIFO_EN|FLD_M_CHANNEL_WPTR_EN);
	}
}

/**
 * @brief      This function servers to set ADC aif data enable.
 * @param[in]  none.
 * @return     none.
 */
static inline void adc_aif_set_use_raw_data_en(void){
	reg_aif_adc_ctrl |= FLD_USE_RAW_DATA;
}

/**
 * @brief       This function serves to set the channel reference voltage.
 * @param[in]   ch_n - enum variable of ADC input channel.
 * @param[in]   v_ref - enum variable of ADC reference voltage.
 * @return none
 */
void adc_set_all_vref(ADC_ChTypeDef ch_n, ADC_RefVolTypeDef v_ref);

/**
 * @brief      This function servers to set MISC channel buffer address and size.
 * @param[in]  pbuff - the enum variable of data buffer.
 * @param[in]  size_buff - the enum variable of data size.
 * @return     none
 */
static inline void adc_config_misc_channel_buf(signed short* pbuff,unsigned int size_buff)
{
	reg_dfifo_misc_chn_addr = (unsigned short)((unsigned int)pbuff);
	reg_dfifo_misc_chn_size = (size_buff>>4)-1;

	BM_SET(reg_dfifo_config,FLD_AUD_DFIFO_EN|FLD_AUD_WPTR_EN|FLD_AUD_DATA_BIT_WIDTH);
}

/** \defgroup GP1  ADC Examples
 *
 * 	@{
 */

/*! \page adc Table of Contents
	- [API-ADC-CASE1:ADC BASE MODE](#ADC_BASE_MODE)
	- [API-ADC-CASE2:ADC VBAT MODE](#ADC_VBAT_MODE)
	- [API-ADC-CASE3:ADC RNG MODE](#ADC_RNG_MODE)
	- [API-ADC-CASE4:ADC TEMP MODE](#ADC_TEMP_MODE)

<h1 id=ADC_BASE_MODE> API-ADC-CASE1:ADC BASE MODE </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | None ||| Interrupt handler function [**Mandatory**] | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | adc_init()|| initiate ADC module  | ^ |
| ^ | ^ | adc_base_init()| adc_base_init(GPIO_PB0)| initiate ADC module in the BASE mode | ^ |
| ^ | ^ | adc_power_on() | adc_power_on(1) | Power on ADC module | ^ |
| ^ | main_loop() | base_val = adc_set_sample_and_get_result() || get the result in main program loop | ^ |

<h1 id=ADC_VBAT_MODE> API-ADC-CASE2:ADC VBAT MODE </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | None ||| Interrupt handler function [**Mandatory**] | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | adc_init()|| initiate ADC module  | ^ |
| ^ | ^ | adc_vbat_init()| adc_vbat_init(GPIO_PB0)| initiate ADC module in the BASE mode | ^ |
| ^ | ^ | adc_power_on() | adc_power_on(1) | Power on ADC module | ^ |
| ^ | main_loop() | vbat_val = adc_set_sample_and_get_result() || get the result in main program loop | ^ |

<h1 id=ADC_RNG_MODE> API-ADC-CASE3:ADC RNG MODE </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | None ||| Interrupt handler function [**Mandatory**] | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | rng_init()|| initiate RNG  | ^ |
| ^ | main_loop() | rns_val = rand() || get the result in main program loop | ^ |

<h1 id=ADC_TEMP_MODE> API-ADC-CASE4:ADC TEMP MODE </h1>

<h1> History Record </h1>

| Date | Description | Author |
| :--- | :---------- | :----- |
| 2019-1-10 | initial release | ZJY/LJW |


*/

 /** @}*/ //end of GP1



















