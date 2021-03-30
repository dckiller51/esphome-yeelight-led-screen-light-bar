/********************************************************************************************************
 * @file     qdec.h
 *
 * @brief    This is the head file for TLSR8231
 *
 * @author	 Telink
 * @date    May 24, 2019
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
 * 			 1.initial release(May. 24 2019)
 *
 * @version  A001
 *
 *******************************************************************************************************/
#pragma once

#include "clock.h"
#include "pm.h"
#include "analog.h"
#include "register.h"

typedef enum
{
	PD1A,
	PD3A,
	PC3A,
	PB3A,
	PB4A,
	PB5A,
	PA0A,
	PA1A,
}QDEC_InputAchTypeDef;

typedef enum
{
	PD1B,
	PD3B,
	PC3B,
	PB3B,
	PB4B,
	PB5B,
	PA0B,
	PA1B,
}QDEC_InputBchTypeDef;


typedef enum
{
	COMMON_MODE,
	DOUBLE_ACCURACY_MODE,
}QDEC_ModeTypeDef;



/**
 * @brief      This function servers to set input port.
 * @param[in]  QDEC_InputAchTypeDef - input types of A channel.
 * @param[in]  QDEC_InputBchTypeDef - input types of A channel.
 * @return     none.
 */
void qdec_set_pin(QDEC_InputAchTypeDef channelA,QDEC_InputBchTypeDef channelB);

/**
 * @brief      This function servers to set qdec's mode.
 * @param[in]  QDEC_ModeTypeDef - mode type to select.
 * @return     none.
 */
void qdec_set_mode(QDEC_ModeTypeDef mode);

/**
 * @brief      This function servers to initials qedc source clock.
 * @param[in]  none.
 * @return     none.
 */
void qdec_clk_en(void);

/**
 * @brief      This function servers to read hardware counting value.
 * @param[in]  none.
 * @return     hardware counting value.
 */
unsigned char qdec_get_count_value(void);

/**
 * @brief      This function servers to reset the counter and the QDEC Counter value is cleared zero.
 * @param[in]  none.
 * @return     none.
 */
void qdec_clear_conuter(void);

/**
 * @brief      This function servers to set hardware debouncing.
 * @param[in]  thrsh - lower the value of thrsh,will be regard as jitter.
 * @return     none.
 */
void qdec_set_debouncing(char thrsh);

/**
 * @brief      This function servers to set giop qdec fuction.
 * @param[in]  pin - the pin port selected as qdec interface pin port.
 * @param[in]  func-the function selected for pin.
 * @return     none.
 */
void gpio_set_qdec_func(GPIO_PinTypeDef pin, GPIO_FuncTypeDef func);
