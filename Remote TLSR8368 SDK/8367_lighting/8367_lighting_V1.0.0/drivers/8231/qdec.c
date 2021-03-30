/********************************************************************************************************
 * @file     qdec.c
 *
 * @brief    This is the head file for TLSR8231
 *
 * @author	 Telink
 * @date     May 24, 2019
 *
 * @par      Copyright (c) 2019, Telink Semiconductor (Shanghai) Co., Ltd.
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
*******************************************************************************************************/

#include "qdec.h"
/**
 * @brief      This function servers to set input port.
 * @param[in]  QDEC_InputAchTypeDef - input types of A channel.
 * @param[in]  QDEC_InputBchTypeDef - input types of A channel.
 * @return     none.
 */
void qdec_set_pin(QDEC_InputAchTypeDef channelA,QDEC_InputBchTypeDef channelB)
{
	WRITE_REG8(reg_qdec_channel_a,channelA);

	WRITE_REG8(reg_qdec_channel_b,channelB);
}

/**
 * @brief      This function servers to set qdec's mode.
 * @param[in]  QDEC_ModeTypeDef - mode type to select.
 * @return     none.
 */
void qdec_set_mode(QDEC_ModeTypeDef mode)
{
	WRITE_REG8(reg_qdec_mode,(READ_REG8(reg_qdec_mode)&0xfe)|mode);
}

/**
 * @brief      This function servers to initials qedc source clock.
 * @param[in]  none.
 * @return     none.
 */
void qdec_clk_en(void)
{
	 rc_32k_cal ();
	BM_SET(reg_clk_en2,FLD_CLK2_QDEC_EN);
	BM_SET(reg_clk_en2,FLD_CLK2_32K_QDEC_EN);
}
volatile int total_count =0;

/**
 * @brief      This function servers to read hardware counting value.
 * @param[in]  none.
 * @return     hardware counting value.
 */
unsigned char qdec_get_count_value(void)
{
	char tmp;
	unsigned int pol = 0x100;
	WRITE_REG8(rge_qdec_load,0x01);
	while(READ_REG8(rge_qdec_load) & 0x01);

	tmp = READ_REG8(reg_qdec_count);

	if((tmp >> 7) == 0x01)
		total_count-= (pol -tmp);
	else
		total_count += READ_REG8(reg_qdec_count);

	return tmp;
}


/**
 * @brief      This function servers to reset the counter and the QDEC Counter value is cleared zero.
 * @param[in]  none.
 * @return     none.
 */
void qdec_clear_conuter(void)
{
	WRITE_REG8(reg_qdec_reset,READ_REG8(reg_qdec_reset)&0xfe);
}

/**
 * @brief      This function servers to set hardware debouncing.
 * @param[in]  thrsh - lower the value of thrsh,will be regard as jitter.
 * @return     none.
 */
void qdec_set_debouncing(char thrsh)
{
	WRITE_REG8(reg_qdec_set,(READ_REG8(reg_qdec_set)&0xf8)|(thrsh&0x07));
}

/**
 * @brief      This function servers to set giop qdec fuction.
 * @param[in]  pin - the pin port selected as qdec interface pin port.
 * @param[in]  func-the function selected for pin.
 * @return     none.
 */
void gpio_set_qdec_func(GPIO_PinTypeDef pin, GPIO_FuncTypeDef func)
{
	unsigned char bit = pin & 0xff;
	if(func == AS_GPIO)
	{
		BM_SET(reg_gpio_gpio_func(pin), bit);
		return;
	}
	else
	{
		BM_CLR(reg_gpio_gpio_func(pin), bit);
	}
	//config gpio special func
	unsigned char val = 0;
	unsigned char mask = 0xff;

	switch(pin)
	{
		case GPIO_PA0:
		{
			//0x5a8[1:0]
			mask = (unsigned char)~(BIT(1)|BIT(0));

		}break;
		case GPIO_PA1:
		{
			mask= (unsigned char)~(BIT(3)|BIT(2));

		}break;

		case GPIO_PA2:
		{
			mask= (unsigned char)~(BIT(5)|BIT(4));
			if(func == AS_PWM1_N)
			{
				val =0;
			}
		}break;



		case GPIO_PB3:
		{
			mask= (unsigned char)~(BIT(6)|BIT(7));

		}break;
		case GPIO_PB4:
		{
			mask= (unsigned char)~(BIT(0)|BIT(1));

		}break;
		case GPIO_PB5:
		{
			mask= (unsigned char)~(BIT(2)|BIT(3));

		}break;

		case GPIO_PC0:
		{
			mask= (unsigned char)~(BIT(0)|BIT(1));

		}break;

		case GPIO_PC3:
		{
			mask= (unsigned char)~(BIT(7)|BIT(6));

		}break;


		case GPIO_PD1:
		{
			mask= (unsigned char)~(BIT(2)|BIT(3));

		}break;


		case GPIO_PD3:
		{
			mask= (unsigned char)~(BIT(6)|BIT(7));

		}break;

		default : break;
	}
	unsigned short reg = 0x5a8 + ((pin>>8)<<1) + ((pin&0x0f0) ? 1 : 0 );
	WRITE_REG8(reg, ( READ_REG8(reg) & mask) | val);
}


