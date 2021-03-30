/********************************************************************************************************
 * @file     timer.c
 *
 * @brief    This is the source file for TLSR8231
 *
 * @author	 Telink
 * @date     May 8, 2018
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
#include "timer.h"

/**
 * @brief   This function serves to get the 32k tick.
 * @param   none
 * @return  tick of 32k .
 */
_attribute_ram_code_ unsigned int get_32k_tick(void)
{
    unsigned int t0 = 0;
    unsigned int t1 = 0;
    unsigned int n = 0;

    while (1) {

        t0 = t1;
        t1 = analog_read(0x43);
        t1 <<= 8;
        t1 += analog_read(0x42);
        t1 <<= 8;
        t1 += analog_read(0x41);
        t1 <<= 8;
        t1 += analog_read(0x40);
        if (n)
        {
            if ((unsigned int)(t1 - t0) < 2) {
                return t1;
            }
            else if ( (t0^t1) == 1 ) {
                return t0;
            }
        }
        n++;
    }
    return t1;
}

/**
 * @brief     This function performs to set sleep us.
 * @param[in] us - mounts need to sleep.
 * @return    none
 */

_attribute_ram_code_ void delay_us (unsigned long us)
{
	unsigned long t = get_sys_tick();
	while(!timeout_us(t, us)){
	}
}

/**
 * @brief     This function performs to set sleep us.
 * @param[in] us - mounts need to sleep.
 * @return    none
 */
_attribute_ram_code_ void delay_ms (unsigned long ms)
{
	unsigned long t = get_sys_tick();
	while(!timeout_ms(t, ms)){
	}
}
/**
 * @brief     initiate GPIO for gpio trigger and gpio width mode of timer0.
 * @param[in] pin - select pin for timer0.
 * @param[in] pol - select polarity for gpio trigger and gpio width
 * @return    none
 */
void timer0_gpio_init(GPIO_PinTypeDef pin, GPIO_PolTypeDef pol)
{
	gpio_set_func(pin ,AS_GPIO);
	gpio_set_output_en(pin, 0); //disable output
	gpio_set_input_en(pin ,1);//enable input
	if(pol==POL_FALLING)
	{
		gpio_set_up_down_resistor(pin,GPIO_PULL_UP_10K);


	}
	else if(pol==POL_RISING)
	{
		gpio_set_up_down_resistor(pin,GPIO_PULL_DN_100K);


	}
	gpio_set_interrupt_risc0(pin, pol);
}
/**
 * @brief     initiate GPIO for gpio trigger and gpio width mode of timer1.
 * @param[in] pin - select pin for timer1.
 * @param[in] pol - select polarity for gpio trigger and gpio width
 * @return    none
 */
void timer1_gpio_init(GPIO_PinTypeDef pin,GPIO_PolTypeDef pol)
{
	gpio_set_func(pin ,AS_GPIO);
	gpio_set_output_en(pin, 0); //disable output
	gpio_set_input_en(pin ,1);//enable input
	if(pol==POL_FALLING)
	{
		gpio_set_up_down_resistor(pin,GPIO_PULL_UP_10K);


	}
	else if(pol==POL_RISING)
	{
		gpio_set_up_down_resistor(pin,GPIO_PULL_DN_100K);


	}
	gpio_set_interrupt_risc1(pin, pol);
}
/**
 * @brief     initiate GPIO for gpio trigger and gpio width mode of timer2.
 * @param[in] pin - select pin for timer2.
 * @param[in] pol - select polarity for gpio trigger and gpio width
 * @return    none
 */
void timer2_gpio_init(GPIO_PinTypeDef pin,GPIO_PolTypeDef pol)
{
	gpio_set_func(pin ,AS_GPIO);
	gpio_set_output_en(pin, 0); //disable output
	gpio_set_input_en(pin ,1);//enable input
	if(pol==POL_FALLING)
	{
		gpio_set_up_down_resistor(pin,GPIO_PULL_UP_10K);
	}
	else if(pol==POL_RISING)
	{
		gpio_set_up_down_resistor(pin,GPIO_PULL_DN_100K);
	}

	unsigned char bit = pin & 0xff;

	BM_SET(reg_gpio_irq_risc2_en(pin), bit);

	if(pol==POL_FALLING)
	{
		BM_SET(reg_gpio_pol(pin), bit);
	}
	else if(pol==POL_RISING)
	{
		BM_CLR(reg_gpio_pol(pin), bit);
	}
}
/**
 * @brief     set mode, initial tick and capture of timer0.
 * @param[in] mode - select mode for timer0.
 * @param[in] init_tick - initial tick.
 * @param[in] cap_tick  - tick of capture.
 * @return    none
 */
void timer0_set_mode(TIMER_ModeTypeDef mode,unsigned int init_tick, unsigned int cap_tick)
{
	switch(mode)
	{
		case TIMER_MODE_SYSCLK:
		{
			reg_irq_mask |= FLD_IRQ_TMR0_EN;
			reg_tmr0_tick = init_tick;
			reg_tmr0_capt = cap_tick;
			reg_tmr_sta = FLD_TMR_STA_TMR0; //clear irq status
			reg_tmr_ctrl &= (~FLD_TMR0_MODE);
			reg_tmr_ctrl |= (TIMER_MODE_SYSCLK<<1);
			break;
		}
		case TIMER_MODE_GPIO_TRIGGER:
		{
			reg_irq_mask |= FLD_IRQ_TMR0_EN;
			reg_tmr0_tick = init_tick;
			reg_tmr0_capt = cap_tick;
			reg_tmr_sta = FLD_TMR_STA_TMR0; //clear irq status
			reg_tmr_ctrl &= (~FLD_TMR0_MODE);
			reg_tmr_ctrl |= (TIMER_MODE_GPIO_TRIGGER<<1);
			break;
		}
		case TIMER_MODE_GPIO_WIDTH:
		{
			reg_irq_mask |= FLD_IRQ_TMR0_EN;
			reg_tmr0_tick = init_tick;
			reg_tmr_sta = FLD_TMR_STA_TMR0;
			reg_tmr_ctrl &= (~FLD_TMR0_MODE);
			reg_tmr_ctrl |= (TIMER_MODE_GPIO_WIDTH<<1);
			break;
		}
		case TIMER_MODE_TICK:
		{
			reg_irq_mask |= FLD_IRQ_TMR0_EN;
			reg_tmr0_tick = init_tick; 		//clear counter
			reg_tmr_sta = FLD_TMR_STA_TMR0; //clear irq status
			reg_tmr_ctrl &= (~FLD_TMR0_MODE);
			reg_tmr_ctrl |= (TIMER_MODE_TICK<<1);
			break;
		}
		default: break;
	}
}
/**
 * @brief     set mode, initial tick and capture of timer1.
 * @param[in] mode - select mode for timer1.
 * @param[in] init_tick - initial tick.
 * @param[in] cap_tick  - tick of capture.
 * @return    none
 */
void timer1_set_mode(TIMER_ModeTypeDef mode,unsigned int init_tick, unsigned int cap_tick)
{
	switch(mode)
	{
		case TIMER_MODE_SYSCLK:
		{
			reg_irq_mask |= FLD_IRQ_TMR1_EN;
			reg_tmr1_tick = init_tick;
			reg_tmr1_capt = cap_tick;
			reg_tmr_sta = FLD_TMR_STA_TMR1; //clear irq status
			reg_tmr_ctrl &= (~FLD_TMR1_MODE);
			reg_tmr_ctrl |= (TIMER_MODE_SYSCLK<<4);
			break;
		}
		case TIMER_MODE_GPIO_TRIGGER:
		{
			reg_irq_mask |= FLD_IRQ_TMR1_EN;
			reg_tmr1_tick = init_tick;
			reg_tmr1_capt = cap_tick;
			reg_tmr_sta = FLD_TMR_STA_TMR1; //clear irq status
			reg_tmr_ctrl &= (~FLD_TMR1_MODE);
			reg_tmr_ctrl |= (TIMER_MODE_GPIO_TRIGGER<<4);
			break;
		}
		case TIMER_MODE_GPIO_WIDTH:
		{
			reg_irq_mask |= FLD_IRQ_TMR1_EN;
			reg_tmr1_tick = init_tick;
			reg_tmr_sta = FLD_TMR_STA_TMR1;
			reg_tmr_ctrl &= (~FLD_TMR1_MODE);
			reg_tmr_ctrl |= (TIMER_MODE_GPIO_WIDTH<<4);
			break;
		}
		case TIMER_MODE_TICK:
		{
			reg_irq_mask |= FLD_IRQ_TMR1_EN;
			reg_tmr1_tick = init_tick; //clear counter
			reg_tmr_sta = FLD_TMR_STA_TMR1; //clear irq status
			reg_tmr_ctrl &= (~FLD_TMR1_MODE);
			reg_tmr_ctrl |= (TIMER_MODE_TICK<<4);
			break;
		}
		default: break;
	}
}
/**
 * @brief     set mode, initial tick and capture of timer2.
 * @param[in] mode - select mode for timer2.
 * @param[in] init_tick - initial tick.
 * @param[in] cap_tick  - tick of capture.
 * @return    none
 */
void timer2_set_mode(TIMER_ModeTypeDef mode,unsigned int init_tick, unsigned int cap_tick)
{
	switch(mode)
	{
		case TIMER_MODE_SYSCLK:
		{
			reg_irq_mask |= FLD_IRQ_TMR2_EN;
			reg_tmr2_tick = init_tick;
			reg_tmr2_capt = cap_tick;
			reg_tmr_sta = FLD_TMR_STA_TMR2; //clear irq status
			reg_tmr_ctrl &= (~FLD_TMR2_MODE);
			reg_tmr_ctrl |= (TIMER_MODE_SYSCLK<<7);
			break;
		}
		case TIMER_MODE_GPIO_TRIGGER:
		{
			reg_irq_mask |= FLD_IRQ_TMR2_EN;
			reg_tmr2_tick = init_tick;
			reg_tmr2_capt = cap_tick;
			reg_tmr_sta = FLD_TMR_STA_TMR2; //clear irq status
			reg_tmr_ctrl &= (~FLD_TMR2_MODE);
			reg_tmr_ctrl |= (TIMER_MODE_GPIO_TRIGGER<<7);
			break;
		}
		case TIMER_MODE_GPIO_WIDTH:
		{
			reg_irq_mask |= FLD_IRQ_TMR2_EN;
			reg_tmr2_tick = init_tick;
			reg_tmr_sta = FLD_TMR_STA_TMR2;
			reg_tmr_ctrl &= (~FLD_TMR2_MODE);
			reg_tmr_ctrl |= (TIMER_MODE_GPIO_WIDTH<<7);
			break;
		}
		case TIMER_MODE_TICK:
		{
			reg_irq_mask |= FLD_IRQ_TMR2_EN;
			reg_tmr2_tick = init_tick; //clear counter
			reg_tmr_sta = FLD_TMR_STA_TMR2; //clear irq status
			reg_tmr_ctrl &= (~FLD_TMR2_MODE);
			reg_tmr_ctrl |= (TIMER_MODE_TICK<<7);
			break;
		}
		default: break;
	}
}
/**
 * @brief     the specifed timer start working.
 * @param[in] type - select the timer to start.
 * @return    none
 */
void timer_start(TIMER_TypeDef type)
{
	switch(type)
	{
		case TIMER0:
			reg_tmr_ctrl |= FLD_TMR0_EN;
			break;
		case TIMER1:
			reg_tmr_ctrl |= FLD_TMR1_EN;
			break;
		case TIMER2:
			reg_tmr_ctrl |= FLD_TMR2_EN;
			break;
		default:
			break;
	}
}
/**
 * @brief     the specifed timer stop working.
 * @param[in] type - select the timer to stop.
 * @return    none
 */
void timer_stop(TIMER_TypeDef type)
{
	switch(type)
	{
		case TIMER0:
			reg_tmr_ctrl &= (~FLD_TMR0_EN);
			break;
		case TIMER1:
			reg_tmr_ctrl &= (~FLD_TMR1_EN);
			break;
		case TIMER2:
			reg_tmr_ctrl &= (~FLD_TMR2_EN);
			break;
		default:
			break;
	}
}


