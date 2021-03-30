/********************************************************************************************************
 * @file     watchdog.c 
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

#include "register.h"
#include "gpio.h"
#include "watchdog.h"
/**
 * @brief     This function set the seconds period.It is likely with WD_SetInterval.
 *            Just this function calculate the value to set the register automatically .
 * @param[in] period_s - The seconds to set. unit is second
 * @param[in] tick_per_ms - set the tick of every ms
 * @return    none
 */
void wd_set_interval_ms(unsigned int period_ms,unsigned long int tick_per_ms)
{
	static unsigned short tmp_period_ms = 0;
	tmp_period_ms = (period_ms*tick_per_ms>>18);
	reg_tmr2_tick = 0x00000000;    //reset tick register
	reg_tmr_ctrl &= (~FLD_TMR_WD_CAPT);
	reg_tmr_ctrl |= (tmp_period_ms<<9); //set the capture register
}

#if (BLE_SDK_EN)
/**
 * @brief     This function set the seconds period.It is likely with WD_SetInterval.
 *            Just this function calculate the value to set the register automatically .
 * @param[in] ms - The seconds to set. unit is second
 * @return    none
 */
static inline void wd_set_interval(int ms)	//  in ms
{
	SET_FLD_V(reg_tmr_ctrl, FLD_TMR_WD_CAPT, (ms * (CLOCK_RUN_CLOCK_HZ/1000) >> WATCHDOG_TIMEOUT_COEFF), FLD_TMR2_MODE, 0);
	reg_tmr2_tick = 0;
}
#endif
