
#pragma once

#include "register.h"

/**
 * @brief     This function set the seconds period.It is likely with WD_SetInterval.
 *            Just this function calculate the value to set the register automatically .
 * @param[in] period_ms - The seconds to set. unit is second
 * @param[in] tick_per_ms - set the tick of every ms
 * @return    none
 */
void wd_set_interval_ms(unsigned int period_ms,unsigned long int tick_per_ms);
/**
 * @brief     start watchdog. ie enable watchdog
 * @param[in] none
 * @return    none
 */
static inline void wd_start(void)
{
#if(MODULE_WATCHDOG_ENABLE)		//  if watchdog not set,  start wd would cause problem
	BM_SET(reg_tmr_ctrl, FLD_TMR_WD_EN);
#endif
}


/**
 * @brief     stop watchdog. ie disable watchdog
 * @param[in] none
 * @return    none
 */
static inline void wd_stop(void){
#if(MODULE_WATCHDOG_ENABLE)
	BM_CLR(reg_tmr_ctrl, FLD_TMR_WD_EN);
#endif
}

/**
 * @brief     clear watchdog.
 * @param[in] none
 * @return    none
 */
static inline void wd_clear(void)
{
	reg_tmr_sta = FLD_TMR_STA_WD;
}
/*----------------------------- End of File ----------------------------------*/
