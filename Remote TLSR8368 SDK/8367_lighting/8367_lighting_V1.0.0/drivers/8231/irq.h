
#pragma once

#include "register.h"


/**
 * @brief      This function servers to enable IRQ.
 * @param[in]  none
 * @return     if return 1 is enable.else disable.
 */
static inline unsigned char irq_enable(void){
	unsigned char r = reg_irq_en;		// don't worry,  the compiler will optimize the return value if not used
	reg_irq_en = 1;
	return r;
}

/**
 * @brief      This function servers to disable IRQ.
 * @param[in]  none
 * @return     if return 0 is disable.else enable.
 */
static inline unsigned char irq_disable(void){
	unsigned char r = reg_irq_en;		// don't worry,  the compiler will optimize the return value if not used
	reg_irq_en = 0;
	return r;
}

/**
 * @brief      This function servers to restore IRQ.
 * @param[in]  none
 * @return     if return 1 is irg restore.
 */
static inline void irq_restore(unsigned char en){
	reg_irq_en = en;
}

/**
 * @brief      This function servers to set IRQ mask.
 * @param[in]  variable of msk.
 * @return     none.
 */
static inline void irq_set_mask(unsigned long msk){
	BM_SET(reg_irq_mask, msk);
}

/**
 * @brief      This function servers to enable one interrupt
 * @param[in]  none
 * @return     the value of IRQ register.
 */
static inline unsigned long irq_get_mask(void){
	return reg_irq_mask;
}

/**
 * @brief      This function servers to clear IRQ mask.
 * @param[in]  msk - variable of msk.
 * @return     none.
 */
static inline void irq_clr_mask(unsigned long msk){
	BM_CLR(reg_irq_mask, msk);
}

/**
 * @brief      This function servers to get an IRQ source.
 * @param[in]  none.
 * @return     IRQ source.
 */
static inline unsigned long irq_get_src(){
	return reg_irq_src;
}

/**
 * @brief      This function servers to clear the specified IRQ source.
 * @param[in]  msk - variable of msk.
 * @return     none.
 */
static inline void irq_clr_src(unsigned long msk){
    reg_irq_src |= msk;
}

/**
 * @brief      This function servers to clear all IRQ source.
 * @param[in]  none.
 * @return     none.
 */
static inline void irq_clr_all_src(){
	reg_irq_src = 0xffffffff;
}

/**
 * @brief      This function servers to set the mask of RF IRQ.
 * @param[in]  msk - variable of msk.
 * @return     none.
 */
static inline void rf_irq_set_mask(unsigned int msk)
{
    reg_rf_irq_mask |= msk;
}

/**
 * @brief      This function servers to clear the mask of RF IRQ.
 * @param[in]  msk - variable of msk.
 * @return     none.
 */
static inline void rf_irq_clr_mask(unsigned int msk)
{
    reg_rf_irq_mask &= (~msk);
}

/**
 * @brief      This function servers to get the RF IRQ source.
 * @param[in]  none.
 * @return     the state of RF IRQ register.
 */
static inline unsigned short rf_irq_get_src(void)
{
    return reg_rf_irq_status;
}

/**
 * @brief      This function servers to clear the RF IRQ source.
 * @param[in]  msk - variable of msk.
 * @return     none.
 */
static inline void rf_irq_clr_src(unsigned short msk)
{
    reg_rf_irq_status |= msk;
}

