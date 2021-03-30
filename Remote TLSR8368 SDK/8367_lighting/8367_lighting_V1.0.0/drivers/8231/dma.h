/*
 * dma.h
 *
 *  Created on: 2018-5-31
 *      Author: Administrator
 */

#ifndef DMA_H_
#define DMA_H_

#include "drivers.h"
#include "register.h"

typedef enum{
	DMA0_UART_RX,
	DMA1_UART_TX,
	DMA2_RF_RX,
	DMA3_RF_TX,
	DMA4_AES_DECO,
	DMA5_AES_CODE,
	DMA6,
	DMA7_PWM,
}DMA_chn_Typdef;

/**
 * @brief     This function resets the DMA module.
 * @param[in] none
 * @return    none
 */
static inline void dma_reset(void)
{
	reg_rst1 |= FLD_RST1_DMA;
	reg_rst1 &= (~FLD_RST1_DMA);
}
/**
 * @brief     This function performs to enable DMA interrupt.
 * @param[in] chn - variable to config the DMA interrupt channel.
 * @param[in] en - en: 1 enable. 0 disable.
 * @return    none.
 */
static inline void dma_set_irq_en(unsigned char chn, unsigned int en)
{
	reg_dma_irq_status = chn;

	if(en){
		reg_dma_chn_en |= chn;
		reg_dma_chn_irq_msk |= chn;
	}
	else{
		reg_dma_chn_en &= ~chn;
		reg_dma_chn_irq_msk &= ~chn;
	}
}
/**
 * @brief      Clear IRQ status of uart.
 * @param[in]  irq_src - select tx or rx irq.
 * @return     none
 */
static inline void dma_clr_irq_status(unsigned char irq_status)
{
	reg_dma_irq_status = irq_status;
}


/**
 * @brief      Get IRQ status of uart.
 * @param[in]  irq_src - select tx or rx irq.
 * @return     none
 */
static inline unsigned char dma_get_irq_status(void)
{
    return reg_dma_irq_status;
}

/**
 * @brief      This function serves to set the size of dma buffer
 * @param[in]  size - select tx or rx irq. caution: max size = 2048
 * @return     none
 */
static inline void dma_set_buff_size(DMA_chn_Typdef chn,unsigned int size)
{
	reg_dma_size(chn) = (unsigned char)(size/16);
}

#endif

 /* DMA_H_ */
