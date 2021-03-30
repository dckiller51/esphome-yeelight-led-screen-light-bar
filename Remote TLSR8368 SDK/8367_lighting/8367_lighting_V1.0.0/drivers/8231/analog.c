/********************************************************************************************************
 * @file     analog.c
 *
 * @brief    This is the source file for TLSR8231
 *
 * @author	 Telink
 * @date     May 24, 2019
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
#include "analog.h"
#include "compiler.h"
#include "register.h"
#include "irq.h"

/**
 * @brief      This function serves to wait for analog register ready.
 * @param[in]  none.
 * @return     none.
 */
static inline void analog_wait(void){
	while(reg_ana_ctrl & FLD_ANA_BUSY){}
}
/**
 * @brief      This function serves to analog register read.
 * @param[in]  addr - address need to be read.
 * @return     the result of read.
 */
_attribute_ram_code_ unsigned char analog_read(unsigned char addr){
	unsigned char r = irq_disable();

	reg_ana_addr = addr;
	reg_ana_ctrl = (FLD_ANA_START);
	analog_wait();
	unsigned char data = reg_ana_data;
	reg_ana_ctrl = 0;		// finish
	irq_restore(r);
	return data;
}

/**
 * @brief      This function serves to analog register write.
 * @param[in]  addr - address need to be write.
 * @param[in]  v - the value need to be write.
 * @return     none.
 */
_attribute_ram_code_ void analog_write(unsigned char addr, unsigned char v){
	unsigned char r = irq_disable();

	reg_ana_addr = addr;
	reg_ana_data = v;
	reg_ana_ctrl = (FLD_ANA_START | FLD_ANA_RW);
	analog_wait();
	reg_ana_ctrl = 0;
	irq_restore(r);
}

/**
 * @brief      This function serves to analog register read.
 * @param[in]  addr - address need to be read.
 * @param[in]  v	- data buffer
 * @param[in]  len  - the length of buffer
 * @return     the result of read.
 */
void analog_read_multi(unsigned char addr, unsigned char *v, int len){
	unsigned char r = irq_disable();

	reg_ana_ctrl = 0;		// issue clock
	reg_ana_addr = addr;
	while(len--){
		reg_ana_ctrl = FLD_ANA_CYC | FLD_ANA_START;
		analog_wait();
		*v++ = reg_ana_data;
	}
	reg_ana_ctrl = 0; 		// finish

	irq_restore(r);
}

/**
 * @brief      This function serves to analog register read.
 * @param[in]  addr - address need to be written.
 * @param[in]  v	- data buffer
 * @param[in]  len  - the length of buffer
 * @return     none.
 */
void analog_write_multi(unsigned char addr, unsigned char *v, int len){
	unsigned char r = irq_disable();

	reg_ana_addr = addr;
	while(len--){
		reg_ana_data = *v++;
		reg_ana_ctrl = FLD_ANA_CYC | FLD_ANA_START | FLD_ANA_RW; 	// multi write
		analog_wait();
	}
	reg_ana_ctrl = 0; 		// finish
	irq_restore(r);
}


