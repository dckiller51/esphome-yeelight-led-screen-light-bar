/********************************************************************************************************
 * @file     main.c
 *
 * @brief    This is the source file for TLSR8231
 *
 * @author	 Telink
 * @date     May 9, 2019
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

#include "app_config.h"


extern void user_init();
extern void main_loop (void);


/**
 * @brief		This is main function
 * @param[in]	none
 * @return      none
 */
int main (void)
{
	system_init(BSP_INTERNAL_CAP_ENABLE);

	clock_init(SYS_CLK);

	rf_mode_init();

	rf_set_pri_2M_mode();

	gpio_init();

	user_init();

	while (1) {
		main_loop ();
	}
	return 0;
}




