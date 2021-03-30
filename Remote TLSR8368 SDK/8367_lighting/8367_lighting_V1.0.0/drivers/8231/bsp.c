/********************************************************************************************************
 * @file     bsp.c
 *
 * @brief    This file provides set of common functions for driver
 *
 * @author   Telink
 * @date     May. 9, 2019
 *
 * @par      Copyright (c) 2016, Telink Semiconductor (Shanghai) Co., Ltd.
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
 *
 *******************************************************************************************************/

#include "bsp.h"
#include "clock.h"
#include "analog.h"
#include "timer.h"

/**
 * @brief      This function writes a byte data to analog register
 * @param[in]  addr - the address of the analog register needs to write
 * @param[in]  value  - the data will be written to the analog register
 * @param[in]  e - the end address of value
 * @param[in]  s - the start address of the value
 * @return     none
 */
void sub_wr_ana(unsigned int addr, unsigned char value, unsigned char e, unsigned char s)
{
	unsigned char v, mask, tmp1, target, tmp2;

	v = analog_read(addr);
	mask = BIT_MASK_LEN(e - s + 1);
	tmp1 = value & mask;

	tmp2 = v & (~BIT_RNG(s,e));

	target = (tmp1 << s) | tmp2;
	analog_write(addr, target);
}

/**
 * @brief      This function writes a byte data to a specified analog register
 * @param[in]  addr - the address of the analog register needs to write
 * @param[in]  value  - the data will be written to the analog register
 * @param[in]  e - the end address of value
 * @param[in]  s - the start address of the value
 * @return     none
 */
void sub_wr(unsigned int addr, unsigned char value, unsigned char e, unsigned char s)
{
	unsigned char v, mask, tmp1, target, tmp2;

	v = READ_REG8(addr);
	mask = BIT_MASK_LEN(e - s + 1);
	tmp1 = value & mask;

	tmp2 = v & (~BIT_RNG(s,e));

	target = (tmp1 << s) | tmp2;
	WRITE_REG8(addr, target);
}
/**
 * @brief   This function serves to initialize the related analog registers
 *          to default values after MCU is waked up from deep sleep mode.
 * @param   internal_cap -- the value to set internal cap.
 * @return  none
 */
volatile unsigned char internal_cap_flag;
_attribute_ram_code_ void system_init(Bsp_InternalCapDef internal_cap)    //must on ramcode
{
	internal_cap_flag = internal_cap;
	WRITE_REG8(0x60,0x00);   //open all the clk,disable all the rst
	WRITE_REG8(0x61,0x00);   //open all the clk,disable all the rst
	WRITE_REG8(0x62,0x00);   //open all the clk,disable all the rst
	WRITE_REG8(0x63,0xff);   //open all the clk,disable all the rst
	WRITE_REG8(0x64,0xff);   //open all the clk,disable all the rst
	WRITE_REG8(0x65,0xff);
	WRITE_REG8(0x5b5,0x0c);  //Enable gpio(core) irq and wakeup for keyboard

	analog_write(0x03, 0x43);	//Increase reternant current
	analog_write(0x06, 0x00);	//turn on baseband
	analog_write(0x20, 0x00);	//wakeup reset time: (0xff - 0xc1)*32 = 2000 us
	analog_write(0x2d, 0x48);	//quick settle: 200 us

	if(internal_cap){
		analog_write(0x81, 0xe8); // increase xtal current
	}
	else{
		analog_write(0x81, 0xe0); //increase xtal current (confirmed by peng.sun and wenfeng)
	}
    /* Open 24M XTAL. */
    analog_write(0x05, 0xca);
    for(volatile unsigned int i =0; i<10*24; i++);
	analog_write(0x05, 0xc2);
	for(volatile unsigned int i =0; i<210*24; i++);
	reg_dma_chn_en = 0;
	reg_dma_chn_irq_msk = 0;

	if(internal_cap){
	     /* Set 24M XTAL buffer and doubler. */
	     analog_write(0x80, 0x61); //Enable 24M clk buf
	     analog_write(0x81, 0xd4); //Enable 24M clk buf -> 0x4f
	}else{
		 analog_write(0x80, 0x21); //disable internal cap
         analog_write(0x81, 0xc0); //disable internal cap
	}
	analog_write(0x82, 0x5f); //Enable 48M doubler
	for(volatile int i =0;i<24*50;i++);


	/* 24M RC calibrate. */
	rc_24m_cal();
	/* initiate the value of 32k count */
	WRITE_REG16(0x750, 8000); //set 32k 16cyle avoid err use in a very quick suspend/deepsleep
	/* System Timer enable. */
	reg_sys_timer_ctrl |= FLD_SYSTEM_TICK_START;

	/* Must */
	WRITE_REG8(0x74a,0x29);//Enable calibration and close system timer 0x29
	WRITE_REG8(0x74a,0x28);//End calibration,calibration value is to be written to register 0x749|0x748.0x28
	WRITE_REG8(0x74a,0xa8);//Enable system timer and disable calibration
	for(volatile int i =0;i<16*1000;i++);
}




