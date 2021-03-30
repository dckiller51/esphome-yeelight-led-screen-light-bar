/********************************************************************************************************
 * @file     bsp.h
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
#ifndef BSP_H_
#define BSP_H_

typedef enum {
      BSP_INTERNAL_CAP_DISABLE = 0,
      BSP_INTERNAL_CAP_ENABLE = 1,
} Bsp_InternalCapDef;

/**
 *  define BIT operations
 */
#define BIT(n)                  		( 1<<(n) )
#define BIT_MASK_LEN(len)       		(BIT(len)-1)
#define BIT_RNG(s, e)  					(BIT_MASK_LEN((e)-(s)+1) << (s))

#define BM_CLR_MASK_V(x, mask)    ( (x) & ~(mask) )

#define BM_SET(x, mask)         ( (x) |= (mask) )
#define BM_CLR(x, mask)       	( (x) &= ~(mask) )
#define BM_IS_SET(x, mask)   	( (x) & (mask) )
#define BM_IS_CLR(x, mask)   	( (~x) & (mask) )
#define BM_FLIP(x, mask)      	( (x) ^=  (mask) )

/**
 *  define Reg operations
 */

// Return the bit index of the lowest 1 in y.   ex:  0b00110111000  --> 3
#define BIT_LOW_BIT(y)  (((y) & BIT(0))?0:(((y) & BIT(1))?1:(((y) & BIT(2))?2:(((y) & BIT(3))?3:			\
						(((y) & BIT(4))?4:(((y) & BIT(5))?5:(((y) & BIT(6))?6:(((y) & BIT(7))?7:				\
						(((y) & BIT(8))?8:(((y) & BIT(9))?9:(((y) & BIT(10))?10:(((y) & BIT(11))?11:			\
						(((y) & BIT(12))?12:(((y) & BIT(13))?13:(((y) & BIT(14))?14:(((y) & BIT(15))?15:		\
						(((y) & BIT(16))?16:(((y) & BIT(17))?17:(((y) & BIT(18))?18:(((y) & BIT(19))?19:		\
						(((y) & BIT(20))?20:(((y) & BIT(21))?21:(((y) & BIT(22))?22:(((y) & BIT(23))?23:		\
						(((y) & BIT(24))?24:(((y) & BIT(25))?25:(((y) & BIT(26))?26:(((y) & BIT(27))?27:		\
						(((y) & BIT(28))?28:(((y) & BIT(29))?29:(((y) & BIT(30))?30:(((y) & BIT(31))?31:32		\
						))))))))))))))))))))))))))))))))

// Return the bit index of the highest 1 in (y).   ex:  0b00110111000  --> 8
#define BIT_HIGH_BIT(y)  (((y) & BIT(31))?31:(((y) & BIT(30))?30:(((y) & BIT(29))?29:(((y) & BIT(28))?28:	\
						(((y) & BIT(27))?27:(((y) & BIT(26))?26:(((y) & BIT(25))?25:(((y) & BIT(24))?24:		\
						(((y) & BIT(23))?23:(((y) & BIT(22))?22:(((y) & BIT(21))?21:(((y) & BIT(20))?20:		\
						(((y) & BIT(19))?19:(((y) & BIT(18))?18:(((y) & BIT(17))?17:(((y) & BIT(16))?16:		\
						(((y) & BIT(15))?15:(((y) & BIT(14))?14:(((y) & BIT(13))?13:(((y) & BIT(12))?12:		\
						(((y) & BIT(11))?11:(((y) & BIT(10))?10:(((y) & BIT(9))?9:(((y) & BIT(8))?8:			\
						(((y) & BIT(7))?7:(((y) & BIT(6))?6:(((y) & BIT(5))?5:(((y) & BIT(4))?4:				\
						(((y) & BIT(3))?3:(((y) & BIT(2))?2:(((y) & BIT(1))?1:(((y) & BIT(0))?0:32				\
						))))))))))))))))))))))))))))))))

#define COUNT_ARGS_IMPL2(_1, _2, _3, _4, _5, _6, _7, _8 , _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, N, ...) N
#define COUNT_ARGS_IMPL(args)   COUNT_ARGS_IMPL2 args
#define COUNT_ARGS(...)    		COUNT_ARGS_IMPL((__VA_ARGS__, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0))

#define MACRO_CHOOSE_HELPER2(base, count) 	base##count
#define MACRO_CHOOSE_HELPER1(base, count) 	MACRO_CHOOSE_HELPER2(base, count)
#define MACRO_CHOOSE_HELPER(base, count) 	MACRO_CHOOSE_HELPER1(base, count)

#define MACRO_GLUE(x, y) x y
#define VARARG(base, ...)					MACRO_GLUE(MACRO_CHOOSE_HELPER(base, COUNT_ARGS(__VA_ARGS__)),(__VA_ARGS__))

#define MV(m, v)											(((v) << BIT_LOW_BIT(m)) & (m))

// warning MASK_VALn  are internal used macro, please use MASK_VAL instead
#define MASK_VAL2(m, v)    									(MV(m,v))
#define MASK_VAL4(m1,v1,m2,v2)    							(MV(m1,v1)|MV(m2,v2))
#define MASK_VAL6(m1,v1,m2,v2,m3,v3)    					(MV(m1,v1)|MV(m2,v2)|MV(m3,v3))
#define MASK_VAL8(m1,v1,m2,v2,m3,v3,m4,v4)    				(MV(m1,v1)|MV(m2,v2)|MV(m3,v3)|MV(m4,v4))
#define MASK_VAL10(m1,v1,m2,v2,m3,v3,m4,v4,m5,v5)    		(MV(m1,v1)|MV(m2,v2)|MV(m3,v3)|MV(m4,v4)|MV(m5,v5))
#define MASK_VAL12(m1,v1,m2,v2,m3,v3,m4,v4,m5,v5,m6,v6)    	(MV(m1,v1)|MV(m2,v2)|MV(m3,v3)|MV(m4,v4)|MV(m5,v5)|MV(m6,v6))
#define MASK_VAL14(m1,v1,m2,v2,m3,v3,m4,v4,m5,v5,m6,v6,m7,v7) (MV(m1,v1)|MV(m2,v2)|MV(m3,v3)|MV(m4,v4)|MV(m5,v5)|MV(m6,v6)|MV(m7,v7))
#define MASK_VAL16(m1,v1,m2,v2,m3,v3,m4,v4,m5,v5,m6,v6,m7,v7,m8,v8) (MV(m1,v1)|MV(m2,v2)|MV(m3,v3)|MV(m4,v4)|MV(m5,v5)|MV(m6,v6)|MV(m7,v7)|MV(m8,v8))

#define MASK_VAL(...) 					VARARG(MASK_VAL, __VA_ARGS__)


/**
 *  Reg operations
 */
#define REG_BASE_ADDR			0x800000
#define NULL        			0
#define REG_ADDR8(a)			(*(volatile unsigned char*) (REG_BASE_ADDR + (a)))
#define REG_ADDR16(a)			(*(volatile unsigned short*)(REG_BASE_ADDR + (a)))
#define REG_ADDR32(a)			(*(volatile unsigned long*) (REG_BASE_ADDR + (a)))

#define WRITE_REG8(addr,v)		(*(volatile unsigned char*)  (REG_BASE_ADDR + (addr)) = (unsigned char)(v))
#define WRITE_REG16(addr,v)		(*(volatile unsigned short*) (REG_BASE_ADDR + (addr)) = (unsigned short)(v))
#define WRITE_REG32(addr,v)		(*(volatile unsigned long*)  (REG_BASE_ADDR + (addr)) = (v))

#define READ_REG8(addr)			(*(volatile unsigned char*) (REG_BASE_ADDR + (addr)))
#define READ_REG16(addr)		(*(volatile unsigned short*)(REG_BASE_ADDR + (addr)))
#define READ_REG32(addr)		(*(volatile unsigned long*) (REG_BASE_ADDR + (addr)))

#define write_reg8   WRITE_REG8
#define write_reg16  WRITE_REG16
#define write_reg32  WRITE_REG32

#define read_reg8    READ_REG8
#define	read_reg16   READ_REG16
#define read_reg32   READ_REG32

#define TCMD_UNDER_BOTH			0xc0
#define TCMD_UNDER_RD			0x80
#define TCMD_UNDER_WR			0x40

#define TCMD_MASK				0x3f

#define TCMD_WRITE				0x3
#define TCMD_WAIT				0x7
#define TCMD_WAREG				0x8

/**
 *  command table for special registers
 */
typedef struct Cmd_TblDef {
	unsigned short	ADR;
	unsigned char	DAT;
	unsigned char	CMD;
} Cmd_TblDef;

/**
 * @brief      This function writes a byte data to analog register
 * @param[in]  addr - the address of the analog register needs to write
 * @param[in]  value  - the data will be written to the analog register
 * @param[in]  e - the end address of value
 * @param[in]  s - the start address of the value
 * @return     none
 */
extern void sub_wr_ana(unsigned int addr, unsigned char value, unsigned char e, unsigned char s);

/**
 * @brief      This function writes a byte data to a specified analog register
 * @param[in]  addr - the address of the analog register needs to write
 * @param[in]  value  - the data will be written to the analog register
 * @param[in]  e - the end address of value
 * @param[in]  s - the start address of the value
 * @return     none
 */
extern void sub_wr(unsigned int addr, unsigned char value, unsigned char e, unsigned char s);

/**
 * @brief   This function serves to reset the whole chip
 * @param   none
 * @return  none
 */
static inline void chip_reset(void)
{
	WRITE_REG8(0x6f,0x20);
}

/**
 * @brief   This function serves to initialize the related analog registers
 *          to default values after MCU is waked up from deep sleep mode.
 * @param   internal_cap -- the value to set internal cap.
 * @return  none
 */
extern void system_init(Bsp_InternalCapDef internal_cap);

#endif /* BSP_H_ */

/** \defgroup GP0 CORE Examples
 *
 * 	@{
 */


/** \defgroup GP0 CORE Examples
 *
 * 	@{
 */

/*! \page core Table of Contents
	- [API-CORE-CASE1:ACCESS MEMORY](#ACCESS_MEMORY)
	- [API-CORE-CASE2:CAL RC DBL](#CAL_RC_DBL)
	- [API-IRQ-CASE1:TIMER0 IRQ](#TIMER0_IRQ)
	- [API-IRQ-CASE2:TIMER1 IRQ](#TIMER1_IRQ)
	- [API-IRQ-CASE3:TIMER2 IRQ](#TIMER2_IRQ)
	- [API-IRQ-CASE4:USB POWER DOWN IRQ](#USB_POWER_DOWN_IRQ)
	- [API-IRQ-CASE5:DMA IRQ](#DMA_IRQ)
	- [API-IRQ-CASE6:DFIFO IRQ](#DFIFO_IRQ)
	- [API-IRQ-CASE7:UART IRQ](#UART_IRQ)
	- [API-IRQ-CASE8:MIX(QDEC/I2C/SPI) IRQ](#MIX_IRQ)
	- [API-IRQ-CASE9:USB SETUP IRQ](#USB_SETUP_IRQ)
	- [API-IRQ-CASE10:USB DATA IRQ](#USB_DATA_IRQ)
	- [API-IRQ-CASE11:USB STATUS IRQ](#USB_STATUS_IRQ)
	- [API-IRQ-CASE12:USB SETINF IRQ](#USB_SETINF_IRQ)
	- [API-IRQ-CASE13:USB EDP IRQ](#USB_EDP_IRQ)
	- [API-IRQ-CASE14:ZB RT IRQ](#ZB_RT_IRQ)
	- [API-IRQ-CASE15:PWM IRQ](#PWM_IRQ)
	- [API-IRQ-CASE16:USB 250US IRQ](#USB_250US_IRQ)
	- [API-IRQ-CASE17:USB RESET IRQ](#USB_RESET_IRQ)
	- [API-IRQ-CASE18:GPIO IRQ](#GPIO_IRQ)
	- [API-IRQ-CASE19:PM IRQ](#PM_IRQ)
	- [API-IRQ-CASE20:SYSTEM TIMER IRQ](#SYSTEM_TIMER_IRQ)
	- [API-IRQ-CASE21:GPIO2RISC_1 IRQ](#GPIO2RISC_1_IRQ)
	- [API-IRQ-CASE22:GPIO2RISC_0 IRQ](#GPIO2RISC_0_IRQ)

<h1 id=ACCESS_MEMORY> API-CORE-CASE1:ACCESS MEMORY </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | clock_32k_init() | clock_32k_init(CLK_32K_XTAL) || 32k Clock initialization, 32k Clock source is 32k RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | sram_val = READ_REG32(0x40000) || read 4 bytes from sram | ^ |
| ^ | ^ | WRITE_REG32(0x40000,0xffffffff) || write 4 bytes to sram | ^ |
| ^ | ^ | dig_val = READ_REG32(0x00) || read 4 bytes from digital register | ^ |
| ^ | ^ | WRITE_REG32(0x00,0xffffffff) || write 4 bytes to digital register | ^ |
| ^ | ^ | analog_read() | ana_val = analog_read(0x00) | read one byte from analog register | ^ |
| ^ | ^ | analog_write() | analog_write(0x00,0xff) | write one byte to analog register | ^ |
| ^ | ^ | chip_reset() || write one byte to analog register | ^ |
| ^ | main_loop() | none || main program loop | ^ |

\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
unsigned int sram_val = 0;
unsigned int dig_val =0;
unsigned char ana_val = 0;
~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=CAL_RC_DBL> API-CORE-CASE2:CAL RC DBL </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | rc_24m_cal() || calibrate the RC1 to 24M as system clock | ^ |
| ^ | ^ | rc_32k_cal() || calibrate the RC2 to 32K as 32k timer | ^ |
| ^ | ^ | doubler_cal() || calibrate the doubler | ^ |
| ^ | main_loop() | none || main program loop | ^ |

<h1 id=TIMER0_IRQ> API-IRQ-CASE1:TIMER0 IRQ </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-TIMER-CASE2:TIMER GPIO TRIGGER MODE](group___g_p10.html#TIMER_GPIO_TRIGGER_MODE)

<h1 id=TIMER1_IRQ> API-IRQ-CASE2:TIMER1 IRQ </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-TIMER-CASE2:TIMER GPIO TRIGGER MODE](group___g_p10.html#TIMER_GPIO_TRIGGER_MODE)

<h1 id=TIMER2_IRQ> API-IRQ-CASE3:TIMER2 IRQ </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-TIMER-CASE2:TIMER GPIO TRIGGER MODE](group___g_p10.html#TIMER_GPIO_TRIGGER_MODE)

<h1 id=USB_POWER_DOWN_IRQ> API-IRQ-CASE4:USB POWER DOWN IRQ </h1>

<h1 id=DMA_IRQ> API-IRQ-CASE5:DMA IRQ </h1>

<h1 id=DFIFO_IRQ> API-IRQ-CASE6:DFIFO IRQ </h1>

<h1 id=UART_IRQ> API-IRQ-CASE7:UART IRQ </h1>

<h1 id=MIX_IRQ> API-IRQ-CASE8:MIX(QDEC/I2C/SPI) IRQ </h1>

<h1 id=USB_SETUP_IRQ> API-IRQ-CASE9:USB SETUP IRQ </h1>

<h1 id=USB_DATA_IRQ> API-IRQ-CASE10:USB DATA IRQ </h1>

<h1 id=USB_STATUS_IRQ> API-IRQ-CASE11:USB STATUS IRQ </h1>

<h1 id=USB_SETINF_IRQ> API-IRQ-CASE12:USB SETINF IRQ </h1>

<h1 id=USB_EDP_IRQ> API-IRQ-CASE13:USB EDP IRQ </h1>

<h1 id=ZB_RT_IRQ> API-IRQ-CASE14:ZB RT IRQ </h1>

<h1 id=PWM_IRQ> API-IRQ-CASE15:PWM IRQ </h1>

<h1 id=USB_250US_IRQ> API-IRQ-CASE16:USB 250US IRQ </h1>

<h1 id=USB_RESET_IRQ> API-IRQ-CASE17:USB RESET IRQ </h1>

<h1 id=GPIO_IRQ> API-IRQ-CASE18:GPIO IRQ </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-GPIO-CASE1:GPIO IRQ](group___g_p5.html#GPIO_IRQ)

<h1 id=PM_IRQ> API-IRQ-CASE19:PM IRQ </h1>

<h1 id=SYSTEM_TIMER_IRQ> API-IRQ-CASE20:SYSTEM TIMER IRQ </h1>

<h1 id=GPIO2RISC_1_IRQ> API-IRQ-CASE21:GPIO2RISC_1 IRQ </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-GPIO-CASE3:GPIO IRQ RSIC1](group___g_p5.html#GPIO_IRQ_RSIC1)

<h1 id=GPIO2RISC_0_IRQ> API-IRQ-CASE22:GPIO2RISC_0 IRQ </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-GPIO-CASE2:GPIO IRQ RSIC0](group___g_p5.html#GPIO_IRQ_RSIC0)

<h1> History Record </h1>

| Date | Description | Author |
| :--- | :---------- | :----- |
| 2019-1-10 | initial release | LJW |


*/

 /** @}*/ //end of GP11

