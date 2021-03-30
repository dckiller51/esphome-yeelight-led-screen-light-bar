/********************************************************************************************************
 * @file     timer.h
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

#ifndef TIMER_H_
#define TIMER_H_
#include "compiler.h"
#include "register.h"
#include "analog.h"
#include "gpio.h"


#define	    tl_sys_tick_per_us   				16

/**
 * @brief   system Timer : 16Mhz, Constant
 */
enum{
	CLOCK_SYS_TIMER_CLK_1S =  16*1000*1000,
	CLOCK_SYS_TIMER_CLK_1MS = 16*1000,
	CLOCK_SYS_TIMER_CLK_1US = 16,
};

/**
 * @brief   Type of Timer
 */
typedef enum{
	TIMER0		=0,
	TIMER1		=1,
	TIMER2		=2,
}TIMER_TypeDef;

/**
 * @brief   Mode of Timer
 */
typedef enum{
	TIMER_MODE_SYSCLK		=0,
	TIMER_MODE_GPIO_TRIGGER	=1,
	TIMER_MODE_GPIO_WIDTH	=2,
	TIMER_MODE_TICK			=3,
}TIMER_ModeTypeDef;

/**
 * @brief     This function performs to gets system timer0 address.
 * @param[in] none.
 * @return    timer0 address.
 */
#if(BLE_SDK_EN)
static inline unsigned int clock_time(void)
#else
static inline unsigned long get_sys_tick(void)
#endif
{
	return reg_system_tick;
}

/**
 * @brief   This function serves to get the 32k tick.
 * @param   none
 * @return  tick of 32k .
 */
_attribute_ram_code_ unsigned int get_32k_tick(void);

/**
 * @brief     This function performs to set sleep us.
 * @param[in] microsec - mounts need to sleep.
 * @return    none
 */
#if(BLE_SDK_EN)
_attribute_ram_code_  void sleep_us(unsigned long us);
#else
_attribute_ram_code_  void delay_us (unsigned long us);
#endif
/**
 * @brief     This function performs to set sleep us.
 * @param[in] microsec - mounts need to sleep.
 * @return    none
 */
_attribute_ram_code_ void delay_ms (unsigned long ms);


/**
 * @brief     This function performs to calculation exceed us of the timer.
 * @param[in] ref - Variable of reference timer address.
 * @param[in] span_us - Variable of span us.
 * @return    the exceed.
 */
static inline unsigned int timeout_us(unsigned int ref, unsigned int us){
	return ((unsigned int)(get_sys_tick() - ref) > us * 16);
}

/**
 * @brief     This function performs to calculation exceed us of the timer.
 * @param[in] ref - Variable of reference timer address.
 * @param[in] span_us - Variable of span us.
 * @return    the exceed.
 */
static inline unsigned int timeout_ms(unsigned int ref, unsigned int ms){
	return ((unsigned int)(get_sys_tick() - ref) > ms * 16 *1000);
}
/**
 * @brief     initiate GPIO for gpio trigger and gpio width mode of timer0.
 * @param[in] pin - select pin for timer0.
 * @param[in] pol - select polarity for gpio trigger and gpio width
 * @return    none
 */
extern void timer0_gpio_init(GPIO_PinTypeDef pin, GPIO_PolTypeDef pol);

/**
 * @brief     initiate GPIO for gpio trigger and gpio width mode of timer1.
 * @param[in] pin - select pin for timer1.
 * @param[in] pol - select polarity for gpio trigger and gpio width
 * @return    none
 */
extern void timer1_gpio_init(GPIO_PinTypeDef pin, GPIO_PolTypeDef pol);

/**
 * @brief     initiate GPIO for gpio trigger and gpio width mode of timer2.
 * @param[in] pin - select pin for timer2.
 * @param[in] pol - select polarity for gpio trigger and gpio width
 * @return    none
 */
extern void timer2_gpio_init(GPIO_PinTypeDef pin,GPIO_PolTypeDef pol);

/**
 * @brief     set mode, initial tick and capture of timer0.
 * @param[in] mode - select mode for timer0.
 * @param[in] init_tick - initial tick.
 * @param[in] cap_tick  - tick of capture.
 * @return    none
 */
extern void timer0_set_mode(TIMER_ModeTypeDef mode,unsigned int init_tick, unsigned int cap_tick);

/**
 * @brief     set mode, initial tick and capture of timer1.
 * @param[in] mode - select mode for timer1.
 * @param[in] init_tick - initial tick.
 * @param[in] cap_tick  - tick of capture.
 * @return    none
 */
extern void timer1_set_mode(TIMER_ModeTypeDef mode,unsigned int init_tick, unsigned int cap_tick);

/**
 * @brief     set mode, initial tick and capture of timer2.
 * @param[in] mode - select mode for timer2.
 * @param[in] init_tick - initial tick.
 * @param[in] cap_tick  - tick of capture.
 * @return    none
 */
extern void timer2_set_mode(TIMER_ModeTypeDef mode,unsigned int init_tick, unsigned int cap_tick);

/**
 * @brief     the specifed timer start working.
 * @param[in] type - select the timer to start.
 * @return    none
 */
extern void timer_start(TIMER_TypeDef type);

/**
 * @brief     the specifed timer stop working.
 * @param[in] type - select the timer to stop.
 * @return    none
 */
extern void timer_stop(TIMER_TypeDef type);

#endif /* TIMER_H_ */

/** \defgroup GP10 Timer Examples
 *
 * 	@{
 */

/*! \page timer Table of Contents
	- [API-TIMER-CASE1:TIMER SYS CLOCK MODE](#TIMER_SYS_CLOCK_MODE)
	- [API-TIMER-CASE2:TIMER GPIO TRIGGER MODE](#TIMER_GPIO_TRIGGER_MODE)
	- [API-TIMER-CASE3:TIMER GPIO WIDTH MODE](#TIMER_GPIO_WIDTH_MODE)
	- [API-TIMER-CASE4:TIMER TICK MODE](#TIMER_TICK_MODE)
	- [API-TIMER-CASE5:TIMER WATCHDOG MODE](#TIMER_WATCHDOG_MODE)

\n
Variables used in the following cases are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
#define LED1     		        GPIO_PD0
#define LED2     		        GPIO_PD3
#define LED3     		        GPIO_PD4
#define LED4     		        GPIO_PD5

#define SW1      		        GPIO_PD1
#define SW2      		        GPIO_PD2
~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=TIMER_SYS_CLOCK_MODE> API-TIMER-CASE1:TIMER SYS CLOCK MODE </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | timer2_set_mode() | timer2_set_mode(TIMER_MODE_SYSCLK,0,1000 * CLOCK_SYS_CLOCK_1MS) | set the mode and parameter for timer2  | ^ |
| ^ | ^ | timer_start() | timer_start(TIMER2) | start timer2 | ^ |
| ^ | main_loop() | none || Main program loop | ^ |

<h1 id=TIMER_GPIO_TRIGGER_MODE> API-TIMER-CASE2:TIMER GPIO TRIGGER MODE </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | if(reg_tmr_sta & FLD_TMR_STA_TMR2 == FLD_TMR_STA_TMR2) ||| determine whether timer2 interrupt flag is right | 2019-1-10 |
| ^ | reg_tmr_sta &Iota;= FLD_TMR_STA_TMR2 ||| clear interrupt flag | ^ |
| ^ | timer2_irq_cnt ++ ||| interrupt processing function | ^ |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | timer2_gpio_init() | timer2_gpio_init(SW1, pol_falling) | open interrupt of the specified pin for timer2  | ^ |
| ^ | ^ | irq_enable() || enable global interrupt | ^ |
| ^ | ^ | timer2_set_mode() | timer2_set_mode(TIMER_MODE_GPIO_TRIGGER,0,3) | set the mode and parameter for timer2  | ^ |
| ^ | ^ | timer_start() | timer_start(TIMER2) | start timer2 | ^ |
| ^ | main_loop() | none || Main program loop | ^ |

<h1 id=TIMER_GPIO_WIDTH_MODE> API-TIMER-CASE3:TIMER GPIO WIDTH MODE </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | if(reg_tmr_sta & FLD_TMR_STA_TMR2 == FLD_TMR_STA_TMR2) ||| determine whether timer2 interrupt flag is right | 2019-1-10 |
| ^ | reg_tmr_sta &Iota;= FLD_TMR_STA_TMR2 ||| clear interrupt flag | ^ |
| ^ | gpio_width = reg_tmr2_tick ||| get the tick of gpio width | ^ |
| ^ | reg_tmr2_tick = 0 ||| clear tick of timer2 to count again | ^ |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | timer2_gpio_init() | timer2_gpio_init(SW1, pol_falling) | open interrupt of the specified pin for timer2  | ^ |
| ^ | ^ | irq_enable() || enable global interrupt | ^ |
| ^ | ^ | timer2_set_mode() | timer2_set_mode(TIMER_MODE_GPIO_WIDTH,0,0) | set the mode and parameter for timer2  | ^ |
| ^ | ^ | timer_start() | timer_start(TIMER2) | start timer2 | ^ |
| ^ | main_loop() | none || Main program loop | ^ |

<h1 id=TIMER_TICK_MODE> API-TIMER-CASE4:TIMER TICK MODE </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | timer2_set_mode() | timer2_set_mode(TIMER_MODE_TICK,0,0) | set the mode and parameter for timer2  | ^ |
| ^ | ^ | timer_start() | timer_start(TIMER2) | start timer2 | ^ |
| ^ | main_loop() | none || Main program loop | ^ |

<h1 id=TIMER_WATCHDOG_MODE> API-TIMER-CASE5:TIMER WATCHDOG MODE </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | wd_set_interval_ms() | wd_set_interval_ms(1000,CLOCK_SYS_CLOCK_1MS) | set parameter for watchdog mode of timer2 | ^ |
| ^ | ^ | wd_start() || start watchdog | ^ |
| ^ | main_loop() | wd_clear() || feed the dog | ^ |

<h1> History Record </h1>

| Date | Description | Author |
| :--- | :---------- | :----- |
| 2019-1-10 | initial release | LJW |


*/

 /** @}*/ //end of GP10
