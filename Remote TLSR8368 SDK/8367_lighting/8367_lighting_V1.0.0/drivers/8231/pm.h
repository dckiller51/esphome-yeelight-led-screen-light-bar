/********************************************************************************************************
 * @file     pm.h
 *
 * @brief    This is the header file for TLSR8231
 *
 * @author	 Telink
 * @date     May 12, 2019
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

#pragma once

#include "gpio.h"
#include "bsp.h"

#define PM_PAD_FILTER_EN           	 1			//Pad filter enable/disable
/**
 * @brief   5316 analog register 0x34-0x3e can store information when MCU in deep sleep mode
 *          store your information in these ana_regs before deep sleep by calling analog_write function
 *          when MCU wake up from deep sleep, read the information by by calling analog_read function
 */
//these five below are stable
#define DEEP_ANA_REG0    0x3a
#define DEEP_ANA_REG1    0x3b
#define DEEP_ANA_REG2    0x3c
#define DEEP_ANA_REG3    0x3d

/**
 * @brief   these six below may have some problem when user enter deep sleep but ERR wake up
 *          for example, when set a GPIO PAD high wake up deep sleep, but this gpio is high before
 *          you call func pm_sleep_wakeup, then deep sleep will be ERR wake up, these 6 register
 *          information loss.
 */
#define DEEP_ANA_REG5    0x34
#define DEEP_ANA_REG6    0x35
#define DEEP_ANA_REG7    0x36
#define DEEP_ANA_REG8    0x37
#define DEEP_ANA_REG9    0x38
#define DEEP_ANA_REG10   0x39

#define ADV_DEEP_FLG	 0x01
#define CONN_DEEP_FLG	 0x02

#define SYS_DEEP_ANA_REG 0x3e             //ana_3e system use for external 32k mode, user can not use


#define BLT_RESET_WAKEUP_TIME_2000		1

#if (BLT_RESET_WAKEUP_TIME_2000)
	#define RESET_TIME_US	    	  2000//1500
	#define EARLYWAKEUP_TIME_US       2150//1620
	#define EMPTYRUN_TIME_US       	  2450//1920
#elif(BLT_RESET_WAKEUP_TIME_2200)
	#define RESET_TIME_US	    	  2200
	#define EARLYWAKEUP_TIME_US       2300
	#define EMPTYRUN_TIME_US       	  2600
#elif(BLT_RESET_WAKEUP_TIME_2400)
	#define RESET_TIME_US	    	  2400
	#define EARLYWAKEUP_TIME_US       2500
	#define EMPTYRUN_TIME_US       	  2800
#elif(BLT_RESET_WAKEUP_TIME_2600)
	#define RESET_TIME_US	    	  2600
	#define EARLYWAKEUP_TIME_US       2700
	#define EMPTYRUN_TIME_US       	  3000
#elif(BLT_RESET_WAKEUP_TIME_2800)
	#define RESET_TIME_US	    	  2800
	#define EARLYWAKEUP_TIME_US       2900
	#define EMPTYRUN_TIME_US       	  3200
#else
#endif

/**
 * @brief  Retention mode reference voltage 1.8v or 1.2v.
 */
typedef enum{
	RETENTION_MODE_DISABLE 		= 0x00,
	RETENTION_REF_VOL_1_2V 		= BIT(4),
	RETENTION_REF_VOL_1_8V 		= BIT(5),
}RetentionMode_TypeDef;

#define RetentionModeFlag	RETENTION_REF_VOL_1_8V
/**
 * @brief Sleep mode define.
 */
typedef enum{
	SUSPEND_MODE 		        = 0x00,
	DEEPSLEEP_MODE    	        = 0x01,
	RETENTION_DEEPSLEEP_MODE	= DEEPSLEEP_MODE | RetentionModeFlag,
}SleepMode_TypeDef;

/**
 * @brief set wake up source.
 */
typedef enum {
	 PM_WAKEUP_PAD   = BIT(4),
	 PM_WAKEUP_CORE  = BIT(5),
	 PM_WAKEUP_TIMER = BIT(6),
	 PM_WAKEUP_GPIO  = PM_WAKEUP_PAD | PM_WAKEUP_CORE, // 一般不用PM_WAKEUP_CORE,  PM_WAKEUP_CORE 只能在suspend 上使用
}SleepWakeupSrc_TypeDef;

/**
 * @brief wakeup status from return value of "pm_sleep_wakeup".
 */
typedef enum {
	 WAKEUP_STATUS_COMP           = BIT(0),  //wakeup by comparator
	 WAKEUP_STATUS_TIMER          = BIT(1),
	 WAKEUP_STATUS_CORE           = BIT(2),
	 WAKEUP_STATUS_PAD            = BIT(3),
	 STATUS_GPIO_ERR_NO_ENTER_PM  = BIT(7),
}PM_WakeupStatusTypeDef;

/**
 * @brief     This function serves to wake up cpu from stall mode by timer0.
 * @param[in] tick - capture value of timer0.
 * @return    none.
 */
void mcu_stall_wakeup_by_timer0(unsigned int tick);

/**
 * @brief   This function serves to wake up cpu from stall mode by timer1.
 * @param   tick - capture value of timer1.
 * @return  none.
 */
void mcu_stall_wakeup_by_timer1(unsigned int tick);

/**
 * @brief     This function serves to wake up cpu from stall mode by timer2.
 * @param[in] tick - capture value of timer2.
 * @return    none.
 */
void mcu_stall_wakeup_by_timer2(unsigned int tick);

/**
 * @brief     This function servers to set a pin as wake up source.
 * @param[in] pin - the pin needs to be wake up.
 * @param[in] level - the type of level to wake up a pin.
 * @param[in] en - enable or disable the pin's function of interrupt(1: enable, 0: disable).
 * @return    none
 */
unsigned char gpio_set_wakeup(GPIO_PinTypeDef pin, GPIO_LevelTypeDef level, int en);

/**
 * @brief     this function srevers to start sleep mode.
 * @param[in] none
 * @return    none
 */
_attribute_ram_code_ _attribute_no_inline_ void  sleep_start(void);

/**
 * @brief      This function configures a GPIO pin as the wake up pin.(NOT ALL THE GPIOs)
 * @param[in]  pin - the pin needs to be configured as wake up pin.Just include PA<7:5>,PB<7:0>.
 * @param[in]  pol - the wake up polarity of the pad pin(0: low-level wake up, 1: high-level wake up)
 * @param[in]  en  - enable or disable the wake up function for the pan pin(1: Enable, 0: Disable)
 * @return     none
 */
void pm_set_gpio_wakeup (GPIO_PinTypeDef pin, GPIO_LevelTypeDef pol, int en);

/**
 * @brief      This function serves to set the working mode of MCU,e.g. suspend mode, deep sleep mode, deep sleep with SRAM retention mode and shutdown mode.
 * @param[in]  deepsleep - sleep mode type select.
 * @param[in]  wakeup_src - wake up source select.
 * @param[in]  wakeup_tick - the time of short sleep, which means MCU can sleep for less than 5 minutes.
 * @return     indicate whether the cpu is wake up successful.
 */
int pm_sleep_wakeup(SleepMode_TypeDef deepsleep, SleepWakeupSrc_TypeDef wakeup_src,unsigned int wakeup_tick);

/**
 * @brief:    Set pad filter.
 * @param[in] en - enable or disable the pin's function of interrupt(1: enable, 0: disable).
 * @return:   none.
 */
void pm_set_filter(unsigned char en);


/** \defgroup GP7  PM Examples
 *
 * 	@{
 */

/*! \page pm Table of Contents
	- [API-PM-CASE1:IDLE TIMER WAKEUP](#IDLE_TIMER_WAKEUP)
	- [API-PM-CASE2:SUSPEND PAD WAKEUP](#SUSPEND_PAD_WAKEUP)
	- [API-PM-CASE3:SUSPEND 32K WAKEUP](#SUSPEND_32K_WAKEUP)
	- [API-PM-CASE4:DEEP PAD WAKEUP](#DEEP_PAD_WAKEUP)
	- [API-PM-CASE5:DEEP 32K WAKEUP](#DEEP_32K_WAKEUP)
	- [API-PM-CASE6:SHUT DOWN](#SHUTDOWN)

<h1 id=IDLE_TIMER_WAKEUP> API-PM-CASE1:IDLE TIMER WAKEUP </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | none || user initialization | ^ |
| ^ | main_loop() | mcu_stall_wakeup_by_timer0() | mcu_stall_wakeup_by_timer0(100*CLOCK_SYS_CLOCK_1MS) | let MCU enter idle mode and wait for being waked up by Timer0 | ^ |
| ^ | ^ | delay_ms() | delay_ms(3000) | delay 3000ms after MCU waking up  | ^ |

<h1 id=SUSPEND_PAD_WAKEUP> API-PM-CASE2:SUSPEND PAD WAKEUP </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | pm_set_gpio_wakeup() | pm_set_gpio_wakeup(GPIO_PB0, LEVEL_HIGH, 1) | set the specified pin as GPIO wakeup source | ^ |
| ^ | ^ | gpio_set_up_down_resistor() | gpio_set_up_down_resistor(GPIO_PB0, GPIO_PULL_DN_100K) | enable 100k pull-down resistor of the specified pin | ^ |
| ^ | main_loop() | pm_sleep_wakeup() | pm_sleep_wakeup(SUSPEND_MODE, PM_WAKEUP_PAD, 0) | let MCU enter suspend mode and wait for being waked up by PB0 | ^ |
| ^ | ^ | delay_ms() | delay_ms(3000) | delay 3000ms after MCU waking up  | ^ |

<h1 id=SUSPEND_32K_WAKEUP> API-PM-CASE3:SUSPEND 32K WAKEUP </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | none || user initialization | ^ |
| ^ | main_loop() | pm_sleep_wakeup() | pm_sleep_wakeup(SUSPEND_MODE, PM_WAKEUP_TIMER, <br> get_sys_tick() + 100*CLOCK_SYS_TIMER_CLK_1MS) | let MCU enter suspend mode and wait for being waked up by 32k Timer | ^ |
| ^ | ^ | delay_ms() | delay_ms(3000) | delay 3000ms after MCU waking up  | ^ |

<h1 id=DEEP_PAD_WAKEUP> API-PM-CASE4:DEEP PAD WAKEUP </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | pm_set_gpio_wakeup() | pm_set_gpio_wakeup(GPIO_PB0, LEVEL_HIGH, 1) | set the specified pin as GPIO wakeup source | ^ |
| ^ | ^ | gpio_set_up_down_resistor() | gpio_set_up_down_resistor(GPIO_PB0, GPIO_PULL_DN_100K) | enable 100k pull-down resistor of the specified pin | ^ |
| ^ | ^ | pm_sleep_wakeup() | pm_sleep_wakeup(DEEPSLEEP_MODE , PM_WAKEUP_PAD, 0) | let MCU enter deepsleep mode and wait for being waked up by PB0 | ^ |
| ^ | main_loop() | none || Main program loop | ^ |

<h1 id=DEEP_32K_WAKEUP> API-PM-CASE5:DEEP 32K WAKEUP </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | pm_sleep_wakeup() | pm_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_TIMER, <br> get_sys_tick() + 100*CLOCK_SYS_TIMER_CLK_1MS) |  let MCU enter deepsleep mode and wait for being waked up by 32k Timer | ^ |
| ^ | main_loop() | none || Main program loop | ^ |

<h1 id=SHUTDOWN> API-PM-CASE6:SHUT DOWN </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | pm_sleep_wakeup() | pm_sleep_wakeup(SHUTDOWN_MODE , 0,0) |  let MCU enter shutdown mode | ^ |
| ^ | main_loop() | None || Main program loop | ^ |

<h1> History Record </h1>

| Date | Description | Author |
| :--- | :---------- | :----- |
| 2019-1-10 | initial release | LJW |


*/

 /** @}*/ //end of GP7


