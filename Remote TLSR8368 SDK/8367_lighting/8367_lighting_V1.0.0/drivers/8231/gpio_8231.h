/********************************************************************************************************
 * @file     gpio.h
 *
 * @brief    This file provides set of functions to manage GPIOs
 *
 * @author   rui.li@telink-semi.com;
 * @date     May.7, 2019
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
#ifndef _GPIO_H
#define _GPIO_H

#include "bsp.h"
#include "register_8231.h"
#include "gpio_default_8231.h"

/**
 *  @brief  Define GPIO types
 */
typedef enum{
		GPIO_GROUPA    = 0x000,
		GPIO_GROUPB    = 0x100,
		GPIO_GROUPC    = 0x200,
		GPIO_GROUPD    = 0x300,
		GPIO_GROUPE    = 0x400,

	    GPIO_PA0  = GPIO_GROUPA | BIT(0),   GPIO_PWM0A0  = GPIO_PA0,   GPIO_PWM3A0      = GPIO_PA0,   GPIO_RX_CYC2LNAA0 = GPIO_PA0,   GPIO_RESERVE_3A0 = GPIO_PA0,
		GPIO_PA1  = GPIO_GROUPA | BIT(1),   GPIO_PWM3NA1 = GPIO_PA1,   GPIO_RESERVE_1A1 = GPIO_PA1,   GPIO_RESERVE_2A1  = GPIO_PA1,   GPIO_RESERVE_3A1 = GPIO_PA1,
		GPIO_PA2  = GPIO_GROUPA | BIT(2),   GPIO_PWM1NA2 = GPIO_PA2,   GPIO_RESERVE_1A2 = GPIO_PA2,   GPIO_RESERVE_2A2  = GPIO_PA2,   GPIO_RESERVE_3A2 = GPIO_PA2,
		GPIO_PA3  = GPIO_GROUPA | BIT(3),   GPIO_PWM4A3  = GPIO_PA3,   GPIO_I2C_CKA3    = GPIO_PA3,   GPIO_RESERVE_2A3  = GPIO_PA3,   GPIO_RESERVE_3A3 = GPIO_PA3,
		GPIO_PA4  = GPIO_GROUPA | BIT(4),   GPIO_PWM2A4  = GPIO_PA4,   GPIO_I2C_SDA4    = GPIO_PA4,   GPIO_RESERVE_2A4  = GPIO_PA4,   GPIO_RESERVE_3A4 = GPIO_PA4,
		GPIO_PA5  = GPIO_GROUPA | BIT(5),   GPIO_PWM2NA5 = GPIO_PA5,   GPIO_I2C_CKA5    = GPIO_PA5,   GPIO_I2C_MCKA5    = GPIO_PA5,   GPIO_RESERVE_3A5 = GPIO_PA5,
		GPIO_PA6  = GPIO_GROUPA | BIT(6),   GPIO_PWM4NA6 = GPIO_PA6,   GPIO_I2C_SDA6    = GPIO_PA6,   GPIO_I2C_MSDA6    = GPIO_PA6,   GPIO_RESERVE_3A6 = GPIO_PA6,
		GPIO_PA7  = GPIO_GROUPA | BIT(7),   GPIO_PWM0A7  = GPIO_PA7,   GPIO_RESERVE_1A7 = GPIO_PA7,   GPIO_TX_CYC2PAA7  = GPIO_PA7,   GPIO_RESERVE_3A7 = GPIO_PA7,
		GPIOA_ALL = GPIO_GROUPA | 0x00ff,

		GPIO_PB0 = GPIO_GROUPB | BIT(0),   GPIO_PWM4B0  = GPIO_PB0,    GPIO_MCNB0     = GPIO_PB0,     GPIO_RX_CYC2LNAB0 = GPIO_PB0,   GPIO_RESERVE_3B0 = GPIO_PB0,
		GPIO_PB1 = GPIO_GROUPB | BIT(1),   GPIO_PWM1B1  = GPIO_PB1,    GPIO_MDOB1     = GPIO_PB1,     GPIO_TX_CYC2PAB1  = GPIO_PB1,   GPIO_RESERVE_3B1 = GPIO_PB1,
		GPIO_PB2 = GPIO_GROUPB | BIT(2),   GPIO_PWM0B2  = GPIO_PB2,    GPIO_MDIB2     = GPIO_PB2,     GPIO_RESERVR_2B2  = GPIO_PB2,   GPIO_RESERVR_3B2 = GPIO_PB2,
		GPIO_PB3 = GPIO_GROUPB | BIT(3),   GPIO_PWM3NB3 = GPIO_PB3,    GPIO_MCKB3     = GPIO_PB3,     GPIO_PWM2B3       = GPIO_PB3,   GPIO_RESERVE_3B3 = GPIO_PB3,
		GPIO_PB4 = GPIO_GROUPB | BIT(4),   GPIO_PWM2NB4 = GPIO_PB4,    GPIO_I2C_MCKB4 = GPIO_PB4,     GPIO_UART_TXB4    = GPIO_PB3,   GPIO_RESERVE_3B4 = GPIO_PB3,//Pull out J18, if use for pull-up.
		GPIO_PB5 = GPIO_GROUPB | BIT(5),   GPIO_PWM4B5  = GPIO_PB5,    GPIO_I2C_MSDB5 = GPIO_PB5,     GPIO_UART_RXB5    = GPIO_PB5,   GPIO_RESERVE_3B5 = GPIO_PB5,//Pull out J5, if use for pull-up.
		GPIO_PB6 = GPIO_GROUPB | BIT(6),   GPIO_PWM0NB6 = GPIO_PB6,    GPIO_I2C_MCKB6 = GPIO_PB6,     GPIO_UART_RTSB6   = GPIO_PB6,   GPIO_RESERVE_3B6 = GPIO_PB6,
		GPIO_PB7 = GPIO_GROUPB | BIT(7),   GPIO_PWM1B7  = GPIO_PB7,    GPIO_I2C_MSDB7 = GPIO_PB7,     GPIO_UART_CTSB7   = GPIO_PB7,   GPIO_RESERVE_3B7 = GPIO_PB7,
		GPIOB_ALL = GPIO_GROUPB | 0x00ff,

		GPIO_PC0 = GPIO_GROUPC | BIT(0),   GPIO_RESERVE_0C0 = GPIO_PC0,   GPIO_RESERVE_1C0 = GPIO_PC0,   GPIO_RESERVE_2C0 = GPIO_PC0,    GPIO_RESERVE_3C0        = GPIO_PC0,
		GPIO_PC1 = GPIO_GROUPC | BIT(1),   GPIO_PWM2NC1     = GPIO_PC1,   GPIO_RESERVE_1C1 = GPIO_PC1,   GPIO_RESERVE_2C1 = GPIO_PC1,    GPIO_RESERVE_3C1        = GPIO_PC1,
		GPIO_PC2 = GPIO_GROUPC | BIT(2),   GPIO_SPI_CNC2    = GPIO_PC2,   GPIO_PWM1C2      = GPIO_PC2,   GPIO_SPI_MCNC2   = GPIO_PC2,    GPIO_UART_CTSC2         = GPIO_PC2,
		GPIO_PC3 = GPIO_GROUPC | BIT(3),   GPIO_SPI_DOC3    = GPIO_PC3,   GPIO_PWM0NC3     = GPIO_PC3,   GPIO_SPI_MDOC3   = GPIO_PC3,    GPIO_UART_RTSC3         = GPIO_PC3,
		GPIO_PC4 = GPIO_GROUPC | BIT(4),   GPIO_I2C_MSDC4   = GPIO_PC4,   GPIO_SPI_MDIC4   = GPIO_PC4,   GPIO_UART_TXC4   = GPIO_PC4,    GPIO_SPI_DI_OR_I2C_SDC4 = GPIO_PC4,
		GPIO_PC5 = GPIO_GROUPC | BIT(5),   GPIO_I2C_MCKC5   = GPIO_PC5,   GPIO_SPI_MCKC5   = GPIO_PC5,   GPIO_UART_RXC5   = GPIO_PC5,    GPIO_SPI_CK_OR_I2C_CKC5 = GPIO_PC5,
		GPIO_PC6 = GPIO_GROUPC | BIT(6),   GPIO_PWM4C6      = GPIO_PC6,   GPIO_RESERVE_1C6 = GPIO_PC6,   GPIO_RESERVE_2C6 = GPIO_PC6,    GPIO_RESERVE_3C6        = GPIO_PC6,
		GPIO_PC7 = GPIO_GROUPC | BIT(7),   GPIO_SWSC7       = GPIO_PC7,   GPIO_PWM3C7      = GPIO_PC7,   GPIO_RESERVE_2C7 = GPIO_PC7,    GPIO_RESERVE_3C7        = GPIO_PC7,//SWS
		GPIOC_ALL = GPIO_GROUPC | 0x00ff,

		GPIO_PD0 = GPIO_GROUPD | BIT(0),   GPIO_PWM1D0 = GPIO_PD0,     GPIO_UART_CTSD0 = GPIO_PD0,     GPIO_PWM0ND0 = GPIO_PD0,        GPIO_RESERVE_3D0 = GPIO_PD0,
		GPIO_PD1 = GPIO_GROUPD | BIT(1),   GPIO_PWM0D1 = GPIO_PD1,     GPIO_UART_RTSD1 = GPIO_PD1,     GPIO_PWM1ND1 = GPIO_PD1,        GPIO_RESERVE_3D1 = GPIO_PD1,
		GPIO_PD2 = GPIO_GROUPD | BIT(2),   GPIO_PWM3D2 = GPIO_PD2,     GPIO_UART_TXD2 = GPIO_PD2,      GPIO_PWM2D2 = GPIO_PD2,         GPIO_RESERVE_3D2 = GPIO_PD2,
		GPIO_PD3 = GPIO_GROUPD | BIT(3),   GPIO_PWM0D3 = GPIO_PD3,     GPIO_UART_RXD3 = GPIO_PD3,      GPIO_TX_CYC2PAD3 = GPIO_PD3,    GPIO_RESERVE_3D3 = GPIO_PD3,
		GPIOD_ALL = GPIO_GROUPD | 0x00ff,

		GPIO_PE0 = GPIO_GROUPE | BIT(0),
		GPIO_PE1 = GPIO_GROUPE | BIT(1),
		GPIO_PE2 = GPIO_GROUPE | BIT(2),
		GPIO_PE3 = GPIO_GROUPE | BIT(3),
		GPIOE_ALL = GPIO_GROUPE | 0x00ff,

		GPIO_ALL = 0x500,
}GPIO_PinTypeDef;


/**
 *  @brief  Define GPIO Function types
 */
typedef enum
{
	AS_GPIO    = 0,
	AS_AF      = (!0),
	AS_MSPI    = 1,
	AS_SWIRE   = 2,
	AS_UART    = 3,
	AS_I2C_M   = 4,
	AS_I2C_S   = 5,
	AS_SPI_S   = 6,
	AS_ETH_MAC = 7,
	AS_I2S	   = 8,
	AS_SDM	   = 9,
	AS_DMIC    = 10,
	AS_USB	   = 11,
	AS_SWS	   = 12,
	AS_SWM	   = 13,
	AS_ADC	   = 14,

	AS_PWM0 	= 20,
	AS_PWM1		= 21,
	AS_PWM2 	= 22,
	AS_PWM3		= 23,
	AS_PWM4 	= 24,
	AS_PWM5		= 25,
	AS_PWM0_N	= 26,
	AS_PWM1_N	= 27,
	AS_PWM2_N	= 28,
	AS_PWM3_N	= 29,
	AS_PWM4_N	= 30,
	AS_PWM5_N	= 31,
	AS_SPI_M   = 32,
}GPIO_FuncTypeDef;

typedef enum{
	LEVEL_LOW = 0,
	LEVEL_HIGH,
}GPIO_LevelTypeDef;

/**
 *  @brief  Define rising/falling types
 */
typedef enum{
	POL_RISING = 0,
	POL_FALLING,
}GPIO_PolTypeDef;

/**
 *  @brief  Define pull-up/down types
 */
#define GPIO_PULL_NONE		0
#define GPIO_PULL_UP_1M		1
#define GPIO_PULL_UP_10K	2
#define GPIO_PULL_DN_100K	3

#define GPIO_CHN0				GPIO_PB0	//  chn0
#define GPIO_CHN1				GPIO_PB1	//  chn1
#define GPIO_CHN2				GPIO_PB2	//  chn2
#define GPIO_CHN3				GPIO_PB3	//  chn3
#define GPIO_CHN4				GPIO_PB4	//  chn4
#define GPIO_CHN5				GPIO_PB5	//  chn5
#define GPIO_CHN6				GPIO_PB6	//  chn6
#define GPIO_CHN7				GPIO_PB7	//  chn7

#define areg_pull_b0_b3			8
#define B0_1M		(GPIO_PULL_UP_1M)
#define B0_10K		(GPIO_PULL_UP_10K)
#define B0_100K		(GPIO_PULL_DN_100K)
#define B1_1M		(GPIO_PULL_UP_1M << 2)
#define B1_10K		(GPIO_PULL_UP_10K << 2)
#define B1_100K		(GPIO_PULL_DN_100K << 2)
#define B2_1M		(GPIO_PULL_UP_1M << 4)
#define B2_10K		(GPIO_PULL_UP_10K << 4)
#define B2_100K		(GPIO_PULL_DN_100K << 4)
#define B3_1M		(GPIO_PULL_UP_1M << 6)
#define B3_10K		(GPIO_PULL_UP_10K << 6)
#define B3_100K		(GPIO_PULL_DN_100K << 6)

#define areg_pull_b4_b7			9
#define B4_1M		(GPIO_PULL_UP_1M)
#define B4_10K		(GPIO_PULL_UP_10K)
#define B4_100K		(GPIO_PULL_DN_100K)
#define B5_1M		(GPIO_PULL_UP_1M << 2)
#define B5_10K		(GPIO_PULL_UP_10K << 2)
#define B5_100K		(GPIO_PULL_DN_100K << 2)
#define B6_1M		(GPIO_PULL_UP_1M << 4)
#define B6_10K		(GPIO_PULL_UP_10K << 4)
#define B6_100K		(GPIO_PULL_DN_100K << 4)
#define B7_1M		(GPIO_PULL_UP_1M << 6)
#define B7_10K		(GPIO_PULL_UP_10K << 6)
#define B7_100K		(GPIO_PULL_DN_100K << 6)

#define areg_pull_a0_a1a5_a7	10
#define A5_1M		(GPIO_PULL_UP_1M)
#define A5_10K		(GPIO_PULL_UP_10K)
#define A5_100K		(GPIO_PULL_DN_100K)
#define A6_1M		(GPIO_PULL_UP_1M << 2)
#define A6_10K		(GPIO_PULL_UP_10K << 2)
#define A6_100K		(GPIO_PULL_DN_100K << 2)
#define A7_1M		(GPIO_PULL_UP_1M << 4)
#define A7_10K		(GPIO_PULL_UP_10K << 4)
#define A7_100K		(GPIO_PULL_DN_100K << 4)
#define A0_100K		BIT(6)
#define A1_100K		BIT(7)

#define areg_pull_a2_a4c0_c4	11
#define A2_100K		BIT(0)
#define A3_100K		BIT(1)
#define A4_100K		BIT(2)
#define C0_100K		BIT(3)
#define C1_100K		BIT(4)
#define C2_100K		BIT(5)
#define C3_100K		BIT(6)
#define C4_100K		BIT(7)

#define areg_pull_c5_c7d0_d3	12
#define C5_100K		BIT(0)
#define C6_100K		BIT(1)
#define C7_100K		BIT(2)
#define D0_100K		BIT(3)
#define D1_100K		BIT(4)
#define D2_100K		BIT(5)
#define D3_100K		BIT(6)

///  GPIO function, register: 0x5a8 -- 0x5ad
/*
for example:
	reg_gpio_pa_gpio = ~(GPIO_A0 | GPIO_A2);		// diable gpio function
	reg_gpio_pa0pa3_func = A0_PWM0 | A2_PWM1N;	// set specific function
*/
#define A0_PWM0       	0
#define A0_PWM3       	1
#define A0_RX_CYC2LNA 	2
#define A0_RESERVE_3  	3
#define A1_PWM3N      	(0<<2)
#define A1_RESERVE_1  	(1<<2)
#define A1_RESERVE_2  	(2<<2)
#define A1_RESERVE_3    (3<<2)
#define A2_PWM1N        (0<<4)
#define A2_RESERVE_1    (1<<4)
#define A2_RESERVE_2    (2<<4)
#define A2_RESERVE_3    (3<<4)
#define A3_PWM4         (0<<6)
#define A3_SCL       	(1<<6)
#define A3_RESERVE_2    (2<<6)
#define A3_RESERVE_3    (3<<6)
#define A4_PWM2         0
#define A4_SDA       	1
#define A4_RESERVE_2    2
#define A4_RESERVE_3    3
#define A5_PWM2N        (0<<2)
#define A5_SCL       	(1<<2)
#define A5_MCL      	(2<<2)
#define A5_RESERVE_3    (3<<2)
#define A6_PWM4N        (0<<4)
#define A6_SDA       	(1<<4)
#define A6_MDA      	(2<<4)
#define A6_RESERVE_3    (3<<4)
#define A7_PWM0         (0<<6)
#define A7_RESERVE_1    (1<<6)
#define A7_TX_CYC2PA    (2<<6)
#define A7_RESERVE_3    (3<<6)
#define B0_PWM4         0
#define B0_MCN	        1
#define B0_RX_CYC2LNA   2
#define B0_RESERVE_3    3
#define B1_PWM1         (0<<2)
#define B1_MDO          (1<<2)
#define B1_TX_CYC2PA    (2<<2)
#define B1_RESERVE_3    (3<<2)
#define B2_PWM0         (0<<4)
#define B2_MDI          (1<<4)
#define B2_RESERVR_2    (2<<4)
#define B2_RESERVR_3    (3<<4)
#define B3_PWM3N        (0<<6)
#define B3_MCK          (1<<6)
#define B3_PWM2         (2<<6)
#define B3_RESERVE_3    (3<<6)
#define B4_PWM2N        0
#define B4_MCL      	1
#define B4_UTX      	2
#define B4_RESERVE_3    3
#define B5_PWM4         (0<<2)
#define B5_MDA      	(1<<2)
#define B5_URX      	(2<<2)
#define B5_RESERVE_3    (3<<2)
#define B6_PWM0N        (0<<4)
#define B6_MCL      	(1<<4)
#define B6_RTS     		(2<<4)
#define B6_RESERVE_3    (3<<4)
#define B7_PWM1         (0<<6)
#define B7_MDA      	(1<<6)
#define B7_CTS     		(2<<6)
#define B7_RESERVE_3    (3<<6)
#define C0_RESERVE_0    0
#define C0_RESERVE_1    1
#define C0_RESERVE_2    2
#define C0_RESERVE_3    3
#define C1_PWM2N        (0<<2)
#define C1_RESERVE_1    (1<<2)
#define C1_RESERVE_2    (2<<2)
#define C1_RESERVE_3    (3<<2)
#define C2_CN       	(0<<4)
#define C2_PWM1         (1<<4)
#define C2_MCN			(2<<4)
#define C2_CTS     		(3<<4)
#define C3_DO       	(0<<6)
#define C3_PWM0N        (1<<6)
#define C3_MDO      	(2<<6)
#define C3_RTS     		(3<<6)
#define C4_DI_SDA   	0
#define C4_MDA      	1
#define C4_MDI     		2
#define C4_UTX      	3
#define C5_CK_SCL   	(0<<2)
#define C5_MCL      	(1<<2)
#define C5_MCK      	(2<<2)
#define C5_URX      	(3<<2)
#define C6_PWM4         (0<<4)
#define C6_RESERVE_1    (1<<4)
#define C6_RESERVE_2    (2<<4)
#define C6_RESERVE_3    (3<<4)
#define C7_SWS          (0<<6)
#define C7_PWM3         (1<<6)
#define C7_RESERVE_2    (2<<6)
#define C7_RESERVE_3    (3<<6)
#define D0_PWM1         0
#define D0_CTS     		1
#define D0_PWM0N     	2
#define D0_RESERVE_3    3
#define D1_PWM0         (0<<2)
#define D1_RTS     		(1<<2)
#define D1_PWM1N        (2<<2)
#define D1_RESERVE_3    (3<<2)
#define D2_PWM3         (0<<4)
#define D2_UTX      	(1<<4)
#define D2_PWM2         (2<<4)
#define D2_RESERVE_3    (3<<4)
#define D3_PWM0         (0<<6)
#define D3_URX     		(1<<6)
#define D3_TX_CYC2PA    (2<<6)
#define D3_RESERVE_3    (3<<6)

/**
 * @brief      This function servers to initialization all GPIO.
 * @param[in]  none.
 * @return     none.
 */
void gpio_init(void);

/**
 * @brief      This function servers to set the GPIO's function.
 * @param[in]  pin - the special pin.
 * @param[in]  func - the function of GPIO.
 * @return     none.
 */
void gpio_set_func(GPIO_PinTypeDef pin, GPIO_FuncTypeDef func);

/**
 * @brief      This function servers to set the GPIO wakeup.
 * @param[in]  en - enable or disable the GPIO's wakeup (1: enable, 0: disable).
 * @return     none.
 */
static inline void gpio_core_wakeup_enable_all (int en)
{
    if (en) {
        BM_SET(reg_gpio_wakeup_and_irq_en, FLD_GPIO_CORE_WAKEUP_EN);
    }
    else {
        BM_CLR(reg_gpio_wakeup_and_irq_en, FLD_GPIO_CORE_WAKEUP_EN);
    }
}

/**
 * @brief      This function servers to enable IRQ of all GPIO.
 * @param[in]  en - enable or disable the GPIO's IRQ (1: enable, 0: disable).
 * @return     none.
 */
static inline void gpio_core_irq_enable_all (int en)
{
    if (en) {
        BM_SET(reg_gpio_wakeup_and_irq_en, FLD_GPIO_CORE_INTERRUPT_EN);
    }
    else {
        BM_CLR(reg_gpio_wakeup_and_irq_en, FLD_GPIO_CORE_INTERRUPT_EN);
    }
}

/**
 * @brief      This function set the output function of a pin.
 * @param[in]  pin - the pin needs to set the output function
 * @param[in]  value - enable or disable the pin's output function(0: enable, 1: disable)
 * @return     none
 */
static inline void gpio_set_output_en(GPIO_PinTypeDef pin, unsigned int value)
{
	unsigned char	bit = pin & 0xff;
	if(!value){
		BM_SET(reg_gpio_oen(pin), bit);
	}else{
		BM_CLR(reg_gpio_oen(pin), bit);
	}
}

/**
 * @brief      This function set the input function of a pin.
 * @param[in]  pin - the pin needs to set the input function
 * @param[in]  value - enable or disable the pin's input function(0: disable, 1: enable)
 * @return     none
 */
void gpio_set_input_en(GPIO_PinTypeDef pin, unsigned int value);

/**
 * @brief      This function determines whether the output function of a pin is enabled.
 * @param[in]  pin - the pin needs to determine whether its output function is enabled.
 * @return     1: the pin's output function is enabled ;
 *             0: the pin's output function is disabled
 */
static inline int gpio_is_output_en(GPIO_PinTypeDef pin)
{
	return !BM_IS_SET(reg_gpio_oen(pin), pin & 0xff);
}

/**
 * @brief     This function to judge whether a pin's input is enable.
 * @param[in] pin - the pin needs to enable its input.
 * @return    1:enable the pin's input function.
 *            0:disable the pin's input function.
 */
int  gpio_is_input_en(GPIO_PinTypeDef pin);

/**
 * @brief     This function set the pin's output level.
 * @param[in] pin - the pin needs to set its output level
 * @param[in] value - value of the output level(1: high 0: low)
 * @return    none
 */
static inline void gpio_write(GPIO_PinTypeDef pin, unsigned int value)
{
	unsigned char	bit = pin & 0xff;
	if(value){
		BM_SET(reg_gpio_out(pin), bit);
	}else{
		BM_CLR(reg_gpio_out(pin), bit);
	}
}

/**
 * @brief     This function read the pin's input/output level.
 * @param[in] pin - the pin needs to read its level
 * @return    the pin's level(1: high 0: low)
 */
static inline unsigned char gpio_read(GPIO_PinTypeDef pin)
{
	return BM_IS_SET(reg_gpio_in(pin), pin & 0xff);
}

/**
 * @brief     This function set the pin toggle.
 * @param[in] pin - the pin needs to toggle
 * @return    none
 */
static inline void gpio_toggle(GPIO_PinTypeDef pin)
{
	reg_gpio_out(pin) ^= (pin & 0xFF);
}

/**
 * @brief      This function set the pin's driving strength.
 * @param[in]  pin - the pin needs to set the driving strength
 * @param[in]  value - the level of driving strength(1: strong 0: poor)
 * @return     none
 */
void gpio_set_data_strength(GPIO_PinTypeDef pin, unsigned int value);

/**
 * @brief     This function set a gpio_pad pull-up/down resistor(NOT ALL THE PINS).
 * @param[in] gpio - the pin needs to set its pull-up/down resistor.Just for (PA<7:5>,PB<7:0>).
 * @param[in] up_down - the type of the pull-up/down resistor
 * @return    none
 */
void gpio_set_up_down_resistor(GPIO_PinTypeDef gpio, unsigned short up_down);

/**
 * @brief     This function set a gpio_core PULL_UP resistor.this function include all GPIOs.
 * @param[in] gpio - the pin needs to set its pull-up resistor.
 * @return    none
 */
void gpio_set_up_30k(GPIO_PinTypeDef pin);

/**
 * @brief     This function set a pin's IRQ polarity.
 * @param[in] pin - the pin needs to enable its IRQ
 * @param[in] falling - value of the edge polarity(1: falling edge 0: rising edge)
 * @return    none
 */
static inline void gpio_set_interrupt_pol(GPIO_PinTypeDef pin, GPIO_PolTypeDef falling)
{
	unsigned char	bit = pin & 0xff;
	if(falling){
		BM_SET(reg_gpio_pol(pin), bit);
	}else{
		BM_CLR(reg_gpio_pol(pin), bit);
	}
}

/**
 * @brief     This function set a GPIO's IRQ source and its polarity.
 * @param[in] pin - the pin needs to enable its IRQ
 * @param[in] falling - value of the edge polarity(1: falling edge 0: rising edge)
 * @return    none
 */
static inline void gpio_set_interrupt_risc0(GPIO_PinTypeDef pin, GPIO_PolTypeDef falling)
{
	unsigned char	bit = pin & 0xff;
	BM_SET(reg_gpio_irq_risc0_en(pin), bit);
	reg_irq_mask |= FLD_IRQ_GPIO_RISC0_EN;
	if(falling){
		BM_SET(reg_gpio_pol(pin), bit);
	}else{
		BM_CLR(reg_gpio_pol(pin), bit);
	}
}

/**
 * @brief     This function set a GPIO's IRQ source and its polarity.
 * @param[in] pin - the pin needs to enable its IRQ
 * @param[in] falling - value of the edge polarity(1: falling edge 0: rising edge)
 * @return    none
 */
static inline void gpio_set_interrupt_risc1(GPIO_PinTypeDef pin, GPIO_PolTypeDef falling)
{
	unsigned char	bit = pin & 0xff;
	BM_SET(reg_gpio_irq_risc1_en(pin), bit);
	reg_irq_mask |= FLD_IRQ_GPIO_RISC1_EN;
	if(falling){
		BM_SET(reg_gpio_pol(pin), bit);
	}else{
		BM_CLR(reg_gpio_pol(pin), bit);
	}
}

/**
 * @brief     This function set a GPIO's IRQ source and its polarity.
 * @param[in] pin - the pin needs to enable its IRQ
 * @param[in] falling - value of the edge polarity(1: falling edge 0: rising edge)
 * @return    none
 */
static inline void gpio_set_interrupt_risc2(GPIO_PinTypeDef pin, GPIO_PolTypeDef falling)
{
	unsigned char	bit = pin & 0xff;
	BM_SET(reg_gpio_irq_risc2_en(pin), bit);
	reg_irq_mask |= FLD_IRQ_GPIO_RISC2_EN;
	if(falling){
		BM_SET(reg_gpio_pol(pin), bit);
	}else{
		BM_CLR(reg_gpio_pol(pin), bit);
	}
}

/**
 * @brief     This function servers to read a pin's cache value.
 * @param[in] pin - the pin needs to be read.
 * @param[in] p   - value of pointer to store the result of read.
 * @return    the value of cache.
 */
static inline unsigned char gpio_read_cache(GPIO_PinTypeDef pin, unsigned char *p)
{
	return p[pin>>8] & (pin & 0xff);
}

/**
 * @brief     This function servers to read all GPIOs.
 * @param[in] p   - value of pointer stands for different GPIO groups.
 * @return    none.
 */
static inline void gpio_read_all(unsigned char *p)
{
	p[0] = REG_ADDR8(0x580);//PA
	p[1] = REG_ADDR8(0x588);//PB
	p[2] = REG_ADDR8(0x590);//PC
	p[3] = REG_ADDR8(0x598);//PD
}

/**
 * @brief     This function servers to enable a GPIO as interrupt source.
 * @param[in] pin - the pin needs to be set.
 * @param[in] en  - enable or disable the pin's function of interrupt(1: enable, 0: disable).
 * @return    none.
 */
static inline void gpio_en_interrupt(GPIO_PinTypeDef pin, int en)
{
	unsigned char	bit = pin & 0xff;
	if(en){
		BM_SET(reg_gpio_irq_wakeup_en(pin), bit);
	}
	else{
		BM_CLR(reg_gpio_irq_wakeup_en(pin), bit);
	}
}

/**
 * @brief     This function servers to set a GPIO as interrupt source and include polarity config.
 * @param[in] pin - the pin needs to be set.
 * @param[in] falling - value of the GPIO's polarity(1: falling edge 0: rising edge)
 * @return    none.
 */
static inline void gpio_set_interrupt(GPIO_PinTypeDef pin, GPIO_PolTypeDef falling)
{
	unsigned char	bit = pin & 0xff;
	BM_SET(reg_gpio_irq_wakeup_en(pin), bit);
	reg_irq_mask |= FLD_IRQ_GPIO_EN ;
	reg_gpio_wakeup_irq |= FLD_GPIO_CORE_INTERRUPT_EN;
	if(falling){
		BM_SET(reg_gpio_pol(pin), bit);
	}else{
		BM_CLR(reg_gpio_pol(pin), bit);
	}
	BM_SET(reg_gpio_irq_wakeup_en(pin), bit);
}

/**
 * @brief     This function servers to enable a pin's IRQ.
 * @param[in] pin - the pin needs to enable its IRQ
 * @param[in] en - enable or disable the pin's function of interrupt(1: enable, 0: disable).
 * @return    none
 */
static inline void gpio_en_interrupt_risc0(GPIO_PinTypeDef pin, int en)
{
	unsigned char	bit = pin & 0xff;
	if(en){
		BM_SET(reg_gpio_irq_risc0_en(pin), bit);
	}
	else{
		BM_CLR(reg_gpio_irq_risc0_en(pin), bit);
	}
}

/**
 * @brief     This function servers to enable a pin's IRQ.
 * @param[in] pin - the pin needs to enable its IRQ
 * @param[in] en - enable or disable the pin's function of interrupt(1: enable, 0: disable).
 * @return    none
 */
static inline void gpio_en_interrupt_risc1(GPIO_PinTypeDef pin, int en)
{
	unsigned char	bit = pin & 0xff;
	if(en){
		BM_SET(reg_gpio_irq_risc1_en(pin), bit);
	}
	else{
		BM_CLR(reg_gpio_irq_risc1_en(pin), bit);
	}
}

/**
 * @brief     This function servers to enable a pin's IRQ.
 * @param[in] pin - the pin needs to enable its IRQ
 * @param[in] en - enable or disable the pin's function of interrupt(1: enable, 0: disable).
 * @return    none
 */
static inline void gpio_en_interrupt_risc2(GPIO_PinTypeDef pin, int en)
{
	unsigned char	bit = pin & 0xff;
	if(en){
		BM_SET(reg_gpio_irq_risc2_en(pin), bit);
	}
	else{
		BM_CLR(reg_gpio_irq_risc2_en(pin), bit);
	}
}

/**
 * @brief     This function servers to clear a pin's IRQ.
 * @param[in] pin - the pin needs to clear its IRQ
 * @return    none
 */
static inline void gpio_clr_interrupt_risc0(GPIO_PinTypeDef pin){
	unsigned char	bit = pin & 0xff;
	BM_CLR(reg_gpio_irq_risc0_en(pin), bit);
}

/**
 * @brief     This function servers to clear a pin's IRQ.
 * @param[in] pin - the pin needs to clear its IRQ
 * @return    none
 */
static inline void gpio_clr_interrupt_risc1(GPIO_PinTypeDef pin){
	unsigned char	bit = pin & 0xff;
	BM_CLR(reg_gpio_irq_risc1_en(pin), bit);
}

/**
 * @brief     This function servers to clear a pin's IRQ.
 * @param[in] pin - the pin needs to clear its IRQ
 * @return    none
 */
static inline void gpio_clr_interrupt_risc2(GPIO_PinTypeDef pin){
	unsigned char	bit = pin & 0xff;
	BM_CLR(reg_gpio_irq_risc2_en(pin), bit);
}

/**
 * @brief      This function servers to set the specified GPIO as high resistor.
 * @param[in]  pin  - select the specified GPIO
 * @return     none.
 */
void gpio_shutdown(GPIO_PinTypeDef pin);

/**
 * @brief      This function servers to clear the GPIO's IRQ function.
 * @param[in]  none.
 * @return     none.
 */
void gpio_clear_gpio_irq_flag(void);

#endif

/** \defgroup GP5  GPIO Examples
 *
 * 	@{
 */
/** \defgroup GP5  GPIO Examples
 *
 * 	@{
 */

/*! \page gpio Table of Contents
	- [API-GPIO-CASE1:GPIO IRQ](#GPIO_IRQ)
	- [API-GPIO-CASE2:GPIO IRQ RSIC0](#GPIO_IRQ_RSIC0)
	- [API-GPIO-CASE3:GPIO IRQ RSIC1](#GPIO_IRQ_RSIC1)
	- [API-GPIO-CASE4:GPIO READ/WRITE/TOGGLE](#GPIO_TOGGLE)
	- [API-GPIO-CASE5:GPIO HIGH RESISTOR](#GPIO_HIGH_RESISTOR)

\n
Variables used in the following cases are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}

#define LED1     		        GPIO_PD0
#define LED2     		        GPIO_PD3
#define LED3     		        GPIO_PD4
#define LED4     		        GPIO_PD5

#define SW1      		        GPIO_PD1
#define SW2      		        GPIO_PD2

volatile unsigned int gpio_irq_cnt;

~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=GPIO_IRQ> API-GPIO-CASE1:GPIO IRQ </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | if((reg_irq_src & FLD_IRQ_GPIO_EN)==FLD_IRQ_GPIO_EN) ||| determine whether interrupt flag is right | 2019-1-10 |
| ^ | reg_irq_src &Iota;= FLD_IRQ_GPIO_EN ||| clear interrrupt flag | ^ |
| ^ | gpio_irq_cnt++  ||| Interrupt processing | ^ |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | gpio_set_func() | gpio_set_func(SW1 ,AS_GPIO) | set pin as GPIO | ^ |
| ^ | ^ | gpio_set_output_en() | gpio_set_output_en(SW1, 0) | disable GPIO output  | ^ |
| ^ | ^ | gpio_set_input_en() | gpio_set_input_en(SW1 ,1) | enable GPIO input | ^ |
| ^ | ^ | gpio_set_up_down_resistor() | gpio_set_up_down_resistor(SW1, GPIO_PULL_UP_10K) | enable GPIO 10k resistor pull-up | ^ |
| ^ | ^ | gpio_set_interrupt() | gpio_set_interrupt(SW1, POL_FALLING) | set pin as GPIO interrupt  | ^ |
| ^ | main_loop() | None || Main program loop | ^ |

<h1 id=GPIO_IRQ_RSIC0> API-GPIO-CASE2:GPIO IRQ RSIC0 </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | if((reg_irq_src & FLD_IRQ_GPIO_RISC0_EN)==FLD_IRQ_GPIO_RISC0_EN) ||| determine whether interrupt flag is right | 2019-1-10 |
| ^ | reg_irq_src &Iota;= FLD_IRQ_GPIO_RISC0_EN ||| clear interrrupt flag | ^ |
| ^ | gpio_irq_cnt++  ||| Interrupt processing | ^ |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | gpio_set_func() | gpio_set_func(SW1 ,AS_GPIO) | set pin as GPIO | ^ |
| ^ | ^ | gpio_set_output_en() | gpio_set_output_en(SW1, 0) | disable GPIO output  | ^ |
| ^ | ^ | gpio_set_input_en() | gpio_set_input_en(SW1 ,1) | enable GPIO input | ^ |
| ^ | ^ | gpio_set_up_down_resistor() | gpio_set_up_down_resistor(SW1, GPIO_PULL_UP_10K) | enable GPIO 10k resistor pull-up | ^ |
| ^ | ^ | gpio_set_interrupt_risc0() | gpio_set_interrupt_risc0(SW1, POL_FALLING) | set pin as GPIO interrupt risc0  | ^ |
| ^ | main_loop() | None || Main program loop | ^ |


<h1 id=GPIO_IRQ_RSIC1> API-GPIO-CASE3:GPIO IRQ RSIC1 </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | if((reg_irq_src & FLD_IRQ_GPIO_RISC1_EN)==FLD_IRQ_GPIO_RISC1_EN) ||| determine whether interrupt flag is right | 2019-1-10 |
| ^ | reg_irq_src &Iota;= FLD_IRQ_GPIO_RISC1_EN ||| clear interrrupt flag | ^ |
| ^ | gpio_irq_cnt++  ||| Interrupt processing | ^ |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | gpio_set_func() | gpio_set_func(SW1 ,AS_GPIO) | set pin as GPIO | ^ |
| ^ | ^ | gpio_set_output_en() | gpio_set_output_en(SW1, 0) | disable GPIO output  | ^ |
| ^ | ^ | gpio_set_input_en() | gpio_set_input_en(SW1 ,1) | enable GPIO input | ^ |
| ^ | ^ | gpio_set_up_down_resistor() | gpio_set_up_down_resistor(SW1, GPIO_PULL_UP_10K) | enable GPIO 10k resistor pull-up | ^ |
| ^ | ^ | gpio_set_interrupt_risc1() | gpio_set_interrupt_risc1(SW1, POL_FALLING) | set pin as GPIO interrupt risc1 | ^ |
| ^ | main_loop() | None || Main program loop | ^ |


<h1 id=GPIO_TOGGLE> API-GPIO-CASE4:GPIO TOGGLE </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | gpio_set_func() |gpio_set_func(LED1 ,AS_GPIO) | set pin as GPIO | ^ |
| ^ | ^ | gpio_set_output_en() | gpio_set_output_en(LED1, 1) | disable GPIO output  | ^ |
| ^ | ^ | gpio_set_input_en() | gpio_set_input_en(LED1 ,0) | enable GPIO input | ^ |
| ^ | ^ | gpio_write(), gpio_read() | gpio_write(LED1, !gpio_read(LED1)) | toggle GPIO | ^ |
| ^ | ^ | gpio_toggle() | gpio_toggle(LED1) | toggle GPIO | ^ |
| ^ | main_loop() | None || Main program loop | ^ |

<h1 id=GPIO_HIGH_RESISTOR> API-GPIO-CASE5:GPIO HIGH RESISTOR </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_1M) || RF mode initialization [**optional**] | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | gpio_shutdown() | gpio_shutdown(GPIO_ALL) | set all GPIOs as high resistor | ^ |
| ^ | main_loop() | None || Main program loop | ^ |

<h1> History Record </h1>

| Date | Description | Author |
| :--- | :---------- | :----- |
| 2019-1-10 | initial release | LJW |


*/

 /** @}*/ //end of GP5








