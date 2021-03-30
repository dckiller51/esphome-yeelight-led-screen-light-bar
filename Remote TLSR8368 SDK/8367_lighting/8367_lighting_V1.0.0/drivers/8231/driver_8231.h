/********************************************************************************************************
 * @file     driver_8231.h
 *
 * @brief    This is the header file for TLSR8231
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
 *         
 *******************************************************************************************************/
#pragma once

#include "drivers/8231/bsp.h"
#include "drivers/8231/aes.h"
#include "drivers/8231/analog.h"
#include "drivers/8231/compiler.h"
#include "drivers/8231/register.h"
#include "drivers/8231/gpio.h"
#include "drivers/8231/pwm.h"
#include "drivers/8231/irq.h"
#include "drivers/8231/clock.h"
#include "drivers/8231/flash.h"
#include "drivers/8231/rf_drv.h"
#include "drivers/8231/pm.h"
#include "drivers/8231/adc.h"
#include "drivers/8231/i2c.h"
#include "drivers/8231/spi.h"
#include "drivers/8231/uart.h"
#include "drivers/8231/watchdog.h"
#include "drivers/8231/qdec.h"
#include "drivers/8231/dma.h"
#include "drivers/8231/timer.h"
#include "drivers/8231/emi.h"
#include "drivers/8231/otp.h"
/*! \mainpage API User guide for TLSR8231F512
\n
__Keyword:__ \n
Bluetooth Low Energy;BLE Mesh;6LoWPAN;Thread;Zigbee;RF4CE;HomeKit;2.4GHz;Features;Package;Pin layout;Memory;MCU;Working modes; Wakeup sources;RF Transceiver;Clock;Timers; Interrupt;PWM;Audio;QDEC;ADC;PGA;Temperature sensor;Low power comparator; AES; Electrical specification; \n
\n
__Brief:__\n
This manual is dedicated for Telink BLE + IEEE802.15.4 multi-standard SoC TLSR8231F512. In this manual,key features, working mode,main modules, electrical specification and application of the TLSR8231F512 are introduced. \n
\n
__Published by__ \n
__Telink Semiconductor__ \n
\n
__Bldg 3, 1500 Zuchongzhi Rd,__ \n
__Zhangjiang Hi-Tech Park, Shanghai, China__ \n
\n
__Telink Semiconductor__ \n
__All Right Reserved__ \n
\n
__Legal Disclaimer:__ \n
    Telink Semiconductor reserves the right to make changes without further notice to any products herein to improve reliability, function or design. Telink Semiconductor disclaims any and all liability for any errors, inaccuracies or incompleteness contained herein or in any other disclosure relating to any product.\n
    Telink Semiconductor does not assume any liability arising out of the application or use of any product or circuit described herein; neither does it convey any license under its patent rights, nor the rights of others \n
    This document is provided as-is. Telink Semiconductor reserves the right to make improvements without further notice to this document or any products herein. This document may contain technical inaccuracies or typographical errors. Telink Semiconductor disclaims any and all liability for any errors, inaccuracies or incompleteness contained herein.
	\n
	Copyright (c) 2018 Telink Semicondcutor (Shanghai) Ltd, Co.\n
	\n
__Information:__ \n
For further information on the technology, product and business term, please contact [Telink Semiconductor Company](http://www.telink-semi.com "Telink") \n
For sales or technical support, please send email to the address of:\n
<telinkcnsales@telink-semi.com> \n
<telinkcnsupport@telink-semi.com> \n
For more details about this SoC, please look through [Datasheet for Telink BLE+IEEE802.15.4 Multi-Standard Wireless SoC TLSR8231](http://wiki.telink-semi.cn/doc/ds/DS_TLSR8231-E_Datasheet%20for%20Telink%20BLE%20IEEE802.15.4%20Multi-Standard%20Wireless%20SoC%20TLSR8231.pdf) \n
\n
__Revision History:__ \n

| Version | Major Changes   | Date         | Author |
| :-----: | :-------------: | :----------: | :----: |
| 1.0.0   | initial release | Dec. 11 2018 | LJW/ZJY/ZL/LR/SP/YY |

\n

*/
