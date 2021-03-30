/********************************************************************************************************
 * @file     i2c.c
 *
 * @brief    This is the source file for TLSR8231
 *
 * @author	 Telink
 * @date     May 24, 2019
 *
 * @par      Copyright (c) 2019, Telink Semiconductor (Shanghai) Co., Ltd.
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
#ifndef I2C_H
#define I2C_H
#include "gpio.h"

typedef enum {
	I2C_SLAVE_DMA = 0,
	I2C_SLAVE_MAP = 1,
}I2C_SlaveMode;

typedef enum {
	I2C_GPIO_GROUP_A5A6 = 0x00,
	I2C_GPIO_GROUP_B4B5 = 0x01,
	I2C_GPIO_GROUP_B6B7 = 0x02,
	I2C_GPIO_GROUP_C5C4 = 0x03,
	I2C_GPIO_GROUP_A3A4 = 0x04,

}I2C_GPIO_GroupTypeDef;

/**
 * This macro definitions to select a pin port for I2C interface.for example: CFG_PA3_CK(), set PA3 as IIC SCK function
 *    CFG_PA5_MCK(); CFG_PA6_MSD();
 *    CFG_PB4_MCK(); CFG_PB5_MSD();
 *    CFG_PB6_MCK(); CFG_PB7_MSD();
 *    CFG_PC4_MCK(); CFG_PC5_MSD();
 *
 *    CFG_PA3_CK();  CFG_PA4_SD();
 *    CFG_PA5_CK();  CFG_PA6_SD();
 */
enum{
	PA3_CK = BIT_RNG(6,7),//1
	PA4_SD = BIT_RNG(0,1),//1
	PA5_MCK_CK  = BIT_RNG(2, 3),//MCK=2,CK=1
	PA6_MSD_SD  = BIT_RNG(4, 5),//MSD=2,SD=1
	PB4_MCK = BIT_RNG(0,1),//1
	PB5_MSD = BIT_RNG(2,3),//1
	PB6_MCK= BIT_RNG(4,5),//1
	PB7_MSD  = BIT_RNG(6,7),//1
	PC4_MSD=BIT_RNG(0,1),//1
	PC5_MCK=BIT_RNG(2,3),//1
};

#define    CFG_PA3_CK()	do{\
									gpio_set_func(GPIO_PA3,!AS_GPIO);\
									reg_goio_gpa1_setting  &= (~PA3_CK);\
									reg_goio_gpa1_setting |= BIT(6);\
									gpio_set_up_30k(GPIO_PA3);\
									gpio_set_input_en(GPIO_PA3, 1);\
								}while(0)

#define    CFG_PA4_SD()	do{\
									gpio_set_func(GPIO_PA4,!AS_GPIO);\
									reg_goio_gpa2_setting  &= (~PA4_SD);\
									reg_goio_gpa2_setting |= BIT(0);\
									gpio_set_up_30k(GPIO_PA4);\
									gpio_set_input_en(GPIO_PA4, 1);\
								}while(0)
#define    CFG_PA5_CK()	do{\
									gpio_set_func(GPIO_PA5,!AS_GPIO);\
									reg_goio_gpa2_setting  &= (~PA5_MCK_CK) ;\
									reg_goio_gpa2_setting |= BIT(2);\
									gpio_set_up_down_resistor(GPIO_PA5,GPIO_PULL_UP_10K);\
								    gpio_set_input_en(GPIO_PA5, 1);\
								}while(0)
#define    CFG_PA5_MCK()	do{\
									gpio_set_func(GPIO_PA5,!AS_GPIO);\
									reg_goio_gpa2_setting  &= (~PA5_MCK_CK) ;\
									reg_goio_gpa2_setting |= BIT(3);\
									gpio_set_up_down_resistor(GPIO_PA5,GPIO_PULL_UP_10K);\
								    gpio_set_input_en(GPIO_PA5, 1);\
								}while(0)
#define    CFG_PA6_SD()	do{\
									gpio_set_func(GPIO_PA6,!AS_GPIO);\
									reg_goio_gpa2_setting  &= (~PA6_MSD_SD);\
									reg_goio_gpa2_setting |= BIT(4);\
									gpio_set_up_down_resistor(GPIO_PA6,GPIO_PULL_UP_10K);\
								    gpio_set_input_en(GPIO_PA6, 1);\
								}while(0)

#define    CFG_PA6_MSD()	do{\
									gpio_set_func(GPIO_PA6,!AS_GPIO);\
									reg_goio_gpa2_setting  &= (~PA6_MSD_SD) ;\
									reg_goio_gpa2_setting |= BIT(5);\
									gpio_set_up_down_resistor(GPIO_PA6,GPIO_PULL_UP_10K);\
									gpio_set_input_en(GPIO_PA6, 1);\
								}while(0)
#define    CFG_PB4_MCK()	do{\
									gpio_set_func(GPIO_PB4,!AS_GPIO);\
									reg_goio_gpb2_setting  &= (~PB4_MCK) ;\
									reg_goio_gpb2_setting |= BIT(0);\
									gpio_set_up_down_resistor(GPIO_PB4,GPIO_PULL_UP_10K);\
									gpio_set_input_en(GPIO_PB4, 1);\
								}while(0)
#define    CFG_PB5_MSD()	do{\
									gpio_set_func(GPIO_PB5,!AS_GPIO);\
									reg_goio_gpb2_setting  &= (~PB5_MSD) ;\
									reg_goio_gpb2_setting |= BIT(2);\
									gpio_set_up_down_resistor(GPIO_PB5,GPIO_PULL_UP_10K);\
									gpio_set_input_en(GPIO_PB5, 1);\
								}while(0)
#define    CFG_PB6_MCK()	do{\
									gpio_set_func(GPIO_PB6,!AS_GPIO);\
									reg_goio_gpb2_setting  &= (~PB6_MCK) ;\
									reg_goio_gpb2_setting |= BIT(4);\
									gpio_set_up_down_resistor(GPIO_PB6,GPIO_PULL_UP_10K);\
								    gpio_set_input_en(GPIO_PB6, 1);\
								}while(0)
#define    CFG_PB7_MSD()	do{\
									gpio_set_func(GPIO_PB7,!AS_GPIO);\
									reg_goio_gpb2_setting  &= (~PB7_MSD) ;\
									reg_goio_gpb2_setting |= BIT(6);\
									gpio_set_up_down_resistor(GPIO_PB7,GPIO_PULL_UP_10K);\
								    gpio_set_input_en(GPIO_PB7, 1);\
								}while(0)
#define    CFG_PC4_MSD()	do{\
									gpio_set_func(GPIO_PC4,!AS_GPIO);\
									reg_goio_gpc2_setting  &= (~PC4_MSD) ;\
									reg_goio_gpc2_setting |= BIT(0);\
									BM_SET(REG_ADDR8(0x592), BIT(4));\
									BM_SET(REG_ADDR8(0x593),BIT(4));\
									BM_SET(REG_ADDR8(0x591), BIT(4));\
                                }while(0)
#define    CFG_PC5_MCK()	do{\
									gpio_set_func(GPIO_PC5,!AS_GPIO);\
									reg_goio_gpc2_setting  &= (~PC5_MCK) ;\
									reg_goio_gpc2_setting |= BIT(2);\
									BM_SET(REG_ADDR8(0x592), BIT(5));\
								    BM_SET(REG_ADDR8(0x593),BIT(5));\
								    BM_SET(REG_ADDR8(0x591), BIT(5));\
								}while(0)
/**
 * @brief This function reset I2C module.
 * @param[in] none
 * @return none
 */
static inline void i2c_reset(void)
{
	reg_rst0 |= FLD_RST0_I2C;
	reg_rst0 &= (~FLD_RST0_I2C);
}
/**
 * @brief This function serves to set id of I2C module.
 * @param[in] id - this id is fixed id for slave device.For master device, this id is set to access different slave devices.
 * @return none
 */
static inline void i2c_set_id(unsigned char SlaveID)
{
    reg_i2c_id	  = SlaveID; //slave address
}
/**
 * @brief      This function serves to select a pin port for I2C interface.
 * @param[in]  PinGrp - the pin port selected as I2C interface pin port.
 * @return     none
 */
//void i2c_set_pin(I2C_GPIO_GroupTypeDef i2c_pin_group);

/**
 * @brief      This function set the id of slave device and the speed of I2C interface
 *             note: the param ID contain the bit of writting or reading.
 *             eg:the parameter 0x5C. the reading will be 0x5D and writting 0x5C.
 * @param[in]  SlaveID - the id of slave device.it contains write or read bit,the lsb is write or read bit.
 *                       ID|0x01 indicate read. ID&0xfe indicate write.
 * @param[in]  DivClock - the division factor of I2C clock,
 *             I2C clock = System clock / (4*DivClock);if the datasheet you look at is 2*,pls modify it.
 * @return     none
 */
void i2c_master_init(unsigned char SlaveID, unsigned char DivClock);
/**
 *  @brief      the function config the ID of slave and mode of slave.
 *  @param[in]  device_ID - it contains write or read bit,the lsb is write or read bit.
 *              ID|0x01 indicate read. ID&0xfe indicate write.
 *  @param[in]  mode - set slave mode. slave has two modes, one is DMA mode, the other is MAPPING mode.
 *  @param[in]  pMapBuf - if slave mode is MAPPING, set the first address of buffer master write or read slave.
 *  @return     none
 */

void i2c_slave_init(unsigned char device_ID,I2C_SlaveMode mode,unsigned char * pMapBuf);
/**
 * @brief      This function serves to write one byte to the slave device at the specified address
 * @param[in]  Addr - i2c slave address where the one byte data will be written
 * @param[in]  AddrLen - length in byte of the address, which makes this function is
 *             compatible for slave device with both one-byte address and two-byte address
 * @param[in]  Data - the one byte data will be written via I2C interface
 * @return     none
 */
void i2c_dma_write_byte(unsigned int Addr, unsigned int AddrLen, unsigned char Data);
/**
 * @brief      This function serves to read one byte from the slave device at the specified address
 * @param[in]  Addr - i2c slave address where the one byte data will be read
 * @param[in]  AddrLen - length in byte of the address, which makes this function is
 *             compatible for slave device with both one-byte address and two-byte address
 * @return     the one byte data read from the slave device via I2C interface
 */
unsigned char i2c_dma_read_byte(unsigned int Addr, unsigned int AddrLen);
/**
 *  @brief      This function serves to write a packet of data to the specified address of slave device
 *  @param[in]  Addr - the register that master write data to slave in. support one byte and two bytes. i.e param2 AddrLen may be 1 or 2.
 *  @param[in]  AddrLen - the length of register. enum 0 or 1 or 2 or 3. based on the spec of i2c slave.
 *  @param[in]  dataBuf - the first SRAM buffer address to write data to slave in.
 *  @param[in]  dataLen - the length of data master write to slave.
 *  @return     none
 */

void i2c_dma_write_buff (unsigned int Addr, unsigned int AddrLen, unsigned char * dataBuf, int dataLen);
/**
 * @brief      This function serves to read a packet of data from slave device working in mapping mode
 * @param[in]  Addr - the register that master write data to slave in. support one byte and two bytes. i.e param2 AddrLen may be 1 or 2.
 * @param[in]  dataBuf - the first address of SRAM buffer master store data in.
 * @param[in]  dataLen - the length of data master read from slave.
 * @return     none.
 */
void i2c_dma_read_buff(unsigned int Addr, unsigned int AddrLen, unsigned char * dataBuf, int dataLen);

/**
 *  @brief      This function serves to write a packet of data to slave device working in mapping mode
 *  @param[in]  Addr - the register that master write data to slave in. support one byte and two bytes. i.e param2 AddrLen may be 1 or 2.
 *  @param[in]  dataBuf - the first SRAM buffer address to write data to slave in.
 *  @param[in]  dataLen - the length of data master write to slave.
 *  @return     none
 */
void i2c_map_write_buff(unsigned char * dataBuf, int dataLen);

/**
 * @brief      This function serves to read a packet of data from slave device working in mapping mode
 * @param[in]  dataBuf - the first address of SRAM buffer master store data in.
 * @param[in]  dataLen - the length of data master read from slave.
 * @return     none.
 */
void i2c_map_read_buff(unsigned char * dataBuf, int dataLen);

#endif


