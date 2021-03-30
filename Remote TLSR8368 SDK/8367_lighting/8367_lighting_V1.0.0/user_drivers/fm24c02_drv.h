/*
 * fm24c02_drv.h
 *
 *  Created on: Sep 17, 2013
 *      Author: xuzhen
 */

#ifndef FM24C02_DRV_H_
#define FM24C02_DRV_H_

#define FM24C02ID_RD        0xA1
#define FM24C02ID_WR        0xA0
#define FM24C02ID           0xA0
#define SET_WP_LOW			gpio_write(GPIO_PWM2,0)

#if 0 //USE_IIC_HW_IF
#include "../../proj/tl_common.h"
#include "../../proj/mcu/gpio.h"

#define		TL_I2C_ADDRESS_16BIT		1
//////////////////////////////////////////////////////////////////////
///// tl_i2c_nb: non-blocking access
//////////////////////////////////////////////////////////////////////

static inline int tl_i2c_busy () {
	return (reg_i2c_status & FLD_I2C_CMD_BUSY);
}

static inline void tl_i2c_nb_write_adr8_dat (int adr, int dat) {
	reg_i2c_dat_ctrl = adr | (dat << 16) |
			((FLD_I2C_CMD_START | FLD_I2C_CMD_STOP |
				FLD_I2C_CMD_ID | FLD_I2C_CMD_ADR | FLD_I2C_CMD_DI) << 24);
}

static inline void tl_i2c_nb_write_start8 (int adr, int stop) {
	reg_i2c_dat_ctrl = adr |
			((FLD_I2C_CMD_START | (stop ? FLD_I2C_CMD_STOP : 0) |
				FLD_I2C_CMD_ID | FLD_I2C_CMD_ADR ) << 24);
}

static inline void tl_i2c_nb_write_byte (int dat, int stop) {
	reg_i2c_dat_ctrl = (dat<<8) |
			((FLD_I2C_CMD_DO | (stop ? FLD_I2C_CMD_STOP : 0)) << 24);
}

///////////// for read command ///////////////////////////////////////////////
static inline void tl_i2c_nb_read_byte () {
	reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_ID | FLD_I2C_CMD_READ_ID |
			FLD_I2C_CMD_DI | FLD_I2C_CMD_STOP;
}

static inline void tl_i2c_nb_read_start (int stop) {
	reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_ID | FLD_I2C_CMD_DI |
			FLD_I2C_CMD_READ_ID | (stop ? FLD_I2C_CMD_STOP | FLD_I2C_CMD_NAK : 0);
}

static inline void tl_i2c_nb_read_next (int stop) {
	reg_i2c_ctrl = FLD_I2C_CMD_DI | FLD_I2C_CMD_READ_ID |
			(stop ? FLD_I2C_CMD_STOP | FLD_I2C_CMD_NAK : 0);
}


void tl_i2c_init (int id, int divider, int address_16bit);
int tl_i2c_nb_write (int adr, u8 * buff, int len);
int tl_i2c_nb_read (int adr, u8 * buff, int len);

//data block: offset is buff, length is len
//write data to address
static inline int tl_i2c_write (int adr, u8 * buff, int len) {
	int ret;
	do {
		ret = tl_i2c_nb_write (adr, buff, len);
	} while (ret == 0);
	return ret;
}
//data block: offset is adr, length is len.
//read data to buff.
static inline int tl_i2c_read (int adr, u8 * buff, int len) {
	int ret;
	do {
		ret = tl_i2c_nb_read (adr, buff, len);
	} while (ret == 0);
	return ret;
}
#endif

void fm24c02_init_func(unsigned int scl,unsigned int sda);
unsigned int fm24c02_read(int address);
void fm24c02_write(int address, int data);

#endif /* FM24C02_DRV_H_ */
