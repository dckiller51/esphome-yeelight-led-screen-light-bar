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
#include "clock.h"
#include "i2c.h"
#include "gpio.h"

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
void i2c_master_init(unsigned char SlaveID, unsigned char DivClock)
{
    reg_i2c_speed = DivClock; //i2c clock = system_clock/(4*DivClock)
    reg_i2c_id	  = MASK_VAL(FLD_I2C_ID, SlaveID);//slave address
    reg_i2c_mode &= ~(FLD_I2C_MASTER_EN|FLD_I2C_SLAVE_EN);
    reg_i2c_mode |= FLD_I2C_MASTER_EN; //enable master mode

    reg_clk_en0 |= FLD_CLK0_I2C_EN;    //enable i2c clock
    reg_spi_sp  &= ~FLD_SPI_ENABLE;   //force PADs act as I2C; i2c and spi share the hardware of IC

}

/**
 *  @brief      the function config the ID of slave and mode of slave.
 *  @param[in]  device_ID - it contains write or read bit,the lsb is write or read bit.
 *              ID|0x01 indicate read. ID&0xfe indicate write.
 *  @param[in]  mode - set slave mode. slave has two modes, one is DMA mode, the other is MAPPING mode.
 *  @param[in]  pBuf - if slave mode is MAPPING, set the first address of buffer master write or read slave.
 *  @return     none
 */
void i2c_slave_init(unsigned char device_ID,I2C_SlaveMode mode,unsigned char * pMapBuf)
{
	reg_i2c_slave_id = MASK_VAL(FLD_I2C_ID,device_ID); 	 //slave device id


	reg_clk_en0 |= FLD_CLK0_I2C_EN;    //enable i2c clock
	reg_spi_sp  &= ~FLD_SPI_ENABLE;   //force PADs act as I2C; i2c and spi share the hardware of IC


	reg_i2c_mode &= (~FLD_I2C_MASTER_EN); //enable slave mode
	 //notice that: both dma and mapping mode need this to trigger data address auto increase(confirmed by congqing and sihui)
	reg_i2c_mode |=  FLD_I2C_SLAVE_EN ;

	if(mode == I2C_SLAVE_MAP){
		reg_i2c_mode |= FLD_I2C_SLAVE_MAPPING ;
		reg_i2c_slave_map_addr = (unsigned int) pMapBuf & 0xffff;
	}
	else{
		reg_i2c_mode &= ~FLD_I2C_SLAVE_MAPPING;
	}
}

/**
 * @brief      This function serves to write one byte to the slave device at the specified address
 * @param[in]  Addr - i2c slave address where the one byte data will be written
 * @param[in]  AddrLen - length in byte of the address, which makes this function is
 *             compatible for slave device with both one-byte address and two-byte address
 * @param[in]  Data - the one byte data will be written via I2C interface
 * @return     none
 */
void i2c_dma_write_byte(unsigned int Addr, unsigned int AddrLen, unsigned char Data)
{

	reg_i2c_id	 &= (~FLD_I2C_WRITE_READ_BIT); //SlaveID & 0xfe,.i.e write data. R:High  W:Low

		//start + id(Write) + address

		if (AddrLen == 1) {
			reg_i2c_adr = (unsigned char)Addr;; //address
			//lanuch start /id/04    start
			reg_i2c_ctrl = (FLD_I2C_CMD_ID | FLD_I2C_CMD_ADDR | FLD_I2C_CMD_START);
		}
		else if (AddrLen == 2) {
			reg_i2c_adr = (unsigned char)(Addr>>8); //address high
			reg_i2c_do = (unsigned char)Addr; //address low
			//lanuch start /id/04/05    start
			reg_i2c_ctrl = (FLD_I2C_CMD_ID | FLD_I2C_CMD_ADDR | FLD_I2C_CMD_DO | FLD_I2C_CMD_START);
		}

		while(reg_i2c_status & FLD_I2C_CMD_BUSY	);

		/* Write data to slave //Data buffer for Read or Write in master mode*/
		reg_i2c_di = Data;
		reg_i2c_ctrl = FLD_I2C_CMD_DI;
		while(reg_i2c_status & FLD_I2C_CMD_BUSY	);
		/* Send Stop command */
		reg_i2c_ctrl = FLD_I2C_CMD_STOP;
		while(reg_i2c_status & FLD_I2C_CMD_BUSY	);

}

/**
 * @brief      This function serves to read one byte from the slave device at the specified address
 * @param[in]  Addr - i2c slave address where the one byte data will be read
 * @param[in]  AddrLen - length in byte of the address, which makes this function is
 *             compatible for slave device with both one-byte address and two-byte address
 * @return     the one byte data read from the slave device via I2C interface
 */
unsigned char i2c_dma_read_byte(unsigned int Addr, unsigned int AddrLen)
{
	 unsigned char ret = 0;
	 reg_i2c_id	 &= (~FLD_I2C_WRITE_READ_BIT); //SlaveID & 0xfe,.i.e write data. R:High  W:Low


	 	//start + id(Write) + address

	 	if (AddrLen == 1) {
	 		reg_i2c_adr = (unsigned char)Addr;; //address
	 		//lanuch start /id/04    start
	 		reg_i2c_ctrl = (FLD_I2C_CMD_ID | FLD_I2C_CMD_ADDR | FLD_I2C_CMD_START);
	 	}
	 	else if (AddrLen == 2) {
	 		reg_i2c_adr = (unsigned char)(Addr>>8); //address high
	 		reg_i2c_do = (unsigned char)Addr; //address low
	 		//lanuch start /id/04/05    start
	 		reg_i2c_ctrl = (FLD_I2C_CMD_ID | FLD_I2C_CMD_ADDR | FLD_I2C_CMD_DO | FLD_I2C_CMD_START);
	 	}

	 	while(reg_i2c_status & FLD_I2C_CMD_BUSY	);

	 	//start + id(Read)
	 	reg_i2c_id	 |= FLD_I2C_WRITE_READ_BIT;  //SlaveID & 0xfe,.i.e write data. Read:High  Write:Low
	 	reg_i2c_ctrl = (FLD_I2C_CMD_ID | FLD_I2C_CMD_START);
	 	while(reg_i2c_status & FLD_I2C_CMD_BUSY	);

	 	//read data
	 	 reg_i2c_ctrl = (FLD_I2C_CMD_DI | FLD_I2C_CMD_READ_ID | FLD_I2C_CMD_ACK);
	 	 while(reg_i2c_status & FLD_I2C_CMD_BUSY	);
	 	 ret = reg_i2c_di;

	    //stop
	 	 reg_i2c_ctrl = FLD_I2C_CMD_STOP; //launch stop cycle
	 	 while(reg_i2c_status & FLD_I2C_CMD_BUSY	);

	 	 return ret;
}

/**
 *  @brief      This function serves to write a packet of data to the specified address of slave device
 *  @param[in]  Addr - the register that master write data to slave in. support one byte and two bytes. i.e param2 AddrLen may be 1 or 2.
 *  @param[in]  AddrLen - the length of register. enum 0 or 1 or 2 or 3. based on the spec of i2c slave.
 *  @param[in]  dataBuf - the first SRAM buffer address to write data to slave in.
 *  @param[in]  dataLen - the length of data master write to slave.
 *  @return     none
 */
void i2c_dma_write_buff (unsigned int Addr, unsigned int AddrLen, unsigned char * dataBuf, int dataLen)
{
	reg_i2c_id	 &= (~FLD_I2C_WRITE_READ_BIT); //SlaveID & 0xfe,.i.e write data. R:High  W:Low

	//start + id(Write) + address

	if (AddrLen == 1) {
		reg_i2c_adr = (unsigned char)Addr;//address
		//lanuch start /id/04    start
		reg_i2c_ctrl = (FLD_I2C_CMD_ID | FLD_I2C_CMD_ADDR | FLD_I2C_CMD_START);
	}
	else if (AddrLen == 2) {
		reg_i2c_adr = (unsigned char)(Addr>>8); //address high
		reg_i2c_do = (unsigned char)Addr; //address low
		//lanuch start /id/04/05    start
		reg_i2c_ctrl = (FLD_I2C_CMD_ID | FLD_I2C_CMD_ADDR | FLD_I2C_CMD_DO | FLD_I2C_CMD_START);
	}

	while(reg_i2c_status & FLD_I2C_CMD_BUSY	);

	/* Write data to slave */
	unsigned int buff_index = 0;
	for(buff_index=0;buff_index<dataLen;buff_index++){
		reg_i2c_di = dataBuf[buff_index];//reg_i2c_di,Data buffer for Read or Write in master mode
		reg_i2c_ctrl = FLD_I2C_CMD_DI; //launch data read cycle
		while(reg_i2c_status & FLD_I2C_CMD_BUSY	);
	}


	/* Send Stop command */
	reg_i2c_ctrl = FLD_I2C_CMD_STOP; //launch stop cycle
	while(reg_i2c_status & FLD_I2C_CMD_BUSY	);

}
/**
 * @brief      This function serves to read a packet of data from the specified address of slave device
 * @param[in]  Addr - the register master read data from slave in. support one byte and two bytes.
 * @param[in]  AddrLen - the length of register. enum 0 or 1 or 2 or 3 based on the spec of i2c slave.
 * @param[in]  dataBuf - the first address of SRAM buffer master store data in.
 * @param[in]  dataLen - the length of data master read from slave.
 * @return     none.
 */
void i2c_dma_read_buff(unsigned int Addr, unsigned int AddrLen, unsigned char * dataBuf, int dataLen)
{
	reg_i2c_id	 &= (~FLD_I2C_WRITE_READ_BIT); //SlaveID & 0xfe,.i.e write data. R:High  W:Low


	//start + id(Write) + address

	if (AddrLen == 1) {
		reg_i2c_adr = (unsigned char)Addr;; //address
		//lanuch start /id/04    start
		reg_i2c_ctrl = (FLD_I2C_CMD_ID | FLD_I2C_CMD_ADDR | FLD_I2C_CMD_START);
	}
	else if (AddrLen == 2) {
		reg_i2c_adr = (unsigned char)(Addr>>8); //address high
		reg_i2c_do = (unsigned char)Addr; //address low
		//lanuch start /id/04/05    start
		reg_i2c_ctrl = (FLD_I2C_CMD_ID | FLD_I2C_CMD_ADDR | FLD_I2C_CMD_DO | FLD_I2C_CMD_START);
	}
	while(reg_i2c_status & FLD_I2C_CMD_BUSY	);

	//start + id(Read)
	reg_i2c_id	 |= FLD_I2C_WRITE_READ_BIT;  //SlaveID & 0xfe,.i.e write data. Read:High  Write:Low
	reg_i2c_ctrl = (FLD_I2C_CMD_ID | FLD_I2C_CMD_START);
	while(reg_i2c_status & FLD_I2C_CMD_BUSY	);


	//read data
	unsigned int bufIndex = 0;

	dataLen--;    //the length of reading data must larger than 0
	//if not the last byte master read slave, master wACK to slave
	while(dataLen){  //
		reg_i2c_ctrl = (FLD_I2C_CMD_DI | FLD_I2C_CMD_READ_ID);
		while(reg_i2c_status & FLD_I2C_CMD_BUSY	);
		dataBuf[bufIndex] = reg_i2c_di;
		bufIndex++;
		dataLen--;
	}
	//when the last byte, master will ACK to slave
	reg_i2c_ctrl = (FLD_I2C_CMD_DI | FLD_I2C_CMD_READ_ID |FLD_I2C_CMD_ACK);
	while(reg_i2c_status & FLD_I2C_CMD_BUSY	);
	dataBuf[bufIndex] = reg_i2c_di;

	//termiante
	reg_i2c_ctrl = FLD_I2C_CMD_STOP; //launch stop cycle
	while(reg_i2c_status & FLD_I2C_CMD_BUSY	);

}

/**
 *  @brief      This function serves to write a packet of data to slave device working in mapping mode
 *  @param[in]  dataBuf - the first SRAM buffer address to write data to slave in.
 *  @param[in]  dataLen - the length of data master write to slave.
 *  @return     none
 */
void i2c_map_write_buff(unsigned char * dataBuf, int dataLen)
{
	reg_i2c_id	 &= (~FLD_I2C_WRITE_READ_BIT);

	 /*lanuch start /id  */
	reg_i2c_ctrl = (FLD_I2C_CMD_ID | FLD_I2C_CMD_START );
	while(reg_i2c_status & FLD_I2C_CMD_BUSY	);

	/* Write data to slave */
	unsigned int buff_index = 0;
	for(buff_index=0;buff_index<dataLen;buff_index++){
		reg_i2c_di = dataBuf[buff_index];
		reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_DI, 1);
		while(reg_i2c_status & FLD_I2C_CMD_BUSY	);
	}

	/* Send Stop command */
	    reg_i2c_ctrl = FLD_I2C_CMD_STOP;//launch stop cycle
	    while(reg_i2c_status & FLD_I2C_CMD_BUSY	);

}

/**
 * @brief      This function serves to read a packet of data from slave device working in mapping mode
 * @param[in]  dataBuf - the first address of SRAM buffer master store data in.
 * @param[in]  dataLen - the length of data master read from slave.
 * @return     none.
 */
void i2c_map_read_buff(unsigned char * dataBuf, int dataLen)
{
	reg_i2c_id	 &= (~FLD_I2C_WRITE_READ_BIT); //SlaveID & 0xfe,.i.e write data. R:High  W:Low
	 //lanuch start /id    start
	 reg_i2c_ctrl = (FLD_I2C_CMD_ID | FLD_I2C_CMD_START );
	 while(reg_i2c_status & FLD_I2C_CMD_BUSY	);
	 //start + id(Read)
	 	reg_i2c_id	 |= FLD_I2C_WRITE_READ_BIT;  //SlaveID & 0xfe,.i.e write data. Read:High  Write:Low
	 	reg_i2c_ctrl = (FLD_I2C_CMD_ID | FLD_I2C_CMD_START);
	 	while(reg_i2c_status & FLD_I2C_CMD_BUSY	);


	 	//read data
	 	unsigned int bufIndex = 0;

	 	dataLen--;    //the length of reading data must larger than 0
	 	//if not the last byte master read slave, master wACK to slave
	 	while(dataLen){  //
	 		reg_i2c_ctrl = (FLD_I2C_CMD_DI | FLD_I2C_CMD_READ_ID);
	 		while(reg_i2c_status & FLD_I2C_CMD_BUSY	);
	 		dataBuf[bufIndex] = reg_i2c_di;
	 		bufIndex++;
	 		dataLen--;
	 	}
	 	//when the last byte, master will ACK to slave
	 	reg_i2c_ctrl = (FLD_I2C_CMD_DI | FLD_I2C_CMD_READ_ID |FLD_I2C_CMD_ACK);
	 	while(reg_i2c_status & FLD_I2C_CMD_BUSY	);
	 	dataBuf[bufIndex] = reg_i2c_di;

	 	/* Send stop command */
	 	reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_STOP, 1);
	 	//reg_i2c_ctrl = FLD_I2C_CMD_STOP; //launch stop cycle
	 	while(reg_i2c_status & FLD_I2C_CMD_BUSY	);
}
