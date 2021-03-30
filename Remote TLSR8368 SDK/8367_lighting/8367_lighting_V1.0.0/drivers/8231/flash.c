/********************************************************************************************************
 * @file     flash.c 
 *
 * @brief    This is the source file for TLSR8231
 *
 * @author	 Telink
 * @date     May 24, 2019
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
 * 			 1.initial release(May. 24 2019)
 *
 * @version  A001
 *
 *******************************************************************************************************/


#include "flash.h"
#include "spi_i.h"
#include "irq.h"
#include "timer.h"
// !!!!  In the case of being used, disable the following can save some RAM
#define FLASH_WRITE_ENABLE		1
#define FLASH_READ_ENABLE		1
#define OTA_ENABLE				0


#if(OTA_ENABLE)
_attribute_ota_code_
#else
_attribute_ram_code_
#endif
static inline int flash_is_busy(){
	return mspi_read() & 0x01;				//  the busy bit, pls check flash spec
}

/**
 * @brief     This function serves to set flash write command.
 * @param[in] cmd - set command.
 * @return    none
 */

#if(OTA_ENABLE)
_attribute_ota_code_
#else
_attribute_ram_code_
#endif
 static void flash_send_cmd(unsigned char cmd){
	mspi_high();
	delay_us(1);
	mspi_low();
	mspi_write(cmd);
	mspi_wait();
}

/**
 * @brief     This function serves to send flash address.
 * @param[in] addr - the flash address.
 * @return    none
 */

#if(OTA_ENABLE)
_attribute_ota_code_
#else
_attribute_ram_code_
#endif
static void flash_send_addr(unsigned int addr){
	mspi_write((unsigned char)(addr>>16));
	mspi_wait();
	mspi_write((unsigned char)(addr>>8));
	mspi_wait();
	mspi_write((unsigned char)(addr));
	mspi_wait();
}

/**
 * @brief     This function serves to wait flash done.
 *            (make this a asynchorous version).
 * @param[in] none.
 * @return    none.
 */

#if(OTA_ENABLE)
_attribute_ota_code_
#else
_attribute_ram_code_
#endif
static void flash_wait_done(void)
{
	delay_us(100);
	flash_send_cmd(FLASH_READ_STATUS_CMD);

	int i;
	for(i = 0; i < 10000000; ++i){
		if(!flash_is_busy()){
			break;
		}
	}
	mspi_high();
}

#if(OTA_ENABLE)
_attribute_ota_code_
#else
_attribute_ram_code_
#endif
static void flash_erase_read_write ( unsigned int addr,  unsigned int len, unsigned char *buf, unsigned char cmd){
	if(FLASH_READ_CMD != cmd){
		flash_send_cmd(FLASH_WRITE_ENABLE_CMD);
	}
	    flash_send_cmd(cmd);

	//--   send address ---
     flash_send_addr(addr);
	if(FLASH_WRITE_ENABLE && (FLASH_WRITE_CMD == cmd)){
		for(int i = 0; i < len; ++i){
			mspi_write(buf[i]);		/* write data */
			mspi_wait();
		}
	}else if(FLASH_READ_ENABLE && (FLASH_READ_CMD == cmd)){
		mspi_write(0x00);		/* dummy,  to issue clock */
		mspi_wait();
		mspi_ctrl_write(0x0a);	/* auto mode */
		mspi_wait();
		/* get data */
		for(int i = 0; i < len; ++i){
			*buf++ = mspi_get();
			mspi_wait();
		}
	}
	mspi_high();
	flash_wait_done();

}
/**
 * @brief This function serves to erase a page(256 bytes).
 * @param[in]   addr the start address of the page needs to erase.
 * @return none
 */

#if(OTA_ENABLE)
_attribute_ota_code_
#else
_attribute_ram_code_
#endif
void flash_erase_page(unsigned int addr)
{
	unsigned char r = irq_disable();
	flash_erase_read_write(addr, 0, 0, FLASH_PAGE_ERASE_CMD);
    irq_restore(r);
}

/**
 * @brief This function serves to erase a sector.
 * @param[in]   addr the start address of the sector needs to erase.
 * @return none
 */

#if(OTA_ENABLE)
_attribute_ota_code_
#else
_attribute_ram_code_
#endif
void flash_erase_sector(unsigned long addr){
	unsigned char r = irq_disable();//same
	flash_erase_read_write(addr, 0, 0, FLASH_SECT_ERASE_CMD);
	irq_restore(r);//same
}

/**
 * @brief This function serves to erase a block(32k).
 * @param[in]   addr the start address of the block needs to erase.
 * @return none
 */

#if(OTA_ENABLE)
_attribute_ota_code_
#else
_attribute_ram_code_
#endif
void flash_erase_32kblock(unsigned int addr)
{
	unsigned char r = irq_disable();
	flash_erase_read_write(addr, 0, 0, FLASH_32KBLK_ERASE_CMD);
    irq_restore(r);
}

/**
 * @brief This function serves to erase a block(64k).
 * @param[in]   addr the start address of the block needs to erase.
 * @return none
 */
#if(OTA_ENABLE)
_attribute_ota_code_
#else
_attribute_ram_code_
#endif
void flash_erase_64kblock(unsigned int addr)
{   unsigned char r  = irq_disable();
    flash_erase_read_write(addr, 0, 0, FLASH_64KBLK_ERASE_CMD);
	    irq_restore(r);
}

/**
 * @brief This function serves to erase a page(256 bytes).
 * @param[in]   addr the start address of the page needs to erase.
 * @return none
 */

#if(OTA_ENABLE)
_attribute_ota_code_
#else
_attribute_ram_code_
#endif
void flash_erase_chip(void)
{
	unsigned char r = irq_disable();

	flash_send_cmd(FLASH_WRITE_ENABLE_CMD);
	flash_send_cmd(FLASH_CHIP_ERASE_CMD);
	mspi_high();
	flash_wait_done();
    irq_restore(r);
}

/**
 * @brief This function writes the buffer's content to a page.
 * @param[in]   addr the start address of the page
 * @param[in]   len the length(in byte) of content needs to write into the page
 * @param[in]   buf the start address of the content needs to write into
 * @return none
 */
#if(FLASH_WRITE_ENABLE)
#if(OTA_ENABLE)
_attribute_ota_code_
#else
_attribute_ram_code_
#endif
void flash_write_page(unsigned long addr, unsigned long len, unsigned char *buf){
	unsigned char r = irq_disable();
	flash_erase_read_write(addr, len, buf, FLASH_WRITE_CMD);
	irq_restore(r);
}
#endif


/**
 * @brief This function reads the content from a page to the buf.
 * @param[in]   addr the start address of the page
 * @param[in]   len the length(in byte) of content needs to read out from the page
 * @param[out]  buf the start address of the buffer
 * @return none
 */
#if(FLASH_READ_ENABLE)
#if(OTA_ENABLE)
_attribute_ota_code_
#else
_attribute_ram_code_
#endif
void flash_read_page(unsigned long addr, unsigned long len, unsigned char *buf){
	unsigned char r = irq_disable();
	flash_erase_read_write(addr, len, buf, FLASH_READ_CMD);
	irq_restore(r);
}
#endif

/**
 * @brief This function write the status of flash.
 * @param[in]  the value of status
 * @return status
 */
_attribute_ram_code_ unsigned char flash_write_status(unsigned char data)
{
	unsigned char r = irq_disable();
	unsigned char result;
	//int i;
	flash_send_cmd(FLASH_WRITE_ENABLE_CMD);
	flash_send_cmd(FLASH_WRITE_STATUS_CMD);
	mspi_write(data);
	mspi_wait();
	mspi_high();
	flash_wait_done();
	delay_us(100);
	flash_send_cmd(FLASH_READ_STATUS_CMD);
	result = mspi_read();
	mspi_high();
	irq_restore(r);
	return  result;
}

/**
 * @brief This function reads the status of flash.
 * @param[in]  none
 * @return none
 */
_attribute_ram_code_ unsigned char flash_read_status(void){
	unsigned char r = irq_disable();
	unsigned char status =0;
	flash_send_cmd(FLASH_READ_STATUS_CMD);
	/* get low 8 bit status */
	status = mspi_read();
	mspi_high();
	irq_restore(r);
	return status;
}

/***********************************
 * @brief  	Deep Power Down mode to put the device in the lowest consumption mode
 * 			it can be used as an extra software protection mechanism,while the device
 * 			is not in active use,since in the mode,  all write,Program and Erase commands
 * 			are ignored,except the Release from Deep Power-Down and Read Device ID(RDI)
 * 			command.This release the device from this mode
 * @param[in] none
 * @return none.
 */
_attribute_ram_code_ void flash_deep_powerdown(void)
{
	unsigned char r = irq_disable();

	flash_send_cmd(FLASH_POWER_DOWN);
	mspi_high();
	delay_us(1);
    irq_restore(r);
}

/***********************************
 * @brief		The Release from Power-Down or High Performance Mode/Device ID command is a
 * 				Multi-purpose command.it can be used to release the device from the power-Down
 * 				State or High Performance Mode or obtain the devices electronic identification
 * 				(ID)number.Release from Power-Down will take the time duration of tRES1 before
 * 				the device will resume normal operation and other command are accepted.The CS#
 * 				pin must remain high during the tRES1(8us) time duration.
 * @param[in] none
 * @return none.
 */
_attribute_ram_code_ void flash_release_deep_powerdown(void)
{
	unsigned char r = irq_disable();

	flash_send_cmd(FLASH_POWER_DOWN_RELEASE);
	mspi_high();
	flash_wait_done();
	mspi_high();
    irq_restore(r);
}
/***********************************
 * @brief	  MAC id. Before reading UID of flash, you must read MID of flash. and then you can
 *            look up the related table to select the idcmd and read UID of flash
 * @param[in] buf - store MID of flash
 * @return    none.
 */
_attribute_ram_code_ void flash_read_mid(unsigned char *buf){
	unsigned char j = 0;
	unsigned char r = irq_disable();
	flash_send_cmd(FLASH_GET_JEDEC_ID);
	mspi_write(0x00);		/* dummy,  to issue clock */
	mspi_wait();
	mspi_ctrl_write(0x0a);	/* auto mode */
	mspi_wait();
	for(j = 0; j < 3; ++j){
		*buf++ = mspi_get();
		 mspi_wait();
	}
	mspi_high();
	irq_restore(r);

}
/***********************************
 * @brief	  UID. Before reading UID of flash, you must read MID of flash. and then you can
 *            look up the related table to select the idcmd and read UID of flash
 * @param[in] idcmd - get this value to look up the table based on MID of flash
 * @param[in] buf   - store UID of flash
 * @return    none.
 */
_attribute_ram_code_ void flash_read_uid(unsigned char idcmd,unsigned char *buf)
{
	unsigned char j = 0;
	unsigned char r = irq_disable();
	flash_send_cmd(idcmd);
	if(idcmd==0x4b)				//< GD/puya
	{
		flash_send_addr(0x00);
		mspi_write(0x00);		/* dummy,  to issue clock */
		mspi_wait();
	}
	else if (idcmd==0x5a)		//< XTX
	{
		flash_send_addr(0x80);
		mspi_write(0x00);		/* dummy,  to issue clock */
		mspi_wait();

	}
	mspi_write(0x00);			/* dummy,  to issue clock */
	mspi_wait();
	mspi_ctrl_write(0x0a);		/* auto mode */
	mspi_wait();

	for(j = 0; j < 16; ++j){
		*buf++ = mspi_get();
		mspi_wait();
	}
	mspi_high();
	irq_restore(r);
}





