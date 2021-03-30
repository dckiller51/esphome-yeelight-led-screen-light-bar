/********************************************************************************************************
 * @file     rf_drv.h
 *
 * @brief    This is the head file for TLSR8231
 *
 * @author	 Telink
 * @date    May 24, 2019
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
#ifndef _RF_DRV_H
#define _RF_DRV_H

#include "bsp.h"
#include "register.h"
#define RF_CHN_AUTO_CAP 	0xff00
#define RF_CHN_TABLE 		0x8000
#define RF_SET_TX_MANAUL	0x4000

#define FRE_OFFSET   	    0
#define FRE_STEP 	        5
#define MAX_RF_CHANNEL      16

#define RF_CHANNEL_MAX		16
#define RF_CHANNEL_MASK		(RF_CHANNEL_MAX - 1)

extern unsigned char rfhw_tx_power;
extern unsigned char cap_tp[RF_CHANNEL_MAX];
extern const unsigned char rf_chn[RF_CHANNEL_MAX];

enum{
	RF_TX_MODE_NORMAL = 0,
	RF_TX_MODE_CARRIER,
	RF_TX_MODE_CONTINUE,

	RF_POWER_LEVEL_MAX = 0,
	RF_POWER_LEVEL_M2 = 1,
	RF_POWER_LEVEL_M3 = 2,
	RF_POWER_LEVEL_MIN = 100,
};

typedef enum {
    RF_MODE_TX = 0,
    RF_MODE_RX = 1,
    RF_MODE_AUTO=2
} RF_StatusTypeDef;

typedef enum{
	RF_MODE_BLE_1M_NO_PN   		    = 0,
	RF_MODE_BLE_1M					= 1,
	RF_MODE_BLE_2M					= 2,
    RF_MODE_BLE_2M_NO_PN			= 3,
    RF_MODE_BLE_250K				= 4,
    RF_MODE_BLE_500K				= 5,
    RF_MODE_PRI_1M     				= 6,
    RF_MODE_PRI_2M					= 7,
    RF_MODE_PRI_250K				= 8,
    RF_MODE_PRI_500K				= 9,

}RF_ModeTypeDef;

typedef enum {
	RF_POWER_7dBm		= 0,
	RF_POWER_6dBm		= 1,
	RF_POWER_3dBm		= 2,
	RF_POWER_2dBm		= 3,
	RF_POWER_0dBm		= 4,
	RF_POWER_m1dBm		= 5,
	RF_POWER_m2dBm		= 6,
	RF_POWER_m10dBm		= 7,
	RF_POWER_m12dBm		= 8,
	RF_POWER_m14dBm		= 9,
	RF_POWER_m15dBm		= 10,
	RF_POWER_m17dBm		= 11,
}RF_TxPowerTypeDef;


#define FR_TX_PA_MAX_POWER	0x40
#define FR_TX_PA_MIN_POWER	0x41

#define	SET_RF_TX_DMA_ADR(a)			WRITE_REG16 (0x80050c, a)

/**
 *  @brief  Used to verify that the length of the packet received in the following RF modes is correct.
 *			ble_1M_mode/ble_2M_mode/ble_250K_mode/ble_500K_mode/ble_1M_NO_PN_mode/ble_2M_NO_PN_mode
 */
#define	RF_BLE_PACKET_LENGTH_OK(p)		(p[0] == (p[13])+17)
/**
 *  @brief  Used to verify that the length of the packet received in the following RF modes is correct.
 *  		pri_1M_mode/pri_2M_mode/pri_500K_mode/pri_250K_mode
 */
#define	RF_PRI_PACKET_LENGTH_OK(p)		(p[0] == (p[12]&0x3f)+15)
/**
 *  @brief  Used to verify that the CRC of the packet received in the following RF mode is correct.
 *  		pri_1M_mode/pri_500K_mode/ble_1M_mode/ble_500K_mode
 */
#define	RF_1M_500K_PACKET_CRC_OK(p)		((p[p[0]+3] & 0x51) == 0x10)
/**
 *  @brief  Used to verify that the CRC of the packet received in the following RF mode is correct.
 *  		pri_2M_mode/ble_2M_mode
 */
#define	RF_2M_PACKET_CRC_OK(p)		((p[p[0]+3] & 0x51) == 0x00)
/**
 *  @brief  Used to verify that the CRC of the packet received in the following RF mode is correct.
 *  		pri_250K_mode/ble_250K_mode
 */
#define	RF_250K_PACKET_CRC_OK(p)		((p[p[0]+3] & 0x51) == 0x50)

/**
*	@brief     This function serves to initiate information of RF
*	@param[in] rf_mode  -  mode of RF
*	@return	   none.
*/
void rf_mode_init (void);
/**
*	@brief     This function serves to  set ble_1M  mode of RF
*	@param[in] rf_mode  -  mode of RF
*	@return	   none.
*/
void rf_set_ble_1M_mode(void);
/**
*	@brief     This function serves to set ble_2M  mode of RF
*	@param[in] rf_mode  -  mode of RF
*	@return	   none.
*/
void rf_set_ble_2M_mode(void);
/**
*	@brief     This function serves to set ble_250K  mode of RF
*	@param[in] rf_mode  -  mode of RF
*	@return	   none.
*/
void rf_set_ble_250K_mode(void);
/**
*	@brief     This function serves to set ble_500K  mode of RF
*	@param[in] rf_mode  -  mode of RF
*	@return	   none.
*/
void rf_set_ble_500K_mode(void);
/**
*	@brief     This function serves to set ble_1M_NO_PN  mode of RF
*	@param[in] rf_mode  -  mode of RF
*	@return	   none.
*/
void rf_set_ble_1M_NO_PN_mode(void);

/**
*	@brief     This function serves to set ble_2M_NO_PN  mode of RF
*	@param[in] rf_mode  -  mode of RF
*	@return	   none.
*/
void rf_set_ble_2M_NO_PN_mode(void);
/**
*	@brief     This function serves to set pri_1M  mode of RF
*	@param[in] rf_mode  -  mode of RF
*	@return	   none.
*/
void rf_set_pri_1M_mode(void);
/**
*	@brief     This function serves to set pri_2M  mode of RF
*	@param[in] rf_mode  -  mode of RF
*	@return	   none.
*/
void rf_set_pri_2M_mode(void);
/**
*	@brief     This function serves to set pri_250K  mode of RF
*	@param[in] rf_mode  -  mode of RF
*	@return	   none.
*/
void rf_set_pri_250K_mode(void);
/**
*	@brief     This function serves to set pri_500K  mode of RF
*	@param[in] rf_mode  -  mode of RF
*	@return	   none.
*/
void rf_set_pri_500K_mode(void);

/**
 * @brief   This function serves to set RF channel.
 * @param   RF_PowerTypeDef - the RF power types.
 * @return  none.
 */

void rf_set_channel (signed char chn, unsigned short set);


/**
 * @brief   This function serves to set RF power level index.
 * @param   RF_PowerTypeDef - the RF power types.
 * @return  none.
 */

void rf_set_power_level_index (RF_TxPowerTypeDef level);
/**
*	@brief	  	This function is to set rx buffer
*
*	@param[out]	RF_RxAddr  	Pointer for Rx buffer in RAM(Generally it¡¯s starting
*							address of an array.Should be 4-byte aligned)
*	@param[in]	size   		Rx buffer size (It¡¯s an integral multiple of 16)
*	@param[in]	PingpongEn 	Enable/Disable Ping-Pong buffer 1£ºEnable 0£ºDisable
*							Note:
*							When ¡°PingpongEn¡± is set as 0, received RF data will
*							be stored in RAM pointed by ¡° RF_RxAddr¡±.
*							When ¡°PingpongEn¡± is set as 1, received RF data will
*							be stored in buffer0 and buffer1 successively.
*							The RAM size reserved for received RF data should be
*							double of ¡°Size¡±.
*
*	@return	 	none
*/
void  rf_set_rx_buff(unsigned char *  RF_RxAddr, int size, unsigned char  PingpongEn);
/**
 * @brief   This function serves to set RF channel.
 * @param   RF_PowerTypeDef - the RF power types.
 * @return  none.
 */

void rf_set_ble_channel (signed char chn);
/**
 * @brief   This function serves to set gain in auto mode.
 * @param   chn  - set tp gain for the specified channel
 * @return  none.
 */

void rf_set_tp_gain (char chn);

/**
*	@brief	 This function is to update TP(two point),this value will affect
*			 RF performance
*	@param[in]	tp0  	    Tp value for lower frequency
*							If you set a value outside the range, you will be set to fail.
*	@param[in]	tp1  	    Tp value for higher frequency
*							If you set a value outside the range, you will be set to fail.
*	@return	 	0 :set success;
*              -1 :set failed
*/
void rf_set_tp(unsigned char tp0, unsigned char tp1);

/**
 * @brief   This function serves to set gain in manual mode.
 * @param   none.
 * @return  none.
 */
void rf_set_manual_max_gain (void);
/**
 * @brief   This function serves to set gain in auto mode.
 * @param   none.
 * @return  none.
 */
void rf_set_agc (void);
/**
*	@brief	  	This function serves to start Tx of ble_mode.
*	@param[in]	addr   Tx packet address in RAM. Should be 4-byte aligned.
*	@param[in]	tick  Tick value of system timer. It determines when to
*						  	  start ble mode and send packet.
*	@return	 	none
*/
void rf_start_brx(void* addr, unsigned int tick);

/**
 * @brief   This function serves to set RF Tx mode.
 * @param   none.
 * @return  none.
 */
static inline void rf_set_rx_on (void)
{
    WRITE_REG8 (0x800428, 0x80 | BIT(0));	// rx disable
    WRITE_REG8 (0x800f02, 0x45 | BIT(5));		// RX enable
}

//manual tx mode
/**
 * @brief   This function serves to set tx  on .
 * @param   none.
 * @return  none.
 */
static inline void rf_set_tx_on (void)
{
	WRITE_REG8  (0x800f02, 0x45 | BIT(4));	// TX enable
}

//maunal mode off
/**
 * @brief   This function serves to set tx_rx  off.
 * @param   none.
 * @return  none.
 */
static inline void rf_set_tx_rx_off(void)
{
	WRITE_REG8 (0x800f16, 0x29);
	WRITE_REG8 (0x800428, 0x80);	// rx disable
	WRITE_REG8 (0x800f02, 0x45);	// reset tx/rx state machine
	WRITE_REG8 (0xf00, 0x80);
}

//auto mode off
/**
 * @brief   This function serves to set auto mode tx and rx  off .
 * @param   none.
 * @return  none.
 */
static inline void rf_set_tx_rx_off_auto(void)
{
	WRITE_REG8 (0xf00, 0x80);
}

/**
*	@brief	  	This function serves to start Tx of ble_mode.
*	@param[in]	addr   Tx packet address in RAM. Should be 4-byte aligned.
*	@param[in]	tick  Tick value of system timer. It determines when to
*						  	  start ble mode and send packet.
*	@return	 	none
*/
void rf_start_btx (void* addr, unsigned int tick);
/**
*	@brief	  	This function serves to start Tx.
*	@param[in]	addr   Tx packet address in RAM. Should be 4-byte aligned.
*	@param[in]	tick  Tick value of system timer.
*	@return	 	none
*/
void rf_start_stx  (void* addr, unsigned int tick);
/**
*	@brief	  	This function serves to start Rx.
*	@param[in]	tick  Tick value of system timer.
*	@return	 	none
*/
void rf_start_srx  (unsigned int tick);
/**
*	@brief	  	This function serves to start stx2rx mode of auto_mode.
*				In this mode, a packet is sent first,RF module waits for 10us,
*				stays in Rx status until data is received or timeout expires,
*				then exits this mode.Timeout duration is set by the parameter
*				¡°timeout_us¡±.The address to store received data is set by the
*				function ¡°RF_RxBufferSet¡±.
*
*	@param[in]	addr  Tx packet address in RAM. Should be 4-byte aligned.
*	@param[in]	tick   	Tick value of system timer. It determines when
*								to start StxToRx mode and send packet.
*	@param[in]	timeout_us  Unit is us. It indicates timeout duration in
*							 	Rx status.Max value: 0xfff (4095)
*
*	@return	 	none
*/
void rf_start_stx2rx  (void* addr, unsigned int tick, unsigned int timeout_us);
/**
*	@brief	  	This function serves to start srx2tx mode of auto_mode.
*				In this mode,RF module stays in Rx status until a packet is
*				received or it fails to receive packetwhen timeout expires.
*				If a packet is received within the timeout duration, RF module
*				will wait for 10us,send a packet, and then exit this mode.
*				If it fails to receive packet when timeout expires, RF module
*				will directly exit this mode.Timeout duration is set by the
*				parameter "timeout_us".	The address to store received data is set
*				by the function ¡°RF_RxBufferSet¡±.
*
*	@param[in]	addr 	Tx packet address in RAM. Should be 4-byte aligned.
*	@param[in]	tick   Tick value of system timer. It determines when to
*								start SrxToTx mode.
*	@param[in]	timeout_us  Unit is us. It indicates timeout duration in Rx status.
*								Max value: 0xffffff (16777215)
*
*	@return	 	none
*/
void rf_start_srx2tx  (void* addr, unsigned int tick, unsigned int timeout_us);

/**
 * @brief     This function performs to enable RF Tx.
 * @param[in] none.
 * @return    none.
 */
static inline void rf_ble_tx_on ()
{
	WRITE_REG8  (0x800f02, 0x45 | BIT(4));	// TX enable
	WRITE_REG32 (0x800f04, 0x38);
}
/**
 * @brief     This function performs to done RF Tx.
 * @param[in] none.
 * @return    none.
 */
static inline void rf_ble_tx_done ()
{
	WRITE_REG8  (0x800f02, 0x45);	// TX enable
	WRITE_REG32 (0x800f04, 0x50);
}


/**
*	@brief     This function serves to reset RF BaseBand
*	@param[in] none.
*	@return	   none.
*/
static inline void reset_baseband(void)
{
	REG_ADDR8(0x60) = BIT(7);		//reset baseband
	REG_ADDR8(0x60) = 0;			//release reset signal
}
/**
 * @brief   This function serves to reset function for RF.
 * @param   none
 * @return  none
 */
static inline void reset_sn_nesn(void)
{
	REG_ADDR8(0xf01) =  0x01;
}
/**
 * @brief   This function serves to reset the RF sn.
 * @param   none.
 * @return  none.
 */
static inline void rf_reset_sn (void)
{
	WRITE_REG8  (0x800f01, 0x3f);
	WRITE_REG8  (0x800f01, 0x00);
}
/**
* @brief   This function serves to settle adjust for RF Tx.
* @param   txstl_us - adjust TX settle time.
* @return  none
*/
static inline void 	tx_settle_adjust(unsigned short txstl_us)
{
	REG_ADDR16(0xf04) = txstl_us;  //adjust TX settle time
}
/**
*	@brief	  	This function serves to clear the Tx finish flag bit.
*				After all packet data are sent, corresponding Tx finish flag bit
*				will be set as 1.By reading this flag bit, it can check whether
*				packet transmission is finished. After the check, it¡¯s needed to
*				manually clear this flag bit so as to avoid misjudgment.
*   @param      none
*	@return	 	none
*/
static inline void rf_clr_tx_finish(void)
{
    WRITE_REG8(0xf20, READ_REG8(0xf20) | 0x02);
}

/**
*	@brief	  	This function serves to judge RF Tx/Rx state.
*	@param[in]	rf_status - Tx/Rx status.
*	@param[in]	rf_channel - RF channel.
*	@return	 	failed -1,else success.
*/
int rf_set_trx_state(RF_StatusTypeDef rf_status, signed char rf_channel);
/**
*	@brief	  	This function serves to set RF Tx packet.
*	@param[in]	*addr - the address RF to send packet.
*	@return	 	none.
*/
void rf_tx_pkt(unsigned char *rf_txaddr);



#define	rf_get_pipe(p)		p[7]

#define RF_TRX_OFF_MANUAL   0x55        //f02

/**
 * @brief   This function serves to set pipe for RF Tx.
 * @param   p - RF Optional range .
 * @return  none
 */
static inline void rf_ble_set_crc (unsigned char *p)
{
	WRITE_REG32 (0x800424, p[0] | (p[1]<<8) | (p[2]<<16));
}

static inline void rf_ble_set_crc_value (unsigned int crc)
{
	WRITE_REG32 (0x800424, crc);
}
/**
 * @brief   This function serves to set CRC value for RF.
 * @param   crc - CRC value.
 * @return  none.
 */
static inline void rf_set_ble_crc_adv ()
{
	WRITE_REG32 (0x800424, 0x555555);
}
/**
 * @brief   This function serves to set RF access code.
 * @param   p - the address to access.
 * @return  none
 */
static inline void rf_set_ble_access_code (unsigned char *p)
{
	WRITE_REG32 (0x800408, p[3] | (p[2]<<8) | (p[1]<<16) | (p[0]<<24));
}
/**
 * @brief   This function serves to set RF access code value.
 * @param   ac - the address value.
 * @return  none
 */
static inline void rf_set_ble_access_code_value (unsigned int ac)
{
	WRITE_REG32 (0x408, ac);
}
/**
 * @brief   This function serves to set RF access code advantage.
 * @param   none.
 * @return  none.
 */
static inline void rf_set_ble_access_code_adv (void)
{
	WRITE_REG32 (0x800408, 0xd6be898e);
}
/**
 * @brief   This function serves to set RF access command.
 * @param   acc - the command.
 * @return  none.
 */
static inline void rf_set_acc_code (unsigned int acc)
{
	WRITE_REG32 (0x800408, acc);
//	WRITE_REG8 (0x800405, read_reg8(0x405)|0x80);
}
/**
 * @brief   This function serves to set RF access code 6bit to 32bit.
 * @param   code - the access code.
 * @return  the value of the access code.
 */
static inline unsigned int rf_access_code_16to32 (unsigned short code)
{
	unsigned int r = 0;
	for (int i=0; i<16; i++) {
		r = r << 2;
		r |= code & BIT(i) ? 1 : 2;
	}
	return r;
}
/**
 * @brief   This function serves to set RF access code 6bit to 32bit.
 * @param   code - the access code.
 * @return  the value of access code.
 */
static inline unsigned short rf_access_code_32to16 (unsigned int code)
{
	unsigned short r = 0;
	for (int i=0; i<16; i++)
	{
		r = r << 1;

		r |= (code & BIT(i*2)) ? 1 : 0;

	}
	return r;
}
/**
*	@brief		this function is to set shock burst for RF.
*	@param[in]	len - length of shockburst.
*	@return	 	none.
*/
static inline void rf_pri_set_shockburst_len(int len)
{
    WRITE_REG8(0x404, READ_REG8(0x404)|0x03); //select shockburst header mode
    WRITE_REG8(0x406, len);
}

/**
 * @brief   This function serves to set pipe for RF Tx.
 * @param   pipe - RF Optional range .
 * @return  none
 */
static inline void rf_set_tx_pipe (unsigned char pipe)
{
	WRITE_REG8 (0x800f15, 0xf0 | pipe);
}
/**
*	@brief	  	This function serves to determine whether sending a packet of data is finished
*	@param[in]	none.
*	@return	 	Yes: 1, NO: 0.
*/
static inline unsigned char rf_is_tx_finish(void)
{
    return ((READ_REG8(0xf20) & BIT(1))==0x02);
}
/**
*	@brief	  	This function serves to determine whether sending a packet of data is finished
*	@param[in]	none.
*	@return	 	Yes: 1, NO: 0.
*/
static inline unsigned char rf_is_rx_finish(void)
{
    return ((READ_REG8(0xf20) & BIT(0))==0x01);
}

/**
*	@brief	  	This function serves to clear the Tx finish flag bit.
*				After all packet data are sent, corresponding Tx finish flag bit
*				will be set as 1.By reading this flag bit, it can check whether
*				packet transmission is finished. After the check, it¡¯s needed to
*				manually clear this flag bit so as to avoid misjudgment.
*   @param      none
*	@return	 	none
*/
static inline void rf_clr_rx_finish(void)
{
    WRITE_REG8(0xf20, READ_REG8(0xf20) | 0x01);
}

#endif
/** \defgroup GP11 RF Examples
 *
 * 	@{
 */
/*! \page rf Table of Contents
	- [API-RF-CASE1:RF BLE 2M TX](#RF_BLE_2M_TX)
	- [API-RF-CASE2:RF BLE 2M RX](#RF_BLE_2M_RX)
	- [API-RF-CASE3:RF BLE 1M TX](#RF_BLE_1M_TX)
	- [API-RF-CASE4:RF BLE 1M RX](#RF_BLE_1M_RX)
	- [API-RF-CASE5:RF BLE 1M NO PN TX](#RF_BLE_1M_NO_PN_TX)
	- [API-RF-CASE6:RF BLE 1M NO PN RX](#RF_BLE_1M_NO_PN_RX)
	- [API-RF-CASE7:RF ZIGBEE 250K TX](#RF_ZIGBEE_250K_TX)
	- [API-RF-CASE8:RF ZIGBEE 250K RX](#RF_ZIGBEE_250K_RX)
	- [API-RF-CASE9:RF LR S2 500K TX](#RF_LR_S2_500K_TX)
	- [API-RF-CASE10:RF LR S2 500K RX](#RF_LR_S2_500K_RX)
	- [API-RF-CASE11:RF LR S8 125K TX](#RF_LR_S8_125K_TX)
	- [API-RF-CASE12:RF LR S8 125K RX](#RF_LR_S8_125K_RX)
	- [API-RF-CASE13:RF PRIVATE 250K TX](#RF_PRIVATE_250K_TX)
	- [API-RF-CASE14:RF PRIVATE 250K RX](#RF_PRIVATE_250K_RX)
	- [API-RF-CASE15:RF PRIVATE 500K TX](#RF_PRIVATE_500K_TX)
	- [API-RF-CASE16:RF PRIVATE 500K RX](#RF_PRIVATE_500K_RX)
	- [API-RF-CASE17:RF PRIVATE 1M TX](#RF_PRIVATE_1M_TX)
	- [API-RF-CASE18:RF PRIVATE 1M RX](#RF_PRIVATE_1M_RX)
	- [API-RF-CASE19:RF PRIVATE 2M TX](#RF_PRIVATE_2M_TX)
	- [API-RF-CASE20:RF PRIVATE 2M RX](#RF_PRIVATE_2M_RX)
	- [API-RF-CASE21:RF ANT TX](#RF_ANT_TX)
	- [API-RF-CASE22:RF ANT RX](#RF_ANT_RX)
	- [API-RF-CASE23:RF BLE 2M AUTO TX](#RF_BLE_2M_AUTO_TX)
	- [API-RF-CASE24:RF BLE 2M AUTO RX](#RF_BLE_2M_AUTO_RX)
	- [API-RF-CASE25:RF BLE 2M AUTO TX2RX](#RF_BLE_2M_AUTO_TX2RX)
	- [API-RF-CASE26:RF BLE 2M AUTO RX2TX](#RF_BLE_2M_AUTO_RX2TX)

<h1 id=RF_BLE_2M_TX> API-RF-CASE1:RF BLE 2M TX </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_2M) || RF mode initialization: RF_MODE_BLE_2M | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | rf_set_power_level_index() | rf_set_power_level_index (RF_POWER) | set tx power level  | ^ |
| ^ | ^ | rf_set_tx_rx_off() || reset RF Tx/Rx mode  | ^ |
| ^ | ^ | rf_set_channel() | rf_set_channel(RF_FREQ,0) | set channel for RF | ^ |
| ^ | ^ | rf_set_acc_code() | rf_set_acc_code(ACCESS_CODE) | set access code for RF | ^ |
| ^ | ^ | rf_set_tx_on() || start Tx mode | ^ |
| ^ | main_loop() | rf_tx_pkt() | rf_tx_pkt(ble_tx_packet) | send packet in manual mode | ^ |
| ^ | ^ | rf_is_tx_finish() | while(!rf_is_tx_finish()) | wait for tx packet being finished | ^ |
| ^ | ^ | rf_clr_tx_finish() || clear the flag of tx finished | ^ |

\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}

#define RF_FREQ				35
#define RF_POWER			RF_POWER_P10p46dBm
#define ACCESS_CODE			0x29417671

volatile unsigned int tx_cnt=0;
unsigned char  ble_tx_packet[48] __attribute__ ((aligned (4))) = {0x23,0x00,0x00,0x00,0x00,0x21,0x00,0x00,0x00,0x00,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};

~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=RF_BLE_2M_RX> API-RF-CASE2:RF BLE 2M RX </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_2M) || RF mode initialization: RF_MODE_BLE_2M | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | rf_set_power_level_index() | rf_set_power_level_index (RF_POWER) | set tx power level  | ^ |
| ^ | ^ | rf_set_tx_rx_off() || reset RF Tx/Rx mode  | ^ |
| ^ | ^ | rf_set_channel() | rf_set_channel(RF_FREQ,0) | set channel for RF | ^ |
| ^ | ^ | rf_set_acc_code() | rf_set_acc_code(ACCESS_CODE) | set access code for RF | ^ |
| ^ | ^ | rf_set_rx_buff() | rf_set_rx_buff(rx_packet,64, 0) | set buffer for rx packet | ^ |
| ^ | ^ | rf_set_rx_on() || start Rx mode | ^ |
| ^ | main_loop() | if( rf_is_rx_finish() ) | rf_is_rx_finish() | determine whether rx packet is finished | ^ |
| ^ | ^ | > if(RF_BLE_PACKET_CRC_OK(rx_packet)&&RF_BLE_PACKET_LENGTH_OK(rx_packet)) || determine whether rx packet is right | ^ |
| ^ | ^ | >> rx_cnt++ || Perform packet parsing and processing | ^ |
| ^ | ^ | > rf_clr_rx_finish() || clear the flag of rx finished | ^ |
| ^ | ^ | > rf_set_tx_rx_off_auto() || turn off RF auto mode | ^ |

\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}

#define RF_FREQ				35
#define RF_POWER			RF_POWER_P10p46dBm
#define ACCESS_CODE			0x29417671

volatile unsigned int rx_cnt=0;
unsigned char  rx_packet[64]  __attribute__ ((aligned (4)));

~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=RF_BLE_1M_TX> API-RF-CASE3:RF BLE 1M TX </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-RF-CASE1:RF BLE 2M TX](group___g_p11.html#RF_BLE_2M_TX)

<h1 id=RF_BLE_1M_RX> API-RF-CASE4:RF BLE 1M RX </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-RF-CASE2:RF BLE 2M RX](group___g_p11.html#RF_BLE_2M_RX)

<h1 id=RF_BLE_1M_NO_PN_TX> API-RF-CASE5:RF BLE 1M NO PN TX </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-RF-CASE1:RF BLE 2M TX](group___g_p11.html#RF_BLE_2M_TX)

<h1 id=RF_BLE_1M_NO_PN_RX> API-RF-CASE6:RF BLE 1M NO PN RX </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-RF-CASE2:RF BLE 2M RX](group___g_p11.html#RF_BLE_2M_RX)

<h1 id=RF_ZIGBEE_250K_TX> API-RF-CASE7:RF ZIGBEE 250K TX </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_ZIGBEE_250K) || RF mode initialization: RF_MODE_ZIGBEE_250K | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | rf_set_power_level_index() | rf_set_power_level_index (RF_POWER) | set tx power level  | ^ |
| ^ | ^ | rf_set_tx_rx_off() || reset RF Tx/Rx mode  | ^ |
| ^ | ^ | rf_set_channel() | rf_set_channel(RF_FREQ,0) | set channel for RF | ^ |
| ^ | ^ | rf_set_acc_code() | rf_set_acc_code(ACCESS_CODE) | set access code for RF | ^ |
| ^ | ^ | rf_set_tx_on() || start Tx mode | ^ |
| ^ | main_loop() | rf_tx_pkt() | rf_tx_pkt(Zigbee_tx_packet) | send packet in manual mode | ^ |
| ^ | ^ | rf_is_tx_finish() | while(!rf_is_tx_finish()) | wait for tx packet being finished | ^ |
| ^ | ^ | rf_clr_tx_finish() || clear the flag of tx finished | ^ |

\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}

#define RF_FREQ				35
#define RF_POWER			RF_POWER_P10p46dBm
#define ACCESS_CODE			0x29417671

volatile unsigned int tx_cnt=0;
unsigned char  Zigbee_tx_packet[48] __attribute__ ((aligned (4))) = {0x12,0x00,0x00,0x00,0x13,0x00,0x00,0x00,0x00,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};

~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=RF_ZIGBEE_250K_RX> API-RF-CASE8:RF ZIGBEE 250K RX </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_ZIGBEE_250K) || RF mode initialization: RF_MODE_ZIGBEE_250K | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | rf_set_power_level_index() | rf_set_power_level_index (RF_POWER) | set tx power level  | ^ |
| ^ | ^ | rf_set_tx_rx_off() || reset RF Tx/Rx mode  | ^ |
| ^ | ^ | rf_set_channel() | rf_set_channel(RF_FREQ,0) | set channel for RF | ^ |
| ^ | ^ | rf_set_acc_code() | rf_set_acc_code(ACCESS_CODE) | set access code for RF | ^ |
| ^ | ^ | rf_set_rx_buff() | rf_set_rx_buff(rx_packet,64, 0) | set buffer for rx packet | ^ |
| ^ | ^ | rf_set_rx_on() || start Rx mode | ^ |
| ^ | main_loop() | if( rf_is_rx_finish() ) | rf_is_rx_finish() | determine whether rx packet is finished | ^ |
| ^ | ^ | > if(RF_ZIGBEE_PACKET_CRC_OK(rx_packet)&&RF_ZIGBEE_PACKET_LENGTH_OK(rx_packet)) || determine whether rx packet is right | ^ |
| ^ | ^ | >> rx_cnt++ || Perform packet parsing and processing | ^ |
| ^ | ^ | > rf_clr_rx_finish() || clear the flag of rx finished | ^ |
| ^ | ^ | > rf_set_tx_rx_off_auto() || turn off RF auto mode | ^ |

\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}

#define RF_FREQ				35
#define RF_POWER			RF_POWER_P10p46dBm
#define ACCESS_CODE			0x29417671

volatile unsigned int rx_cnt=0;
unsigned char  rx_packet[64]  __attribute__ ((aligned (4)));

~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=RF_LR_S2_500K_TX> API-RF-CASE9:RF LR S2 500K TX </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-RF-CASE1:RF BLE 2M TX](group___g_p11.html#RF_BLE_2M_TX)

<h1 id=RF_LR_S2_500K_RX> API-RF-CASE10:RF LR S2 500K RX </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-RF-CASE2:RF BLE 2M RX](group___g_p11.html#RF_BLE_2M_RX)

<h1 id=RF_LR_S8_125K_TX> API-RF-CASE11:RF LR S8 125K TX </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-RF-CASE1:RF BLE 2M TX](group___g_p11.html#RF_BLE_2M_TX)

<h1 id=RF_LR_S8_125K_RX> API-RF-CASE912:RF LR S8 125K RX </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-RF-CASE2:RF BLE 2M RX](group___g_p11.html#RF_BLE_2M_RX)

<h1 id=RF_PRIVATE_250K_TX> API-RF-CASE13:RF PRIVATE 250K TX </h1>

| MODE | Function | Sub-Function | APIs || Description | Update Status |
|:-----| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| ESB | irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| ^ | main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | ^ | rf_mode_init() | rf_mode_init(RF_MODE_PRIVATE_250K) || RF mode initialization: RF_MODE_PRIVATE_250K | ^ |
| ^ | ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | ^ | user_init() | rf_set_power_level_index() | rf_set_power_level_index (RF_POWER) | set tx power level  | ^ |
| ^ | ^ | ^ | rf_set_tx_rx_off() || reset RF Tx/Rx mode  | ^ |
| ^ | ^ | ^ | rf_set_channel() | rf_set_channel(RF_FREQ,0) | set channel for RF | ^ |
| ^ | ^ | ^ | rf_set_acc_code() | rf_set_acc_code(ACCESS_CODE) | set access code for RF | ^ |
| ^ | ^ | ^ | rf_set_tx_on() || start Tx mode | ^ |
| ^ | ^ | main_loop() | rf_tx_pkt() | rf_tx_pkt(Private_ESB_tx_packet) | send packet in manual mode | ^ |
| ^ | ^ | ^ | rf_is_tx_finish() | while(!rf_is_tx_finish()) | wait for tx packet being finished | ^ |
| ^ | ^ | ^ | rf_clr_tx_finish() || clear the flag of tx finished | ^ |
| SB | irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| ^ | main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | ^ | rf_mode_init() | rf_mode_init(RF_MODE_LR_S8_125K) || RF mode initialization: RF_MODE_LR_S8_125K | ^ |
| ^ | ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | ^ | user_init() | rf_set_power_level_index() | rf_set_power_level_index (RF_POWER) | set tx power level  | ^ |
| ^ | ^ | ^ | rf_set_tx_rx_off() || reset RF Tx/Rx mode  | ^ |
| ^ | ^ | ^ | rf_set_channel() | rf_set_channel(RF_FREQ,0) | set channel for RF | ^ |
| ^ | ^ | ^ | rf_set_acc_code() | rf_set_acc_code(ACCESS_CODE) | set access code for RF | ^ |
| ^ | ^ | ^ | rf_pri_set_shockburst_len() | rf_pri_set_shockburst_len(RX_PAYLOAD_LEN) | set the length of shockburst for RF Private mode | ^ |
| ^ | ^ | ^ | rf_set_tx_on() || start Tx mode | ^ |
| ^ | ^ | main_loop() | rf_tx_pkt() | rf_tx_pkt(Private_SB_tx_packet) | send packet in manual mode | ^ |
| ^ | ^ | ^ | rf_is_tx_finish() | while(!rf_is_tx_finish()) | wait for tx packet being finished | ^ |
| ^ | ^ | ^ | rf_clr_tx_finish() || clear the flag of tx finished | ^ |

\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}

#define RF_FREQ				35
#define RF_POWER			RF_POWER_P10p46dBm
#define ACCESS_CODE			0x29417671
#define RX_PAYLOAD_LEN		32

volatile unsigned int tx_cnt=0;
unsigned char  Private_SB_tx_packet[48] __attribute__ ((aligned (4))) = {0x20,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};
unsigned char  Private_ESB_tx_packet[48] __attribute__ ((aligned (4))) = {0x21,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};

~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=RF_PRIVATE_250K_RX> API-RF-CASE14:RF PRIVATE 250K RX </h1>

| MODE | Function | Sub-Function | APIs || Description | Update Status |
|:-----| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| ESB | irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| ^ | main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | ^ | rf_mode_init() | rf_mode_init(RF_MODE_PRIVATE_250K) || RF mode initialization: RF_MODE_PRIVATE_250K | ^ |
| ^ | ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | ^ | user_init() | rf_set_power_level_index() | rf_set_power_level_index (RF_POWER) | set tx power level  | ^ |
| ^ | ^ | ^ | rf_set_tx_rx_off() || reset RF Tx/Rx mode  | ^ |
| ^ | ^ | ^ | rf_set_channel() | rf_set_channel(RF_FREQ,0) | set channel for RF | ^ |
| ^ | ^ | ^ | rf_set_acc_code() | rf_set_acc_code(ACCESS_CODE) | set access code for RF | ^ |
| ^ | ^ | ^ | rf_set_rx_buff() | rf_set_rx_buff(rx_packet,64, 0) | set buffer for rx packet | ^ |
| ^ | ^ | ^ | rf_set_rx_on() || start Rx mode | ^ |
| ^ | ^ | main_loop() | if( rf_is_rx_finish() ) | rf_is_rx_finish() | determine whether rx packet is finished | ^ |
| ^ | ^ | ^ | > if(RF_NRF_ESB_PACKET_CRC_OK(rx_packet)&&RF_PRI_PACKET_LENGTH_OK(rx_packet)) || determine whether rx packet is right | ^ |
| ^ | ^ | ^ | >> rx_cnt++ || Perform packet parsing and processing | ^ |
| ^ | ^ | ^ | > rf_clr_rx_finish() || clear the flag of rx finished | ^ |
| ^ | ^ | ^ | > rf_set_tx_rx_off_auto() || turn off RF auto mode | ^ |
| SB | irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| ^ | main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | ^ | rf_mode_init() | rf_mode_init(RF_MODE_PRIVATE_250K) || RF mode initialization: RF_MODE_PRIVATE_250K | ^ |
| ^ | ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | ^ | user_init() | rf_set_power_level_index() | rf_set_power_level_index (RF_POWER) | set tx power level  | ^ |
| ^ | ^ | ^ | rf_set_tx_rx_off() || reset RF Tx/Rx mode  | ^ |
| ^ | ^ | ^ | rf_set_channel() | rf_set_channel(RF_FREQ,0) | set channel for RF | ^ |
| ^ | ^ | ^ | rf_set_acc_code() | rf_set_acc_code(ACCESS_CODE) | set access code for RF | ^ |
| ^ | ^ | ^ | rf_pri_set_shockburst_len() | rf_pri_set_shockburst_len(RX_PAYLOAD_LEN) | set the length of shockburst for RF Private mode | ^ |
| ^ | ^ | ^ | rf_set_rx_buff() | rf_set_rx_buff(rx_packet,64, 0) | set buffer for rx packet | ^ |
| ^ | ^ | ^ | rf_set_rx_on() || start Rx mode | ^ |
| ^ | ^ | main_loop() | if( rf_is_rx_finish() ) | rf_is_rx_finish() | determine whether rx packet is finished | ^ |
| ^ | ^ | ^ | > if(RF_NRF_SB_PACKET_CRC_OK(rx_packet)) | RF_NRF_SB_PACKET_CRC_OK() | determine whether rx packet is right | ^ |
| ^ | ^ | ^ | >> rx_cnt++ || Perform packet parsing and processing | ^ |
| ^ | ^ | ^ | > rf_clr_rx_finish() || clear the flag of rx finished | ^ |
| ^ | ^ | ^ | > rf_set_tx_rx_off_auto() || turn off RF auto mode | ^ |
\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}

#define RF_FREQ				35
#define RF_POWER			RF_POWER_P10p46dBm
#define ACCESS_CODE			0x29417671
#define RX_PAYLOAD_LEN		32

volatile unsigned int rx_cnt=0;
unsigned char  rx_packet[64]  __attribute__ ((aligned (4)));
~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=RF_PRIVATE_500K_TX> API-RF-CASE15:RF PRIVATE 500K TX </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-RF-CASE13:RF PRIVATE 250K TX](group___g_p11.html#RF_PRIVATE_250K_TX)

<h1 id=RF_PRIVATE_500K_RX> API-RF-CASE16:RF PRIVATE 500K RX </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-RF-CASE14:RF PRIVATE 250K RX](group___g_p11.html#RF_PRIVATE_250K_RX)

<h1 id=RF_PRIVATE_1M_TX> API-RF-CASE17:RF PRIVATE 1M TX </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-RF-CASE13:RF PRIVATE 250K TX](group___g_p11.html#RF_PRIVATE_250K_TX)

<h1 id=RF_PRIVATE_1M_RX> API-RF-CASE18:RF PRIVATE 1M RX </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-RF-CASE14:RF PRIVATE 250K RX](group___g_p11.html#RF_PRIVATE_250K_RX)

<h1 id=RF_PRIVATE_2M_TX> API-RF-CASE19:RF PRIVATE 2M TX </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-RF-CASE13:RF PRIVATE 250K TX](group___g_p11.html#RF_PRIVATE_250K_TX)

<h1 id=RF_PRIVATE_2M_RX> API-RF-CASE20:RF PRIVATE 2M RX </h1>

&nbsp;&nbsp;&nbsp;&nbsp;refer to [API-RF-CASE14:RF PRIVATE 250K RX](group___g_p11.html#RF_PRIVATE_250K_RX)

<h1 id=RF_ANT_TX> API-RF-CASE21:RF ANT TX </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_ANT) || RF mode initialization: RF_MODE_ANT | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | rf_set_power_level_index() | rf_set_power_level_index (RF_POWER) | set tx power level  | ^ |
| ^ | ^ | rf_set_tx_rx_off() || reset RF Tx/Rx mode  | ^ |
| ^ | ^ | rf_set_channel() | rf_set_channel(RF_FREQ,0) | set channel for RF | ^ |
| ^ | ^ | rf_set_acc_code() | rf_set_acc_code(ACCESS_CODE) | set access code for RF | ^ |
| ^ | ^ | rf_pri_set_shockburst_len() | rf_pri_set_shockburst_len(RX_PAYLOAD_LEN) | set the length of shockburst for RF ANT mode | ^ |
| ^ | ^ | rf_set_tx_on() || start Tx mode | ^ |
| ^ | main_loop() | rf_tx_pkt() | rf_tx_pkt(Ant_tx_packet) | send packet in manual mode | ^ |
| ^ | ^ | rf_is_tx_finish() | while(!rf_is_tx_finish()) | wait for tx packet being finished | ^ |
| ^ | ^ | rf_clr_tx_finish() || clear the flag of tx finished | ^ |

\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}

#define RF_FREQ				35
#define RF_POWER			RF_POWER_P10p46dBm
#define ACCESS_CODE			0x29417671
#define RX_PAYLOAD_LEN		32

volatile unsigned int tx_cnt=0;
unsigned char  Ant_tx_packet[48] __attribute__ ((aligned (4))) = {RX_PAYLOAD_LEN-2,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};

~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=RF_ANT_RX> API-RF-CASE22:RF ANT RX </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_ANT) || RF mode initialization: RF_MODE_ANT | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | rf_set_power_level_index() | rf_set_power_level_index (RF_POWER) | set tx power level  | ^ |
| ^ | ^ | rf_set_tx_rx_off() || reset RF Tx/Rx mode  | ^ |
| ^ | ^ | rf_set_channel() | rf_set_channel(RF_FREQ,0) | set channel for RF | ^ |
| ^ | ^ | rf_set_acc_code() | rf_set_acc_code(ACCESS_CODE) | set access code for RF | ^ |
| ^ | ^ | rf_pri_set_shockburst_len() | rf_pri_set_shockburst_len(RX_PAYLOAD_LEN) | set the length of shockburst for RF ANT mode | ^ |
| ^ | ^ | rf_set_rx_buff() | rf_set_rx_buff(rx_packet,64, 0) | set buffer for rx packet | ^ |
| ^ | ^ | rf_set_rx_on() || start Rx mode | ^ |
| ^ | main_loop() | if( rf_is_rx_finish() ) | rf_is_rx_finish() | determine whether rx packet is finished | ^ |
| ^ | ^ | > if(rf_is_rx_right()) | rf_is_rx_right() | determine whether rx packet is right | ^ |
| ^ | ^ | >> rx_cnt++ || Perform packet parsing and processing | ^ |
| ^ | ^ | > rf_clr_rx_finish() || clear the flag of rx finished | ^ |
| ^ | ^ | > rf_set_tx_rx_off_auto() || turn off RF auto mode | ^ |

\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}

#define RF_FREQ				35
#define RF_POWER			RF_POWER_P10p46dBm
#define ACCESS_CODE			0x29417671
#define RX_PAYLOAD_LEN		32

volatile unsigned int rx_cnt=0;

~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=RF_BLE_2M_AUTO_TX> API-RF-CASE23:RF BLE 2M AUTO TX </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_2M) || RF mode initialization: RF_MODE_BLE_2M | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | rf_set_power_level_index() | rf_set_power_level_index (RF_POWER) | set tx power level  | ^ |
| ^ | ^ | rf_set_tx_rx_off() || reset RF Tx/Rx mode  | ^ |
| ^ | ^ | rf_set_channel() | rf_set_channel(RF_FREQ,0) | set channel for RF | ^ |
| ^ | ^ | rf_set_acc_code() | rf_set_acc_code(ACCESS_CODE) | set access code for RF | ^ |
| ^ | main_loop() | rf_start_stx() | rf_start_stx(ble_tx_packet, get_sys_tick() + 16*1000*TX_INTERVAL_MS) | send one packet every 1ms in auto mode and close | ^ |
| ^ | ^ | rf_is_tx_finish() | while(!rf_is_tx_finish()) | wait for tx packet being finished | ^ |
| ^ | ^ | rf_clr_tx_finish() || clear the flag of tx finished | ^ |

\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}

#define RF_FREQ				35
#define RF_POWER			RF_POWER_P10p46dBm
#define ACCESS_CODE			0x29417671
#define TX_INTERVAL_MS    	1

volatile unsigned int tx_cnt=0;
unsigned char  ble_tx_packet[48] __attribute__ ((aligned (4))) = {0x23,0x00,0x00,0x00,0x00,0x21,0x00,0x00,0x00,0x00,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};

~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=RF_BLE_2M_AUTO_TX> API-RF-CASE24:RF BLE 2M AUTO RX </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | none ||| Interrupt handler function | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_2M) || RF mode initialization: RF_MODE_BLE_2M | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | rf_set_power_level_index() | rf_set_power_level_index (RF_POWER) | set tx power level  | ^ |
| ^ | ^ | rf_set_tx_rx_off() || reset RF Tx/Rx mode  | ^ |
| ^ | ^ | rf_set_channel() | rf_set_channel(RF_FREQ,0) | set channel for RF | ^ |
| ^ | ^ | rf_set_acc_code() | rf_set_acc_code(ACCESS_CODE) | set access code for RF | ^ |
| ^ | ^ | rf_set_rx_buff() | rf_set_rx_buff(rx_packet,64, 0) | set buffer for rx packet | ^ |
| ^ | ^ | rf_start_srx() | rf_start_srx(get_sys_tick() + 100*16) | start SRX mode | ^ |
| ^ | main_loop() | if( rf_is_rx_finish() ) | rf_is_rx_finish() | determine whether rx packet is finished | ^ |
| ^ | ^ | > if(RF_BLE_PACKET_CRC_OK(rx_packet)&&RF_BLE_PACKET_LENGTH_OK(rx_packet)) || determine whether rx packet is right | ^ |
| ^ | ^ | >> rx_cnt++ || Perform packet parsing and processing | ^ |
| ^ | ^ | > rf_clr_rx_finish() || clear the flag of rx finished | ^ |
| ^ | ^ | > rf_set_tx_rx_off_auto() || turn off RF auto mode | ^ |
| ^ | ^ | > rf_start_srx() | rf_start_srx(get_sys_tick() + 100*16) | start SRX mode again | ^ |

\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}

#define RF_FREQ				35
#define RF_POWER			RF_POWER_P10p46dBm
#define ACCESS_CODE			0x29417671

volatile unsigned int rx_cnt=0;
__attribute__ ((aligned (4))) unsigned char  rx_packet[64];

~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=RF_BLE_2M_AUTO_TX2RX> API-RF-CASE25:RF BLE 2M AUTO TX2RX </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | Tx IRQ | if(rf_irq_get_src()&FLD_RF_IRQ_TX==FLD_RF_IRQ_TX) | rf_irq_get_src() | determine whether interrupt flag is generated by Tx | 2019-1-10 |
| ^ | ^ | >rf_irq_clr_src(FLD_RF_IRQ_TX) | rf_irq_clr_src() | clear the interrupt flag | 2019-1-10 |
| ^ | ^ | >tx_state = 1 || get tx state | 2019-1-10 |
| ^ | Rx IRQ | if(rf_irq_get_src()&FLD_RF_IRQ_RX==FLD_RF_IRQ_RX) | rf_irq_get_src() | determine whether interrupt flag is generated by Rx | 2019-1-10 |
| ^ | ^ | >rf_irq_clr_src(FLD_RF_IRQ_RX) | rf_irq_clr_src() | clear the interrupt flag | 2019-1-10 |
| ^ | ^ | >rx_state = 1 || get rx state | 2019-1-10 |
| ^ | TX2RX Rx TIMEOUT IRQ | if(rf_irq_get_src()&FLD_RF_IRQ_TX2RX_RX_TIMEOUT==FLD_RF_IRQ_TX2RX_RX_TIMEOUT) || determine whether interrupt flag is generated by Rx timeout | 2019-1-10 |
| ^ | ^ | >rf_irq_clr_src(FLD_RF_IRQ_TX2RX_RX_TIMEOUT) | rf_irq_clr_src() | clear the interrupt flag | 2019-1-10 |
| ^ | ^ | >timeout_state = 1 || get timeout state | 2019-1-10 |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_2M) || RF mode initialization: RF_MODE_BLE_2M | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | rf_set_power_level_index() | rf_set_power_level_index (RF_POWER) | set tx power level  | ^ |
| ^ | ^ | rf_set_trx_state() | rf_set_trx_state(RF_MODE_AUTO,RF_FREQ) | set Tx/Rx auto mode  | ^ |
| ^ | ^ | rf_set_acc_code() | rf_set_acc_code(ACCESS_CODE) | set access code for RF | ^ |
| ^ | ^ | rf_set_rx_buff() | rf_set_rx_buff(rx_packet,64, 0) | set buffer for rx packet | ^ |
| ^ | ^ | irq_disable() || disable global interrupt | ^ |
| ^ | ^ | irq_clr_all_src() || clear global interrupt flag | ^ |
| ^ | ^ | irq_set_mask() | irq_set_mask(FLD_IRQ_ZB_RT_EN) | set global interrupt mask | ^ |
| ^ | ^ | rf_irq_clr_mask() | rf_irq_clr_mask(FLD_RF_IRQ_ALL) | clear all rf irq mask | ^ |
| ^ | ^ | rf_irq_set_mask() | rf_irq_set_mask(FLD_RF_IRQ_TX <br> &Iota; FLD_RF_IRQ_RX <br> &Iota; FLD_RF_IRQ_TX2RX_RX_TIMEOUT) | set rf interrupt mask | ^ |
| ^ | ^ | irq_enable() || enable global interrupt | ^ |
| ^ | main_loop() | tx_state=0 <br> rx_state=0 <br> timeout_state=0 || initiate the Tx/Rx/timeout state | ^ |
| ^ | ^ | delay_ms(100) | delay_ms() | delay 100ms | ^ |
| ^ | ^ | rf_start_stx2rx (ble_tx_packet, <br> get_sys_tick()+16*TX_DELAY_US,STX_WAITTIME_US) | rf_start_stx2rx() | start Tx2Rx | ^ |
| ^ | ^ | ... || for more details , please refer to [Driver Demo](#) | ^ |


\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}

#define STX_WAITTIME_US         4000
#define SRX_WAITTIME_US         100000
#define TX_DELAY_US             10

#define RF_FREQ					35
#define RF_POWER				RF_POWER_P10p46dBm
#define ACCESS_CODE				0x29417671

volatile unsigned int tx_state=0;
volatile unsigned int rx_state=0;
volatile unsigned int timeout_state=0;

unsigned char  rx_packet[64]  __attribute__ ((aligned (4)));
unsigned char  ble_tx_packet[48] __attribute__ ((aligned (4))) = {0x23,0x00,0x00,0x00,0x00,0x21,0x00,0x00,0x00,0x00,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};

~~~~~~~~~~~~~~~~~~~~~~~~~~~

<h1 id=RF_BLE_2M_AUTO_RX2TX> API-RF-CASE26:RF BLE 2M AUTO RX2TX </h1>

| Function | Sub-Function | APIs || Description | Update Status |
| :------- | :----------- | :---------- | :---------- |:---------- | :------------ |
| irq_handler() | Tx IRQ | if(rf_irq_get_src()&FLD_RF_IRQ_TX==FLD_RF_IRQ_TX) | rf_irq_get_src() | determine whether interrupt flag is generated by Tx | 2019-1-10 |
| ^ | ^ | >rf_irq_clr_src(FLD_RF_IRQ_TX) | rf_irq_clr_src() | clear the interrupt flag | ^ |
| ^ | ^ | >tx_state = 1 || get tx state | ^ |
| ^ | Rx IRQ | if(rf_irq_get_src()&FLD_RF_IRQ_RX==FLD_RF_IRQ_RX) | rf_irq_get_src() | determine whether interrupt flag is generated by Rx | ^ |
| ^ | ^ | >rf_irq_clr_src(FLD_RF_IRQ_RX) | rf_irq_clr_src() | clear the interrupt flag | ^ |
| ^ | ^ | >rx_state = 1 || get rx state | ^ |
| ^ | RX2TX Rx TIMEOUT IRQ | if(rf_irq_get_src()&FLD_RF_IRQ_RX2TX_RX_TIMEOUT==FLD_RF_IRQ_RX2TX_RX_TIMEOUT) || determine whether interrupt flag is generated by Rx timeout | ^ |
| ^ | ^ | >rf_irq_clr_src(FLD_RF_IRQ_RX2TX_RX_TIMEOUT) | rf_irq_clr_src() | clear the interrupt flag | ^ |
| ^ | ^ | >timeout_state = 1 || get timeout state | ^ |
| main() | system_init() ||| CPU initialization function [**Mandatory**] | ^ |
| ^ | clock_init() | clock_init(SYS_CLK_24M_XTAL) || Clock initialization function, System Clock is 24M RC by default [**optional**] | ^ |
| ^ | rf_mode_init() | rf_mode_init(RF_MODE_BLE_2M) || RF mode initialization: RF_MODE_BLE_2M | ^ |
| ^ | gpio_init() ||| GPIO initialization: set the initialization status of all GPIOs [**optional**] | ^ |
| ^ | user_init() | rf_set_power_level_index() | rf_set_power_level_index (RF_POWER) | set tx power level  | ^ |
| ^ | ^ | rf_set_trx_state() | rf_set_trx_state(RF_MODE_AUTO,RF_FREQ) | set Tx/Rx auto mode  | ^ |
| ^ | ^ | rf_set_acc_code() | rf_set_acc_code(ACCESS_CODE) | set access code for RF | ^ |
| ^ | ^ | rf_set_rx_buff() | rf_set_rx_buff(rx_packet,64, 0) | set buffer for rx packet | ^ |
| ^ | ^ | irq_disable() || disable global interrupt | ^ |
| ^ | ^ | irq_clr_all_src() || clear global interrupt flag | ^ |
| ^ | ^ | irq_set_mask() | irq_set_mask(FLD_IRQ_ZB_RT_EN) | set global interrupt mask | ^ |
| ^ | ^ | rf_irq_clr_mask() | rf_irq_clr_mask(FLD_RF_IRQ_ALL) | clear all rf irq mask | ^ |
| ^ | ^ | rf_irq_set_mask() | rf_irq_set_mask(FLD_RF_IRQ_TX <br> &Iota; FLD_RF_IRQ_RX <br> &Iota; FLD_RF_IRQ_RX2TX_RX_TIMEOUT) | set rf interrupt mask | ^ |
| ^ | ^ | irq_enable() || enable global interrupt | ^ |
| ^ | main_loop() | tx_state=0 <br> rx_state=0 <br> timeout_state=0 || initiate the Tx/Rx/timeout state | ^ |
| ^ | ^ | rf_start_srx2tx(ble_tx_packet, <br> get_sys_tick()+16*TX_DELAY_US ,SRX_WAITTIME_US) | rf_start_srx2tx() | start Tx2Rx | ^ |
| ^ | ^ | ... || for more details , please refer to [Driver Demo](#) | ^ |

\n
Variables above are defined as below
~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}

#define STX_WAITTIME_US         4000
#define SRX_WAITTIME_US         100000
#define TX_DELAY_US             10

#define RF_FREQ					35
#define RF_POWER				RF_POWER_P10p46dBm
#define ACCESS_CODE				0x29417671

volatile unsigned int tx_state=0;
volatile unsigned int rx_state=0;
volatile unsigned int timeout_state=0;

unsigned char  rx_packet[64]  __attribute__ ((aligned (4)));
unsigned char  ble_tx_packet[48] __attribute__ ((aligned (4))) = {0x23,0x00,0x00,0x00,0x00,0x21,0x00,0x00,0x00,0x00,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};

~~~~~~~~~~~~~~~~~~~~~~~~~~~


<h1> History Record </h1>

| Date | Description | Author |
| :--- | :---------- | :----- |
| 2019-1-10 | initial release | LJW |


*/

 /** @}*/ //end of GP11
