#include "register.h"
#include "gpio.h"

/**
 *  @brief  Define mul bits
 */
enum{
	UART_BW_MUL1  = 0,
	UART_BW_MUL2  = 1,
	UART_BW_MUL3  = 2,
	UART_BW_MUL4  = 3,
};
// tX:
typedef enum{
	UART_TX_PB4 = GPIO_PB4,
	UART_TX_PC4 = GPIO_PC4,
	UART_TX_PD2 = GPIO_PD2,

}UART_TxPinDef;

// rx:
typedef enum{
	UART_RX_PB5 = GPIO_PB5,
	UART_RX_PC5 = GPIO_PC5,
	UART_RX_PD3 = GPIO_PD3,

}UART_RxPinDef;

/* 5317 UART hardware flow pin define. */

//RTS: GPIO_PC3  GPIO_PD1
//CTS: GPIO_PC2
typedef enum
{
	UART_CTS_PC2 = GPIO_PC2,
}UART_CtsPinDef;
typedef enum
{
	UART_RTS_PC3 = GPIO_PC3,//Failure
	UART_RTS_PD1 = GPIO_PD1,//0K
}UART_RtsPinDef;




typedef enum {
    UART_RTS_MODE_AUTO = 0,
    UART_RTS_MODE_MANUAL,
} UART_RTSModeTypeDef;


/**
 *  @brief  Define parity type
 */
typedef enum {
    PARITY_NONE = 0,
    PARITY_EVEN,
    PARITY_ODD,
}UART_ParityTypeDef;

/**
 *  @brief  Define the length of stop bit
 */
typedef enum {
    STOP_BIT_ONE = 0,
    STOP_BIT_ONE_DOT_FIVE = BIT(12),
    STOP_BIT_TWO = BIT(13),
}UART_StopBitTypeDef;

/* UART interrupt flag define. only use for DMA mode of UART. */
typedef enum{
	UART_Flag_RxDone = 0x01,
	UART_Flag_TxDone = 0x02,
}UART_IrqFlagTypeDef;


enum {
	UART_DMA_RX_IRQ_DIS = 0,
	UART_DMA_RX_IRQ_EN  = 1,
	UART_DMA_TX_IRQ_DIS = 0,
	UART_DMA_TX_IRQ_EN  = 1,
};

enum {
	UART_NODMA_RX_IRQ_DIS = 0,
	UART_NODMA_RX_IRQ_EN  = 1,
	UART_NODMA_TX_IRQ_DIS = 0,
	UART_NODMA_TX_IRQ_EN  = 1,
};

#if (MCU_CORE_TYPE == MCU_CORE_5316)
                     	 //TxRx
	#define UART_GPIO_INIT_PA3A4()   uart_pin_init(UART_PIN_PA3A4)
	#define UART_GPIO_INIT_PB4B5()   uart_pin_init(UART_PIN_PB4B5)
	#define UART_GPIO_INIT_PC4C5()   uart_pin_init(UART_PIN_PC4C5)
#elif(MCU_CORE_TYPE == MCU_CORE_5317)
						  //TxRx
	#define UART_GPIO_INIT_PA1A2()   uart_pin_init(UART_PIN_PA1A2)
	#define UART_GPIO_INIT_PB4B5()   uart_pin_init(UART_PIN_PB4B5)
	#define UART_GPIO_INIT_PC4C5()   uart_pin_init(UART_PIN_PC4C5)
#endif


//extern void uart_pin_init(UART_PinTypeDef uartPin);
/**
 *	@brief	reset uart module
 *	@param	none
 *	@return	none
 */
extern void uart_reset(void);
/**
 *	@brief	clear error state of uart rx, maybe used when application detected UART not work
 *	@parm	none
 *	@return	'1' RX error flag rised and cleard success; '0' RX error flag not rised
 */
extern unsigned char uart_ErrorCLR(void);
/**
 * @brief      This function initializes the UART module.
 * @param[in]  Baudrate  	- uart baud rate
 * @param[in]  System_clock - clock of system
 * @param	   *RecvAddr:	receive buffer's address info.
 * @param	   RecvBufLen:	receive buffer's length, the maximum uart packet length should be smaller than (size - 4)
 * @return	   none
 * @return     none
 */
extern void uart_init_baudrate(unsigned int baudrate,unsigned int System_clock, unsigned char* RecvAddr, unsigned short RecvBufLen);
/*
* @brief      This function initializes the UART Parity.
* @param[in]  Parity      	- selected parity type for UART interface
* @return     none
*/
void uart_Parity(UART_ParityTypeDef Parity);
/**
 * @brief      This function initializes the UART StopBit.
 * @param[in]  StopBit     	- selected length of stop bit for UART interface
 * @return     none
 */
void uart_init_StopBit(UART_StopBitTypeDef StopBit);
/* Only use for Normal Mode. */
/**
 * @brief     config the number level setting the irq bit of status register 0x9d
 *            ie 0x9d[3].
 *            If the cnt register value(0x9c[0,3]) larger or equal than the value of 0x99[0,3]
 *            or the cnt register value(0x9c[4,7]) less or equal than the value of 0x99[4,7],
 *            it will set the irq bit of status register 0x9d, ie 0x9d[3]
 * @param[in] rx_level - receive level value. ie 0x99[0,3]
 * @param[in] tx_level - transmit level value.ie 0x99[4,7]
 * @return    none
 */


void uart_ndma_set_triglevel(unsigned char rx_level, unsigned char tx_level);
/**
 * @brief     config the number level setting the irq bit of status register 0x9d
 *            ie 0x9d[3].
 *            If the cnt register value(0x9c[0,3]) larger or equal than the value of 0x99[0,3]
 *            or the cnt register value(0x9c[4,7]) less or equal than the value of 0x99[4,7],
 *            it will set the irq bit of status register 0x9d, ie 0x9d[3]
 * @param[in] rx_level - receive level value. ie 0x99[0,3]
 * @param[in] tx_level - transmit level value.ie 0x99[4,7]
 * @return    none
 */


extern void uart_not_dma_mode_init(unsigned char rx_level,unsigned char tx_level,
		                        unsigned char rx_irq_en,unsigned char tx_irq_en);
extern unsigned char rx_id;
/**
 * @ brief   in not dma mode, receive the data.
 *           the method to read data should be like this: read received data in the order from 0x90 to 0x93.
 *           then repeat the order.
 * @ param[in] none
 * @ return    the data received from the uart.
 */
extern unsigned char uart_not_dma_mode_rev_data(void);
extern unsigned char tx_id;
/**
 * @brief     uart send data function with not DMA method.
 *            variable uart_TxIndex,it must loop the four registers 0x90 0x91 0x92 0x93 for the design of SOC.
 *            so we need variable to remember the index.
 * @param[in] uartData - the data to be send.
 * @return    1: send success ; 0: uart busy
 */
void uart_ndma_send_byte(unsigned char uartData);
//interrupt flag will be cleared automatically
/**
 * @Brief: Only use for Normal mode of UART.(GaoQiu add)
 * @Param:
 * @Return:
 */
extern unsigned char uart_ndma_get_irq(void);

/**
 * @brief     uart send data function, this  function tell the DMA to get data from the RAM and start
 *            the DMA transmission
 * @param[in] byte - single byte data need to send
 * @return    1: send success ;
 *            0: DMA busy
 */
volatile unsigned char uart_dma_send_byte(unsigned char byte);
/* Only use for DMA mode */
/**
 * @brief     enable uart DMA mode,config uart dam interrupt.
 * @param[in] dmaTxIrqEn -- whether or not enable UART TX interrupt.
 * @param[in] dmaRxIrqEn -- whether or not enable UART RX interrupt.
 * @return    none
 */
extern void uart_dma_mode_init(unsigned char dmaTxIrqEn, unsigned char dmaRxIrqEn);
/**
 * @brief     enable uart DMA mode
 * @param[in] none
 * @return    none
 */
extern void uart_dma_en(unsigned char rx_dma_en, unsigned char tx_dma_en);

/**
 * @brief     config the irq of uart tx and rx
 * @param[in] rx_irq_en - 1:enable rx irq. 0:disable rx irq
 * @param[in] tx_irq_en - 1:enable tx irq. 0:disable tx irq
 * @return    none
 */
extern void uart_irq_en(unsigned char rx_irq_en,unsigned char tx_irq_en);
/**
 *	@brief	data receive buffer initiate function. DMA would move received uart data to the address space, uart packet length
 *			needs to be no larger than (recBuffLen - 4).
 *	@param	*recAddr:	receive buffer's address info.
 *			recBuffLen:	receive buffer's length, the maximum uart packet length should be smaller than (recBuffLen - 4)
 *	@return	none
 */

extern void uart_set_recbuff(unsigned short *RecvAddr, unsigned short RecvBufLen);
/**
 *	@brief	uart send data function, this  function tell the DMA to get data from the RAM and start
 *			the DMA send function
 *	@param	sendBuff - send data buffer
 *	@return	'1' send success; '0' DMA busy
 */

extern unsigned char uart_tx_is_busy(void);

/* Use for Hardware Flow of UART */
/**
 * @Brief:  UART CTS/RTS Pin initialization.
 * @Param:  flowCtrlPin -> CTS/RTS Pin.
 * @Retval: None.
 */

extern void uart_set_cts(unsigned char ctsEnable,unsigned char pinValue,UART_CtsPinDef pin);
/**
 * @brief     UART hardware flow control configuration. Configure RTS pin.
 * @param[in] Enable - enable or disable RTS function.
 * @param[in] Mode - set the mode of RTS(auto or manual).
 * @param[in] Thresh - threshold of trig RTS pin's level toggle(only for auto mode),
 *                     it means the number of bytes that has arrived in Rx buf.
 * @param[in] Invert - whether invert the output of RTS pin(only for auto mode)
 * @param[in] GPIO   - RTS pin select,it can be GPIO_PA4/GPIO_PB3/GPIO_PB6/GPIO_PC0.
 * @return    none
 */
extern void uart_set_rts(unsigned char Enable, UART_RTSModeTypeDef Mode, unsigned char Thresh, unsigned char Invert, UART_RtsPinDef pin);
/**
 * @brief     This function determines whether parity error occurs once a packet arrives.
 * @param[in] none
 * @return    1: parity error ;
 *            0: no parity error
 */
extern unsigned char uart_is_parity_error(void);
/**
 * @brief     This function clears parity error status once when it occurs.
 * @param[in] none
 * @return    none
 */
extern void uart_clear_parity_error(void);

/* Reserved for compatibility */
enum UARTIRQSOURCE{
	UARTNONEIRQ = 0,
	UARTRXIRQ = BIT(0),
	UARTTXIRQ = BIT(1),
};

enum{
	UARTRXIRQ_MASK  = BIT(0),
	UARTTXIRQ_MASK  = BIT(1),
	UARTIRQ_MASK    = UARTRXIRQ_MASK | UARTTXIRQ_MASK,
};

//use for DMA mode
/**
 *	@brief	uart send data function, this  function tell the DMA to get data from the RAM and start
 *			the DMA send function
 *	@param	sendBuff - send data buffer
 *	@return	'1' send success; '0' DMA busy
 */
extern unsigned char uart_dma_send(unsigned short* Addr);

/**
 *define the macro that configures pin port for UART interface
 *@param  tx_pin -To set TX pin
 *@param  rx_pin -To set RX pin
 *none
 */

extern void uart_set_pin(UART_TxPinDef tx_pin,UART_RxPinDef rx_pin);


/** \defgroup GP14  UART Usage
 *  This is the first Group
 *  @{
 */

//-----------------------------------------------------------1-14
/*! \page uart UART Usage
This page is for ...
details.
*/

 /** @}*/ //end of GP14
