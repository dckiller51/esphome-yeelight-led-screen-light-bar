/********************************************************************************************************
 * @file     uart.c
 *
 * @brief    This is the source file for TLSR8232
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
#include "uart.h"
#include "register.h"
#include "gpio.h"

#define tickperus                   16
#define BLE_GET_LIST_BWPC 0
/**
 *	@brief	reset uart module
 *	@param	none
 *	@return	none
 */
void uart_reset(void){
	BM_SET(reg_rst1, FLD_RST1_RS232);
	BM_CLR(reg_rst1, FLD_RST1_RS232);
}


/**
 * @Brief: UART Pin initialization.
 * @Param: uartPin ->
 * @ReVal: None.
 */

/**
 *define the macro that configures pin port for UART interface
 *@param  tx_pin -To set TX pin
 *@param  rx_pin -To set RX pin
 *none
 */

void uart_set_pin(UART_TxPinDef tx_pin,UART_RxPinDef rx_pin)
{
	unsigned char val = 0;
	unsigned char mask = 0xff;
	unsigned char bit;
	unsigned short reg;
	//note: pullup setting must before uart gpio config, cause it will lead to ERR data to uart RX buffer(confirmed by sihui&sunpeng)
	//PM_PIN_PULLUP_1M   GPIO_PULL_UP_10K
	if( tx_pin==UART_TX_PB4 ){
		gpio_set_up_down_resistor(tx_pin, GPIO_PULL_UP_1M);  //must, for stability and prevent from current leakage
	}else{
		gpio_set_up_30k(tx_pin);
	}
	if( rx_pin==UART_RX_PB5 ){
		gpio_set_up_down_resistor(rx_pin, GPIO_PULL_UP_10K);  //must  for stability and prevent from current leakage
	}else{
		gpio_set_up_30k(rx_pin);
	}


	bit = tx_pin & 0xff;
	BM_CLR(reg_gpio_gpio_func(tx_pin), bit);
	bit = rx_pin & 0xff;
	BM_CLR(reg_gpio_gpio_func(rx_pin), bit);


	//TX: GPIO_PB4  GPIO_PC4
	//RX: GPIO_PB5  GPIO_PC5
	//RTS: GPIO_PC3  GPIO_PD1
	//CTS: GPIO_PC2

	if(tx_pin == UART_TX_PB4)
	{
		mask= (unsigned char)~(BIT(0)|BIT(1));
		val = BIT(1);
	}
	else if(tx_pin == UART_TX_PC4)
	{
		mask= (unsigned char)~(BIT(1)|BIT(0));
		val = BIT(0)|BIT(1);
	}
	else if(tx_pin == UART_TX_PD2)
	{
		mask= (unsigned char)~(BIT(4)|BIT(5));
		val = BIT(4);
	}
	reg = 0x5a8 + ((tx_pin>>8)<<1) + ((tx_pin&0x0f0) ? 1 : 0 );
	WRITE_REG8(reg, ( READ_REG8(reg) & mask) | val);


	if(rx_pin == UART_RX_PB5)
	{
		mask= (unsigned char)~(BIT(2)|BIT(3));
		val = BIT(3);
	}
	else if(rx_pin ==  UART_RX_PC5)
	{
		mask= (unsigned char)~(BIT(2)|BIT(3));
		val = BIT(2)|BIT(3);
	}
	else if(rx_pin ==  UART_RX_PD3)
	{
		mask= (unsigned char)~(BIT(6)|BIT(7));
		val = BIT(6);
	}
	reg = 0x5a8 + ((rx_pin>>8)<<1) + ((rx_pin&0x0f0) ? 1 : 0 );
	WRITE_REG8(reg, ( READ_REG8(reg) & mask) | val);

	gpio_set_input_en(tx_pin, 1);  //experiment shows that tx_pin should open input en(confirmed by qiuwei)
	gpio_set_input_en(rx_pin, 1);  //

}

/**
 * @brief     This function is used to look for the prime.if the prime is finded,it will
 * 			  return 1, or return 0.
 * @param[in] none
 * @return    none
 */
static unsigned char IsPrime(unsigned int n)
{
	unsigned int i = 5;
	if(n <= 3){
		return 1; //althought n is prime, but the bwpc must be larger than 2.
	}
	else if((n %2 == 0) || (n % 3 == 0)){
		return 0;
	}
	else{
		for(i=5;i*i<n;i+=6){
			if((n % i == 0)||(n %(i+2))==0){
				return 0;
			}
		}
		return 1;
	}
}


#if !BLE_GET_LIST_BWPC
/***********************************************************
 * @brief  calculate the best bwpc(bit width) .i.e reg0x96
 * @param[in] baut_rate -set baud rate
 * @param[in] tmp_sysclk - get the minimum one decimal point
 * @return the position of getting the minimum value
 */

static unsigned int  g_uart_div = 0;
static unsigned char g_bwpc = 0;
static void GetBetterBwpc(unsigned int baut_rate,unsigned int  tmp_sysclk )
{

	unsigned char i = 0, j= 0;
	unsigned int primeInt = 0;
	unsigned char primeDec = 0;
	unsigned int D_intdec[13],D_int[13];
	unsigned char D_dec[13];
//	unsigned int tmp_sysclk = tickperus*1000*1000;
	primeInt = tmp_sysclk/baut_rate;
	primeDec = 10*tmp_sysclk/baut_rate - 10*primeInt;
	/************************************************************
	 * calculate the primeInt and check whether primeInt is prime.
	 */
	if(IsPrime(primeInt)){ // primeInt is prime
		primeInt += 1;  //+1 must be not prime. and primeInt must be larger than 2.
	}
	else{
		if(primeDec > 5){ // >5
			primeInt += 1;
			if(IsPrime(primeInt)){
				primeInt -= 1;
			}
		}
	}
	/*******************************************
	 * get the best division value and bit width
	 */
	for(i=3;i<=15;i++){
		D_intdec[i-3] = (10*primeInt)/(i+1);////get the LSB
		D_dec[i-3] = D_intdec[i-3] - 10*(D_intdec[i-3]/10);///get the decimal section
		D_int[i-3] = D_intdec[i-3]/10;///get the integer section
	}

	//find the max and min one decimation point
	unsigned char position_min = 0,position_max = 0;
	unsigned int min = 0xffffffff,max = 0x00;
	for(j=0;j<13;j++){
		if((D_dec[j] <= min)&&(D_int[j] != 0x01)){
			min = D_dec[j];
			position_min = j;
		}
		if(D_dec[j]>=max){
			max = D_dec[j];
			position_max = j;
		}
	}

	if((D_dec[position_min]<5) && (D_dec[position_max]>=5)){
		if(D_dec[position_min]<(10-D_dec[position_max])){
			g_bwpc = position_min + 3;
			g_uart_div = D_int[position_min]-1;
		}
		else{
			g_bwpc = position_max + 3;
			g_uart_div = D_int[position_max];
		}
	}
	else if((D_dec[position_min]<5) && (D_dec[position_max]<5)){
		g_bwpc = position_min + 3;
		g_uart_div = D_int[position_min] - 1;
	}
	else{
		g_bwpc = position_max + 3;
		g_uart_div = D_int[position_max];
	}
}

#else

static void uartGetConfig(unsigned int baudrate, unsigned int* bw, unsigned int* div,unsigned int System_clock){
	if(System_clock == 8000000){
		if(baudrate == 9600){			//  err = 0.00040
			*bw = 	7;
			*div = 	119;
		}
		else if(baudrate == 19200){		//  err = 0.00160
			*bw = 	13;
			*div = 	32;
		}else if(baudrate == 38400){		//  err = 0.00160
			*bw = 	13;
			*div = 	16;
		}else if(baudrate == 115200){		//  err = 0.00800
			*bw = 	7;
			*div = 	10;
		}
	}else if(System_clock == 16000000){
		if(baudrate == 9600){			//  err = 0.00040
			*bw = 	14;
			*div = 	119;
		}else if(baudrate == 19200){		//  err = 0.00040
			*bw = 	7;
			*div = 	119;
		}else if(baudrate == 38400){		//  err = 0.00160
			*bw = 	13;
			*div = 	32;
		}else if(baudrate == 115200){		//  err = 0.00640
			*bw = 	6;
			*div = 	23;
		}
	}else if(System_clock == 24000000){
		if(baudrate == 9600){			//  err = 0.00000
			*bw = 	10;
			*div = 	250;
		}else if(baudrate == 19200){		//  err = 0.00000
			*bw = 	10;
			*div = 	125;
		}else if(baudrate == 38400){		//  err = 0.00000
			*bw = 	5;
			*div = 	125;
		}else if(baudrate == 115200){		//  err = 0.00160
			*bw = 	13;
			*div = 	16;
		}
	}else if(System_clock == 32000000){
		if(baudrate == 9600){			//  err = 0.00010
			*bw = 	11;
			*div = 	303;
		}else if(baudrate == 19200){		//  err = 0.00040
			*bw = 	14;
			*div = 	119;
		}else if(baudrate == 38400){		//  err = 0.00040
			*bw = 	7;
			*div = 	119;
		}else if(baudrate == 115200){		//  err = 0.00440
			*bw = 	9;
			*div = 	31;
		}
	}else if(System_clock == 48000000){
		if(baudrate == 9600){			//  err = 0.00000
			*bw = 	10;
			*div = 	500;
		}else if(baudrate == 19200){		//  err = 0.00000
			*bw = 	10;
			*div = 	250;
		}else if(baudrate == 38400){		//  err = 0.00000
			*bw = 	10;
			*div = 	125;
		}else if(baudrate == 115200){		//  err = 0.00160
			*bw = 	13;
			*div = 	32;
		}
	}
}

#endif
/**
 * @brief      This function initializes the UART StopBit.
 * @param[in]  StopBit     	- selected length of stop bit for UART interface
 * @return     none
 */
void uart_init_StopBit(UART_StopBitTypeDef StopBit)
{
	//stop bit config
	reg_uart_ctrl1  &= (~FLD_UART_CTRL1_STOP_BIT);
	reg_uart_ctrl1  |= StopBit;

}


/**
 * @brief      This function initializes the UART module.
 * @param[in]  Baudrate  	- uart baud rate
 * @param[in]  System_clock - clock of system
 * @param	   *RecvAddr:	receive buffer's address info.
 * @param	   RecvBufLen:	receive buffer's length, the maximum uart packet length should be smaller than (size - 4)
 * @return	   none
 * @return     none
 */
void uart_init_baudrate(unsigned int baudrate,unsigned int System_clock, unsigned char* RecvAddr, unsigned short RecvBufLen)
{
#if  !BLE_GET_LIST_BWPC
	GetBetterBwpc(baudrate,System_clock); //get the best bwpc and uart_div
	reg_uart_ctrl0 = g_bwpc; //set bwpc
	reg_uart_clk_div = (g_uart_div | FLD_UART_CLK_DIV_EN); //set div_clock
	reg_uart_rx_timeout0 = (g_bwpc+1) * 12; //one byte includes 12 bits at most
	reg_uart_rx_timeout1  = UART_BW_MUL2; //if over 2*(tmp_bwpc+1),one transaction end.
#else
	unsigned int bw, div;
	uartGetConfig(baudrate, &bw, &div,System_clock);
	reg_uart_clk_div = FLD_UART_CLK_DIV_EN | (div - 1);
	reg_uart_ctrl0 = (bw - 1) | FLD_UART_RX_DMA_EN | FLD_UART_TX_DMA_EN;
	reg_uart_rx_timeout = (bw * 12) | BIT(8);	//  12 bits timeout
#endif
	reg_dma0_addr = (unsigned int)(RecvAddr) & 0xffff;//set receive buffer address
	BM_CLR(reg_dma0_ctrl, FLD_DMA_BUF_SIZE);
	reg_dma0_ctrl |= MASK_VAL(FLD_DMA_BUF_SIZE, RecvBufLen>>4);  //set receive buffer size
}

/*
* @brief      This function initializes the UART Parity.
* @param[in]  Parity      	 selected parity type for UART interface
* @return     none
*/
void uart_Parity(UART_ParityTypeDef Parity)
{
	//parity config
	if (Parity) {
		reg_uart_ctrl1  |= FLD_UART_CTRL1_PARITY_EN; //enable parity function
		if (PARITY_EVEN == Parity) {
			reg_uart_ctrl1  &= (~FLD_UART_CTRL1_PARITY_POLARITY); //enable even parity
		}
		else if (PARITY_ODD == Parity) {
			reg_uart_ctrl1  |= FLD_UART_CTRL1_PARITY_POLARITY; //enable odd parity
		}
	}
	else {
		reg_uart_ctrl1  &= (~FLD_UART_CTRL1_PARITY_EN); //disable parity function
	}
}

/**
 * @brief     enable uart DMA mode
 * @param[in] none
 * @return    none
 */
void uart_dma_en(unsigned char rx_dma_en, unsigned char tx_dma_en)
{

	//enable DMA function of tx and rx
	if(rx_dma_en){
		reg_uart_ctrl0 |= FLD_UART_RX_DMA_EN ;
	}else{
		reg_uart_ctrl0 &= (~FLD_UART_RX_DMA_EN );
	}

	if(tx_dma_en){
		reg_uart_ctrl0  |= FLD_UART_TX_DMA_EN;
	}else{
		reg_uart_ctrl0	&= (~FLD_UART_TX_DMA_EN);
	}
}


/**
 * @brief     config the irq of uart tx and rx
 * @param[in] rx_irq_en - 1:enable rx irq. 0:disable rx irq
 * @param[in] tx_irq_en - 1:enable tx irq. 0:disable tx irq
 * @return    none
 */
void uart_irq_en(unsigned char rx_irq_en, unsigned char tx_irq_en)
{
	if(rx_irq_en){
		reg_uart_ctrl0 |= FLD_UART_RX_IRQ_EN ;
	}else{
		reg_uart_ctrl0 &= (~FLD_UART_RX_IRQ_EN );
	}

	if(tx_irq_en){
		reg_uart_ctrl0  |= FLD_UART_TX_IRQ_EN;
	}else{
		reg_uart_ctrl0	&= (~FLD_UART_TX_IRQ_EN);
	}

	if(tx_irq_en||rx_irq_en)
	{
		reg_irq_mask |= FLD_IRQ_UART_EN;
	}
	else
	{
		reg_irq_mask &= ~FLD_IRQ_UART_EN;
	}
}


/**
 *	@brief	uart send data function, this  function tell the DMA to get data from the RAM and start
 *			the DMA send function
 *	@param	sendBuff - send data buffer
 *	@return	'1' send success; '0' DMA busy
 */
unsigned char uart_dma_send(unsigned short* Addr){
    unsigned long len = *((unsigned long *)Addr);

    if(len > 252){
        return 0;
    }

    if (uart_tx_is_busy ())
    {
    	return 0;
    }

    //uart_set_tx_busy_flag();

    reg_dma1_addr = (unsigned short)(unsigned int)Addr;  //packet data, start address is sendBuff+1

    reg_dma_tx_rdy0 |= BIT(1);

	return 1;
}


/**
 * @brief     uart send data function, this  function tell the DMA to get data from the RAM and start
 *            the DMA transmission
 * @param[in] byte - single byte data need to send
 * @return    1: send success ;
 *            0: DMA busy
 */
volatile unsigned char uart_dma_send_byte(unsigned char byte)
{
	unsigned int addr;

	unsigned char b[5] = {1, 0,0,0,0};

	addr = (unsigned int)b;

	b[4] = byte;
	if (reg_uart_status1 & FLD_UART_TX_DONE ) {
		reg_dma1_addr = addr; //packet data, start address is sendBuff+1
		reg_dma_tx_rdy0	 = BIT(1);
		return 1;
	}

	   return 0;
}


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


void uart_ndma_set_triglevel(unsigned char rx_level, unsigned char tx_level)
{
	//1.set the trig level.
	BM_CLR(reg_uart_ctrl2, FLD_UART_CTRL3_RX_IRQ_TRIG_LEVEL);
	reg_uart_ctrl2 |= MASK_VAL(FLD_UART_CTRL3_RX_IRQ_TRIG_LEVEL, rx_level);

	BM_CLR(reg_uart_ctrl2, FLD_UART_CTRL3_TX_IRQ_TRIG_LEVEL);
	reg_uart_ctrl2 |= MASK_VAL(FLD_UART_CTRL3_TX_IRQ_TRIG_LEVEL, tx_level);
}


/**
 * @Brief: Only use for Normal mode of UART.(GaoQiu add)
 * @Param:
 * @Return:
 */
unsigned char uart_ndma_get_irq(void)
{
	return  (reg_uart_status0&FLD_UART_IRQ_FLAG );
}


/**
 * @brief     uart send data function with not DMA method.
 *            variable uart_TxIndex,it must loop the four registers 0x90 0x91 0x92 0x93 for the design of SOC.
 *            so we need variable to remember the index.
 * @param[in] uartData - the data to be send.
 * @return    1: send success ; 0: uart busy
 */
unsigned char tx_id = 0;
void uart_ndma_send_byte(unsigned char uartData)
{


	int t;
		static unsigned char uart_TxIndex = 0;

		t = 0;
		while( uart_tx_is_busy() && (t<0xfffff))
		{
			t++;
		}
		if(t >= 0xfffff)
			return;

		reg_uart_data_buf(uart_TxIndex) = uartData;

		uart_TxIndex++;
		uart_TxIndex &= 0x03;// cycle the four register 0x90 0x91 0x92 0x93.





}

/**
 * @Brief:  UART CTS initialization.
 * @Param:
 * @Retval: None.
 */
void uart_set_cts(unsigned char ctsEnable,unsigned char pinValue,UART_CtsPinDef pin )
{
	if(pinValue)
	{
		reg_uart_ctrl1  |= FLD_UART_CTRL1_CTS_SELECT;
	}
	else
	{
		reg_uart_ctrl1 &= ~FLD_UART_CTRL1_CTS_SELECT;;
	}

	if(ctsEnable)
	{

//		gpio_set_func(pin,AS_UART_CTS);
		unsigned short reg = 0x5ac;
		unsigned char bit = pin & 0xff;
		BM_CLR(reg_gpio_gpio_func(pin), bit);
		WRITE_REG8((reg),(READ_REG8(reg)&0xcf)|0x30);//C2

		gpio_set_input_en(pin, 1);
		reg_uart_ctrl1 |= FLD_UART_CTRL1_CTS_EN;
	}
	else
	{
		reg_uart_ctrl1 &= ~FLD_UART_CTRL1_CTS_EN;
	}
}


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
void uart_set_rts(unsigned char Enable, UART_RTSModeTypeDef Mode, unsigned char Thresh, unsigned char Invert, UART_RtsPinDef pin)
{
    if (Enable)
    {
    	unsigned short reg = 0x5ac;
    	unsigned char bit = pin & 0xff;
//    	gpio_set_func(pin,AS_UART_RTS);
    	BM_CLR(reg_gpio_gpio_func(pin), bit);
    	if(pin == UART_RTS_PC3)
    	WRITE_REG8((reg),(READ_REG8(reg)&0x3f)|0xc0);//C3

    	else if(pin == UART_RTS_PD1)
    	WRITE_REG8((reg+2),(READ_REG8(reg+2)&0xf3)|0x0c);//D1

    	gpio_set_input_en(pin, 1);//enable input
    	gpio_set_output_en(pin, 1);//enable output

        reg_uart_ctrl2 |= FLD_UART_CTRL2_RTS_EN; //enable RTS function
    }
    else
    {
        reg_uart_ctrl2 &= (~FLD_UART_CTRL2_RTS_EN); //disable RTS function
    }

    if (Mode)
    {
    	reg_uart_ctrl2 |= FLD_UART_CTRL2_RTS_MANUAL_EN;
    }
    else {
    	reg_uart_ctrl2 &= (~FLD_UART_CTRL2_RTS_MANUAL_EN);
    }

    if (Invert) {
    	reg_uart_ctrl2 |= FLD_UART_CTRL2_RTS_PARITY;
    }
    else {
    	reg_uart_ctrl2 &= (~FLD_UART_CTRL2_RTS_PARITY);
    }

    //set threshold
    reg_uart_ctrl2 &= (~FLD_UART_CTRL2_RTS_TRIG_LVL);
    reg_uart_ctrl2 |= (Thresh & FLD_UART_CTRL2_RTS_TRIG_LVL);
}

/**
 * @brief     This function determines whether parity error occurs once a packet arrives.
 * @param[in] none
 * @return    1: parity error ;
 *            0: no parity error
 */
unsigned char uart_is_parity_error(void)
{
    return (reg_uart_status0 & FLD_UART_RX_ERR_FLAG);
}

/**
 * @brief     This function clears parity error status once when it occurs.
 * @param[in] none
 * @return    none
 */
void uart_clear_parity_error(void)
{
	reg_uart_status0|= FLD_UART_RX_ERR_CLR; //write 1 to clear
}


/**
 * @Brief: Check UART busy flag.
 * @Param: None.
 * @ReVal: None.
 */
unsigned char uart_tx_is_busy(){
    return (!((reg_uart_status1 & FLD_UART_TX_DONE) ? 1:0));
}
/*******************************************************************************************************
******************************************end file*****************************************************/
