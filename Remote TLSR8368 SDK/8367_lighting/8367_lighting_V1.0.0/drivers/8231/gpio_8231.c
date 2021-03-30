
#include "gpio.h"
#include "analog.h"
/**
 * @brief      This function servers to initialization all GPIO.
 * @param[in]  none.
 * @return     none.
 */
void gpio_init(void){

	/* GPIOA Init ------------------------------------------------------------*/
	//PA IE settings
	analog_write(areg_gpio_pa_ie, (PA0_INPUT_ENABLE<<0)| (PA1_INPUT_ENABLE<<1)|(PA2_INPUT_ENABLE<<2)|(PA3_INPUT_ENABLE<<3)|
				(PA4_INPUT_ENABLE<<4)|(PA5_INPUT_ENABLE<<5)|(PA6_INPUT_ENABLE<<6)|(PA7_INPUT_ENABLE<<7));
	//PA OEN settings
	reg_gpio_pa_oen = (PA0_OUTPUT_ENABLE?0:BIT(0))|(PA1_OUTPUT_ENABLE?0:BIT(1))|(PA2_OUTPUT_ENABLE?0:BIT(2))|(PA3_OUTPUT_ENABLE?0:BIT(3))|
					  (PA4_OUTPUT_ENABLE?0:BIT(4))|(PA5_OUTPUT_ENABLE?0:BIT(5))|(PA6_OUTPUT_ENABLE?0:BIT(6))|(PA7_OUTPUT_ENABLE?0:BIT(7));
    //PA Output settings
	reg_gpio_pa_out = (PA0_DATA_OUT<<0)|(PA1_DATA_OUT<<1)|(PA2_DATA_OUT<<2)|(PA3_DATA_OUT<<3)|
			          (PA4_DATA_OUT<<4)|(PA5_DATA_OUT<<5)|(PA6_DATA_OUT<<6)|(PA7_DATA_OUT<<7);
	//PA DS settings
	analog_write(reg_gpio_pa_ds, (PA0_DATA_STRENGTH<<0)|(PA1_DATA_STRENGTH<<1)|(PA2_DATA_STRENGTH<<2)|(PA3_DATA_STRENGTH<<3)|
				(PA4_DATA_STRENGTH<<4)|(PA5_DATA_STRENGTH<<5)|(PA6_DATA_STRENGTH<<6)|(PA7_DATA_STRENGTH<<7));
	//PA GPIO function
    reg_gpio_pa_gpio = (PA0_FUNC == AS_GPIO?BIT(0):0)|(PA1_FUNC == AS_GPIO?BIT(1):0)|(PA2_FUNC == AS_GPIO?BIT(2):0)|(PA3_FUNC == AS_GPIO?BIT(3):0)|
    				   (PA4_FUNC == AS_GPIO?BIT(4):0)|(PA5_FUNC == AS_GPIO?BIT(5):0)|(PA6_FUNC == AS_GPIO?BIT(6):0)|(PA7_FUNC == AS_GPIO?BIT(7):0);

    /* GPIOB Init ------------------------------------------------------------*/
    //PB IE settings
    analog_write(areg_gpio_pb_ie,(PB0_INPUT_ENABLE<<0)|(PB1_INPUT_ENABLE<<1)|(PB2_INPUT_ENABLE<<2)|(PB3_INPUT_ENABLE<<3)|
    									 (PB4_INPUT_ENABLE<<4)|(PB5_INPUT_ENABLE<<5)|(PB6_INPUT_ENABLE<<6)|(PB7_INPUT_ENABLE<<7));
    //PB OEN settings
    reg_gpio_pb_oen = (PB0_OUTPUT_ENABLE?0:BIT(0))|(PB1_OUTPUT_ENABLE?0:BIT(1))|(PB2_OUTPUT_ENABLE?0:BIT(2))|(PB3_OUTPUT_ENABLE?0:BIT(3))|
    				  (PB4_OUTPUT_ENABLE?0:BIT(4))|(PB5_OUTPUT_ENABLE?0:BIT(5))|(PB6_OUTPUT_ENABLE?0:BIT(6))|(PB7_OUTPUT_ENABLE?0:BIT(7));
    //PB Output settings
    reg_gpio_pb_out = (PB0_DATA_OUT<<0)|(PB1_DATA_OUT<<1)|(PB2_DATA_OUT<<2)|(PB3_DATA_OUT<<3)|
    		          (PB4_DATA_OUT<<4)|(PB5_DATA_OUT<<5)|(PB6_DATA_OUT<<6)|(PB7_DATA_OUT<<7);
    //PB DS settings
    analog_write(areg_gpio_pb_ds,(PB0_DATA_STRENGTH<<0)|(PB1_DATA_STRENGTH<<1)|(PB2_DATA_STRENGTH<<2)|(PB3_DATA_STRENGTH<<3)|
    									 (PB4_DATA_STRENGTH<<4)|(PB5_DATA_STRENGTH<<5)|(PB6_DATA_STRENGTH<<6)|(PB7_DATA_STRENGTH<<7));
    //PB GPIO function
    reg_gpio_pb_gpio = (PB0_FUNC == AS_GPIO?BIT(0):0)|(PB1_FUNC == AS_GPIO?BIT(1):0)|(PB2_FUNC == AS_GPIO?BIT(2):0)|(PB3_FUNC == AS_GPIO?BIT(3):0)|
    				   (PB4_FUNC == AS_GPIO?BIT(4):0)|(PB5_FUNC == AS_GPIO?BIT(5):0)|(PB6_FUNC == AS_GPIO?BIT(6):0)|(PB7_FUNC == AS_GPIO?BIT(7):0);

    /* PC Init ---------------------------------------------------------------*/
	reg_gpio_pc_setting1 =
		(PC0_INPUT_ENABLE<<8) | (PC1_INPUT_ENABLE<<9) |(PC2_INPUT_ENABLE<<10)|(PC3_INPUT_ENABLE<<11) |
		(PC4_INPUT_ENABLE<<12)| (PC5_INPUT_ENABLE<<13)|(PC6_INPUT_ENABLE<<14)|(PC7_INPUT_ENABLE<<15) |
		((PC0_OUTPUT_ENABLE?0:1)<<16)|((PC1_OUTPUT_ENABLE?0:1)<<17)|((PC2_OUTPUT_ENABLE?0:1)<<18)|((PC3_OUTPUT_ENABLE?0:1)<<19) |
		((PC4_OUTPUT_ENABLE?0:1)<<20)|((PC5_OUTPUT_ENABLE?0:1)<<21)|((PC6_OUTPUT_ENABLE?0:1)<<22)|((PC7_OUTPUT_ENABLE?0:1)<<23) |
		(PC0_DATA_OUT<<24)|(PC1_DATA_OUT<<25)|(PC2_DATA_OUT<<26)|(PC3_DATA_OUT<<27)|
		(PC4_DATA_OUT<<28)|(PC5_DATA_OUT<<29)|(PC6_DATA_OUT<<30)|(PC7_DATA_OUT<<31);
	reg_gpio_pc_setting2 =
		(PC0_DATA_STRENGTH<<8) |(PC1_DATA_STRENGTH<<9) |(PC2_DATA_STRENGTH<<10)|(PC3_DATA_STRENGTH<<11)|
		(PC4_DATA_STRENGTH<<12)|(PC5_DATA_STRENGTH<<13)|(PC6_DATA_STRENGTH<<14)|(PC7_DATA_STRENGTH<<15)|
		(PC0_FUNC==AS_GPIO ? BIT(16):0)|(PC1_FUNC==AS_GPIO ? BIT(17):0)|(PC2_FUNC==AS_GPIO ? BIT(18):0)|(PC3_FUNC==AS_GPIO ? BIT(19):0) |
		(PC4_FUNC==AS_GPIO ? BIT(20):0)|(PC5_FUNC==AS_GPIO ? BIT(21):0)|(PC6_FUNC==AS_GPIO ? BIT(22):0)|(PC7_FUNC==AS_GPIO ? BIT(23):0);

	  /* PD Init ---------------------------------------------------------------*/
	reg_gpio_pd_setting1 =
		(PD0_INPUT_ENABLE<<8) | (PD1_INPUT_ENABLE<<9) |(PD2_INPUT_ENABLE<<10)|(PD3_INPUT_ENABLE<<11) |
		((PD0_OUTPUT_ENABLE?0:1)<<16)|((PD1_OUTPUT_ENABLE?0:1)<<17)|((PD2_OUTPUT_ENABLE?0:1)<<18)|((PD3_OUTPUT_ENABLE?0:1)<<19);

	reg_gpio_pd_setting2 =
		(PD0_DATA_STRENGTH<<8) |(PD1_DATA_STRENGTH<<9) |(PD2_DATA_STRENGTH<<10)|(PD3_DATA_STRENGTH<<11)|
		(PD0_FUNC==AS_GPIO ? BIT(16):0)|(PD1_FUNC==AS_GPIO ? BIT(17):0)|(PD2_FUNC==AS_GPIO ? BIT(18):0)|(PD3_FUNC==AS_GPIO ? BIT(19):0);

/************************************      for digital pull_up      **********************************/
	//PA<7:0> Just has one bit to enable or disable pull_up.
	analog_write(0xb7,  PULL_UP_WAKEUP_SRC_PA0|
						(PULL_UP_WAKEUP_SRC_PA1<<1)|
						(PULL_UP_WAKEUP_SRC_PA2<<2)|
						(PULL_UP_WAKEUP_SRC_PA3<<3)|
						(PULL_UP_WAKEUP_SRC_PA4<<4)|
						(PULL_UP_WAKEUP_SRC_PA5<<5)|
						(PULL_UP_WAKEUP_SRC_PA6<<6)|
						(PULL_UP_WAKEUP_SRC_PA7<<7));
	//PB<7:0> Just has one bit to enable or disable pull_up.
	analog_write(0xba,  PULL_UP_WAKEUP_SRC_PB0|
						(PULL_UP_WAKEUP_SRC_PB1<<1)|
						(PULL_UP_WAKEUP_SRC_PB2<<2)|
						(PULL_UP_WAKEUP_SRC_PB3<<3)|
						(PULL_UP_WAKEUP_SRC_PB4<<4)|
						(PULL_UP_WAKEUP_SRC_PB5<<5)|
						(PULL_UP_WAKEUP_SRC_PB6<<6)|
						(PULL_UP_WAKEUP_SRC_PB7<<7));

/*******************************      for analog pull_up/pull_down      ******************************/
	//PA<5,7>,PB<0,7> Has two bits to enable or disable pull_up/pull_down.PA0,PA1 Just has one bit to enable or disable pull_down.
	analog_write(0x0a,  PULL_WAKEUP_SRC_PA5|
						(PULL_WAKEUP_SRC_PA6<<2)|
						(PULL_WAKEUP_SRC_PA7<<4)|
						(PULL_DOWN_WAKEUP_SRC_PA0<<6)|
						(PULL_DOWN_WAKEUP_SRC_PA1<<7));

	analog_write(0x08,  PULL_WAKEUP_SRC_PB0|
						(PULL_WAKEUP_SRC_PB1<<2)|
						(PULL_WAKEUP_SRC_PB2<<4)|
						(PULL_WAKEUP_SRC_PB3<<6));

	analog_write(0x09,  PULL_WAKEUP_SRC_PB4|
						(PULL_WAKEUP_SRC_PB5<<2)|
						(PULL_WAKEUP_SRC_PB6<<4)|
						(PULL_WAKEUP_SRC_PB7<<6));

	//PC<7:0>,PD<3:0> Just has one bit to pull_down here.PA<4:2> Just has one bit to enable or disable pull_down.
	analog_write(0x0b,  PULL_DOWN_WAKEUP_SRC_PA2|
						(PULL_DOWN_WAKEUP_SRC_PA3<<1)|
						(PULL_DOWN_WAKEUP_SRC_PA4<<2)|
						(PULL_WAKEUP_SRC_PC0<<3)|
						(PULL_WAKEUP_SRC_PC1<<4)|
						(PULL_WAKEUP_SRC_PC2<<5)|
						(PULL_WAKEUP_SRC_PC3<<6)|
						(PULL_WAKEUP_SRC_PC4<<7));

	analog_write(0x0c,  PULL_WAKEUP_SRC_PC5|
						(PULL_WAKEUP_SRC_PC6<<1)|
						(PULL_WAKEUP_SRC_PC7<<2)|
						(PULL_WAKEUP_SRC_PD0<<3)|
						(PULL_WAKEUP_SRC_PD1<<4)|
						(PULL_WAKEUP_SRC_PD2<<5)|
						(PULL_WAKEUP_SRC_PD3<<6));

}

/**
 * @brief      This function servers to set the GPIO's function.
 * @param[in]  pin - the special pin.
 * @param[in]  func - the function of GPIO.
 * @return     none.
 */
void gpio_set_func(GPIO_PinTypeDef pin, GPIO_FuncTypeDef func)
{
	unsigned char bit = pin & 0xff;
	if(func == AS_GPIO)
	{
		BM_SET(reg_gpio_gpio_func(pin), bit);
		return;
	}
	else
	{
		BM_CLR(reg_gpio_gpio_func(pin), bit);
	}
}

/**
 * @brief      This function set the input function of a pin.
 * @param[in]  pin - the pin needs to set the input function
 * @param[in]  value - enable or disable the pin's input function(0: disable, 1: enable)
 * @return     none
 */
void gpio_set_input_en(GPIO_PinTypeDef pin, unsigned int value)
{
	unsigned short gpioGroup = pin & 0x0f00;
	unsigned char bit = pin & 0xff;
	unsigned char temp = 0;

	if(gpioGroup == GPIO_GROUPA)
	{
		if(value)
		{
			temp = analog_read(areg_gpio_pa_ie);
			temp |= bit;
			analog_write(areg_gpio_pa_ie, temp);
		}
		else
		{
			temp = analog_read(areg_gpio_pa_ie);
			temp &= ~bit;
			analog_write(areg_gpio_pa_ie, temp);
		}
	}
	else if(gpioGroup == GPIO_GROUPB)
	{
		if(value)
		{
			temp = analog_read(areg_gpio_pb_ie);
			temp |= bit;
			analog_write(areg_gpio_pb_ie, temp);
		}
		else
		{
			temp = analog_read(areg_gpio_pb_ie);
			temp &= ~bit;
			analog_write(areg_gpio_pb_ie, temp);
		}
	}
	else
	{
		if(value)
		{
			BM_SET(reg_gpio_ie(pin), bit);
		}
		else
		{
			BM_CLR(reg_gpio_ie(pin), bit);
		}
	}
}

/**
 * @brief     This function to judge whether a pin's input is enable.
 * @param[in] pin - the pin needs to enable its input.
 * @return    1:enable the pin's input function.
 *            0:disable the pin's input function.
 */
int gpio_is_input_en(GPIO_PinTypeDef pin)
{
	unsigned short gpioGroup = pin & 0x0f00;
	unsigned char bit = pin & 0xff;

	if(gpioGroup == GPIO_GROUPA)
	{
		unsigned char temp = analog_read(areg_gpio_pa_ie);
		return BM_IS_SET(temp, bit);

	}
	else if(gpioGroup == GPIO_GROUPB)
	{
		unsigned char temp = analog_read(areg_gpio_pb_ie);
		return BM_IS_SET(temp, bit);
	}

	return BM_IS_SET(reg_gpio_ie(pin), bit);
}

/**
 * @brief      This function set the pin's driving strength.
 * @param[in]  pin - the pin needs to set the driving strength
 * @param[in]  value - the level of driving strength(1: strong 0: poor)
 * @return     none
 */
void gpio_set_data_strength(GPIO_PinTypeDef pin, unsigned int value)
{
	unsigned short gpioGroup = pin & 0x0f00;
	unsigned char bit = pin & 0xff;
	unsigned char temp = 0;

	if(gpioGroup == GPIO_GROUPA)
	{
		if(value)
		{
			temp = analog_read(reg_gpio_pa_ds);
			temp |= bit;
			analog_write(reg_gpio_pa_ds, temp);
		}
		else
		{
			temp = analog_read(reg_gpio_pa_ds);
			temp &= ~bit;
			analog_write(reg_gpio_pa_ds, temp);
		}
	}
	else if(gpioGroup == GPIO_GROUPB)
	{
		if(value)
		{
			temp = analog_read(areg_gpio_pb_ds);
			temp |= bit;
			analog_write(areg_gpio_pb_ds, temp);
		}
		else
		{
			temp = analog_read(areg_gpio_pb_ds);
			temp &= ~bit;
			analog_write(areg_gpio_pb_ds, temp);
		}
	}
	else
	{
		if(value)
		{
			BM_SET(reg_gpio_ds(pin), bit);
		}
		else
		{
			BM_CLR(reg_gpio_ds(pin), bit);
		}
	}
}

/**
 * @brief   Each hexadecimal value in the array corresponds to a GPIO.
 *          The upper four bits of the value indicate the register address that controls the GPIO.
 *          The lower one or lower two bits are used to control the pull-up/pull_down of the GPIO.
 *          Just as the table means:
 *-----------------------------------------------
 *      BIT(7.6)   BIT(5.4)   BIT(3.2)   BIT(1.0)
 *-----------------------------------------------
 *offset|  6  |    |  4  |    |  2  |    |  0  |
 *-----------------------------------------------
 * 08     B3         B2          B1         B0
 * 09     B7         B6          B5         B4
 * 0a   (A1,A0)      A7          A6         A5
 * 0b   (C4,C3)    (C2,C1)     (C0,A4)    (A3,A2)
 * 0c	  D3       (D2,D1)     (D0,C7)    (C6,C5)
 *-----------------------------------------------
 */
const unsigned char gpioPullResistorMapTab[4][8]=
{
	{0xa6, 0xa7, 0xb0, 0xb1, 0xb2, 0xa0, 0xa2, 0xa4 }, //GPIO_GROUPA:For PA0~PA7
	{0x80, 0x82, 0x84, 0x86, 0x90, 0x92, 0x94, 0x96 }, //GPIO_GROUPB:For PB0~PB7
	{0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xc0, 0xc1, 0xc2 }, //GPIO_GROUPC:For PC0~PC7
	{0xc3, 0xc4, 0xc5, 0xc6}                           //GPIO_GROUPD:For PD0~PD3
};

/**
 * @brief     This function set a gpio_pad pull-up/down resistor(NOT ALL THE PINS).
 * @param[in] gpio - the pin needs to set its pull-up/down resistor.Just for (PA<7:5>,PB<7:0>).
 * @param[in] up_down - the type of the pull-up/down resistor
 * @return    none
 */
void gpio_set_up_down_resistor(GPIO_PinTypeDef gpio, unsigned short up_down)
{
	unsigned char bit = gpio & 0xff;
	unsigned char pinId = 0;
	unsigned short gpioGroup = gpio & 0xf00;
	unsigned char gpioGroupId = gpio >> 8;
	unsigned char pullResistorAddr = 0;
	unsigned char offset = 0;
	unsigned char temp = 0;

	for(volatile int i = 0; i<8; i++)
	{
		if((bit>>i) & 0x01)
		{
			pinId = i;
			break;
		}
	}
	if(pinId >= 8)//parameter error.
		return;

	temp = gpioPullResistorMapTab[gpioGroupId][pinId];
	pullResistorAddr = (temp>>4) & 0x0f;
	offset = temp & 0x0f;

	temp = analog_read(pullResistorAddr);

	if(gpio == GPIO_PA5 || gpio == GPIO_PA6 || gpio == GPIO_PA7 || gpioGroup == GPIO_GROUPB)
	{
		temp &= ~(0x03<<offset);
		temp |= (up_down << offset);
		analog_write(pullResistorAddr,temp);
	}
	else
	{
		if(up_down == GPIO_PULL_NONE ||up_down == GPIO_PULL_DN_100K )
		{
			temp &= ~(0x01 << offset);
			temp |= ((up_down >> 1) << offset);
			analog_write(pullResistorAddr,temp);
		}
	}

}

/**
 * @brief     This function set a gpio_core PULL_UP resistor.this function include all GPIOs.
 * @param[in] gpio - the pin needs to set its pull-up resistor.
 * @return    none
 */
void gpio_set_up_30k(GPIO_PinTypeDef pin)
{
	unsigned short gpioGroup = pin & 0xf00;
	unsigned char temp = 0;
	unsigned char bit = pin & 0xff;
	//For pull_up of PA0~PA7.
	if(gpioGroup == GPIO_GROUPA)
	{
		temp = analog_read(0xb7);
		analog_write(0xb7,temp|bit);
	}
	//For pull_up of PB0~PB7.
	else if(gpioGroup == GPIO_GROUPB)
	{
		temp = analog_read(0xba);
		analog_write(0xba,temp|bit);
	}
	//For pull_up of PC0~PC7,PD0~PD7.
	else if(gpioGroup == GPIO_GROUPC || gpioGroup == GPIO_GROUPD)
	{
	   unsigned char bit = pin & 0xff;
	   BM_SET(reg_gpio_oen(pin), bit);
	   BM_SET(reg_gpio_out(pin), bit);
	}
}

/**
 * @brief      This function servers to set the specified GPIO as high resistor.
 * @param[in]  pin  - select the specified GPIO
 * @return     none.
 */
void gpio_shutdown(GPIO_PinTypeDef pin)
{
	unsigned short group = pin & 0xf00;
	unsigned char bit = pin & 0xff;
	switch(group)
	{
		case GPIO_GROUPA:
			reg_gpio_pa_oen |= bit;
			reg_gpio_pa_out &= (!bit);
			if(bit&0x1f)
			{
				reg_gpio_ie(GPIO_GROUPA) &= (!bit);
			}
			else
			{
				analog_write(areg_gpio_pa_ie, analog_read(areg_gpio_pa_ie) & (!bit));
			}
			break;
		case GPIO_GROUPB:
			reg_gpio_pb_oen |= bit;
			reg_gpio_pb_out &= (!bit);
			analog_write(areg_gpio_pb_ie, analog_read(areg_gpio_pb_ie) & (!bit));
			break;
		case GPIO_GROUPC:
			reg_gpio_pc_oen |= bit;
			reg_gpio_pc_out &= (!bit);
			reg_gpio_pc_ie &= (!bit);
			break;
		case GPIO_ALL:
		{
			//output disable
			reg_gpio_pa_oen = 0xff;
			reg_gpio_pb_oen = 0xff;
			reg_gpio_pc_oen = 0xff;

			//dataO = 0
			reg_gpio_pa_out = 0x00;
			reg_gpio_pb_out = 0x00;
			reg_gpio_pc_out = 0x00;

			//ie = 0
			reg_gpio_ie(GPIO_GROUPA) = 0x00;
			analog_write(areg_gpio_pa_ie, 0x00);
			analog_write(areg_gpio_pb_ie, 0x00);
			reg_gpio_pc_ie = 0x80;
		}
	}
}

/**
 * @brief      This function servers to clear the GPIO's IRQ function.
 * @param[in]  none.
 * @return     none.
 */
void gpio_clear_gpio_irq_flag(void)
{
	REG_ADDR8(0x64A) |= BIT(2);
}




