/********************************************************************************************************
 * @file     pwm.h
 *
 * @brief    This is the header file for TLSR8231
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
 * @par      History:
 * 			 1.initial release(May. 24 2019)
 *
 * @version  A001
 *
*******************************************************************************************************/
#ifndef _PWM_H_
#define _PWM_H_
#include "register.h"
#include "clock.h"
#include "timer.h"
/**
 * @brief  enum variable, the number of PWM channels supported
 */
typedef enum {
	PWM0 = 0,
	PWM1,
	PWM2,
	PWM3,
	PWM4,
}PWM_TypeDef;
/**
 * @brief  enum variable used for PWM work mode setting
 */
typedef enum{
	PWM_NORMAL_MODE   = 0x00,
	PWM_COUNT_MODE    = 0x01,
	PWM_IR_MODE       = 0x03,
	PWM_IR_FIFO_MODE  = 0x07,
	PWM_IR_DMA_FIFO_MODE  = 0x0F,
}PWM_ModeDef;

/**
 * @brief  pwm interrupt source
 */
typedef enum{
	PWM0_PULSE_NORMAL =		0,       // duty cycle and period from TCMP0/TMAX0 					 0x794~0x797
	PWM0_PULSE_SHADOW =		BIT(14), // duty cycle and period from TCMP0_SHADOW / TMAX0_SHADOW   0x7c4~0x7c7
}PWM0_Pulse_TypeDef;

/**
 * @brief  pwm interrupt source
 */
typedef enum{
	PWM0_PNUM_IRQ =					BIT(0),
	PWM0_IR_DMA_FIFO_DONE_IRQ =		BIT(1),
	PWM0_FRAME_IRQ =				BIT(2),
	PWM1_FRAME_IRQ =				BIT(3),
	PWM2_FRAME_IRQ =				BIT(4),
	PWM0_IR_FIFO_CNT = 	(0x80|BIT(0)),	// write to reg_pwm_irq_mask1
}PWM_Irq_TypeDef;


/**
 * @brief  enum variable used for PA0~PA3 PWM GPIO function setting
 */
enum{
	PA0_PWM0_PWM3 = BIT_RNG(0, 1),//PWM3:0  PWM0:1
	PA1_PWM3_N = BIT_RNG(2, 3),//0
	PA2_PWM1_N = BIT_RNG(4, 5),//0
	PA3_PWM4 = BIT_RNG(6, 7),//0
};
#define    PWM0_CFG_GPIO_A0()	do{\
									gpio_set_func(GPIO_PA0,!AS_GPIO);\
									reg_goio_gpa1_setting &= (~PA0_PWM0_PWM3) ;\
									reg_goio_gpa1_setting |= BIT(0) ;\
								}while(0)
#define    PWM3_CFG_GPIO_A0()	do{\
									gpio_set_func(GPIO_PA0,!AS_GPIO);\
									reg_goio_gpa1_setting &= (~PA0_PWM0_PWM3) ;\
								}while(0)
#define    PWM3_N_CFG_GPIO_A1()	do{\
									gpio_set_func(GPIO_PA1,!AS_GPIO);\
									reg_goio_gpa1_setting &= (~PA1_PWM3_N) ;\
								}while(0)
#define    PWM1_N_CFG_GPIO_A2()	do{\
									gpio_set_func(GPIO_PA2,!AS_GPIO);\
									reg_goio_gpa1_setting &= (~PA2_PWM1_N) ;\
								}while(0)
#define    PWM4_CFG_GPIO_A3()	do{\
									gpio_set_func(GPIO_PA3,!AS_GPIO);\
									reg_goio_gpa1_setting &= (~PA3_PWM4) ;\
								}while(0)
/**
 * @brief  enum variable used for PA4~PA7 PWM GPIO function setting
*/
enum{
		PA4_PWM2     =    BIT_RNG(0, 1),//0
		PA5_PWM2_N  = BIT_RNG(2, 3),//0
	    PA6_PWM4_N  = BIT_RNG(4, 5),//0
		PA7_PWM0  = BIT_RNG(6, 7),//0
};
#define    PWM2_CFG_GPIO_A4()	do{\
									gpio_set_func(GPIO_PA4,!AS_GPIO);\
									reg_goio_gpa2_setting &= (~PA4_PWM2) ;\
								}while(0)
#define    PWM2_N_CFG_GPIO_A5()	do{\
									gpio_set_func(GPIO_PA5,!AS_GPIO);\
									reg_goio_gpa2_setting &= (~PA5_PWM2_N) ;\
								}while(0)
#define    PWM4_N_CFG_GPIO_A6()	do{\
									gpio_set_func(GPIO_PA6,!AS_GPIO);\
									reg_goio_gpa2_setting &= (~PA6_PWM4_N) ;\
								}while(0)
#define    PWM0_CFG_GPIO_A7()	do{\
									gpio_set_func(GPIO_PA7,!AS_GPIO);\
									reg_goio_gpa2_setting &= (~PA7_PWM0 ) ;\
								}while(0)

/**
 * @brief  enum variable used for PB0~PB3 PWM GPIO function setting
 */
enum{
		PB0_PWM4  = BIT_RNG(0, 1),//0
		PB1_PWM1  = BIT_RNG(2, 3),//0
	    PB2_PWM0  = BIT_RNG(4, 5),//0
		PB3_PWM2_PWM3_N  = BIT_RNG(6, 7),//PWM2:2  PWM3_N:0
	};
#define    PWM4_CFG_GPIO_B0()	do{\
									gpio_set_func(GPIO_PB0,!AS_GPIO);\
									reg_goio_gpb1_setting &= (~PB0_PWM4) ;\
								}while(0)
#define    PWM1_CFG_GPIO_B1()	do{\
									gpio_set_func(GPIO_PB1,!AS_GPIO);\
									reg_goio_gpb1_setting &= (~PB1_PWM1) ;\
								}while(0)
#define    PWM0_CFG_GPIO_B2()	do{\
									gpio_set_func(GPIO_PB2,!AS_GPIO);\
									reg_goio_gpb1_setting &= (~PB2_PWM0 ) ;\
								}while(0)
#define    PWM2_CFG_GPIO_B3()	do{\
									gpio_set_func(GPIO_PB3,!AS_GPIO);\
									reg_goio_gpb1_setting &= (~PB3_PWM2_PWM3_N) ;\
									reg_goio_gpb1_setting |= BIT(7) ;\
								}while(0)

#define    PWM3_N_CFG_GPIO_B3()	do{\
									gpio_set_func(GPIO_PB3,!AS_GPIO);\
									reg_goio_gpb1_setting &= (~PB3_PWM2_PWM3_N) ;\
								}while(0)
/**
 * @brief  enum variable used for PB4~PB7 PWM GPIO function setting
 */
enum{
		PB4_PWM2_N = BIT_RNG(0, 1),//0
		PB5_PWM4   = BIT_RNG(2, 3),//0
	    PB6_PWM0_N = BIT_RNG(4, 5),//0
		PB7_PWM1   = BIT_RNG(6, 7),//0
	};
#define    PWM2_N_CFG_GPIO_B4()	do{\
									gpio_set_func(GPIO_PB4,!AS_GPIO);\
									reg_goio_gpb2_setting &= (~PB4_PWM2_N) ;\
								}while(0)
#define    PWM4_CFG_GPIO_B5()	do{\
									gpio_set_func(GPIO_PB5,!AS_GPIO);\
									reg_goio_gpb2_setting &= (~PB5_PWM4) ;\
								}while(0)
#define    PWM0_N_CFG_GPIO_B6()	do{\
									gpio_set_func(GPIO_PB6,!AS_GPIO);\
									reg_goio_gpb2_setting &= (~ PB6_PWM0_N ) ;\
								}while(0)
#define    PWM1_CFG_GPIO_B7()	do{\
									gpio_set_func(GPIO_PB7,!AS_GPIO);\
									reg_goio_gpb2_setting &= (~PB7_PWM1) ;\
								}while(0)
/**
 * @brief  enum variable used for PC1 PC2 PC3 PC6 PWM GPIO function setting
 */
enum{
		PC1_PWM2_N   = BIT_RNG(2, 3),//0
		PC2_PWM1     = BIT_RNG(4, 5),//1
	    PC3_PWM0_N   = BIT_RNG(6, 7),//1
		PC6_PWM4     = BIT_RNG(4, 5),//0
	};
#define    PWM2_N_CFG_GPIO_PC1()	do{\
									gpio_set_func(GPIO_PC1,!AS_GPIO);\
									reg_goio_gpc1_setting &= (~PC1_PWM2_N ) ;\
								}while(0)
#define    PWM1_CFG_GPIO_C2()	do{\
									gpio_set_func(GPIO_PC2,!AS_GPIO);\
									reg_goio_gpc1_setting &= (~PC2_PWM1) ;\
									reg_goio_gpc1_setting |= BIT(4) ;\
								}while(0)
#define    PWM0_N_CFG_GPIO_C3()	do{\
									gpio_set_func(GPIO_PC3,!AS_GPIO);\
									reg_goio_gpc1_setting &= (~ PC3_PWM0_N);\
									reg_goio_gpc1_setting |= BIT(6) ;\
								}while(0)
#define    PWM4_CFG_GPIO_C6()	do{\
									gpio_set_func(GPIO_PC6,!AS_GPIO);\
									reg_goio_gpc2_setting &= (~PC6_PWM4);\
								}while(0)

/**
 * @brief  enum variable used for PD0~PD3  PWM GPIO function setting
 */
enum{
		PD0_PWM1_PWM0_N = BIT_RNG(0, 1),//PWM1£º0 PWM0_N:2
		PD1_PWM0_PWM1_N = BIT_RNG(2, 3),//PWM0£º0 PWM1_N:2
	    PD2_PWM2_PWM3   = BIT_RNG(4, 5),//PWM3£º0 PWM2:2
		PD3_PWM0        = BIT_RNG(6, 7),//0
	};
#define    PWM1_CFG_GPIO_PD0()	do{\
									gpio_set_func(GPIO_PD0,!AS_GPIO);\
									reg_goio_gpd1_setting &= (~PD0_PWM1_PWM0_N ) ;\
								}while(0)
#define    PWM0_N_CFG_GPIO_PD0()	do{\
									gpio_set_func(GPIO_PD0,!AS_GPIO);\
									reg_goio_gpd1_setting &= (~PD0_PWM1_PWM0_N) ;\
									reg_goio_gpd1_setting |= BIT(1) ;\
								}while(0)
#define    PWM0_CFG_GPIO_PD1()	do{\
									gpio_set_func(GPIO_PD1,!AS_GPIO);\
									reg_goio_gpd1_setting &= (~ PD1_PWM0_PWM1_N ) ;\
								}while(0)
#define    PWM1_N_CFG_GPIO_PD1()	do{\
									gpio_set_func(GPIO_PD1,!AS_GPIO);\
									reg_goio_gpd1_setting &= (~PD1_PWM0_PWM1_N) ;\
									reg_goio_gpd1_setting |= BIT(3) ;\
								}while(0)
#define    PWM2_CFG_GPIO_PD2()	do{\
									gpio_set_func(GPIO_PD2,!AS_GPIO);\
									reg_goio_gpd1_setting &= (~ PD2_PWM2_PWM3 ) ;\
									reg_goio_gpd1_setting |= BIT(5) ;\
								}while(0)
#define    PWM3_CFG_GPIO_PD2()	do{\
									gpio_set_func(GPIO_PD2,!AS_GPIO);\
									reg_goio_gpd1_setting &= (~PD2_PWM2_PWM3) ;\
								}while(0)
#define    PWM0_CFG_GPIO_PD3()	do{\
									gpio_set_func(GPIO_PD3,!AS_GPIO);\
									reg_goio_gpd1_setting &= (~PD3_PWM0) ;\
								}while(0)

#endif /* PWM_H_ */

/**
 * @brief     This fuction servers to set pwm mode.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @param[in] mode - variable of enum to indicates the pwm mode.
 * @return	  none.
 */
 static inline void pwm_set_mode(PWM_TypeDef id, PWM_ModeDef mode)
{
	if(PWM0== id){
		reg_pwm_mode = mode;  //only PWM0 has count/IR/fifo IR mode
	}
}




/**
 * @brief     This fuction servers to set pwm clock frequency
 * @param[in] system_clock_hz - variable to set system clock hz.
 * @param[in] pwm_clk - variable of the pwm clock.
 * @return	  none.
 */
static inline void pwm_set_clk(int system_clock_hz, int pwm_clk)
{
	reg_rst1 &= ~FLD_RST1_PWM;
	reg_clk_en1 |= FLD_CLK1_PWM_EN;
	reg_pwm_clk = (int)system_clock_hz /pwm_clk - 1;
}


/**
 * @brief     This fuction servers to set pwm count status(CMP) time.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @param[in] cmp_tick - variable of the CMP.
 * @return	  none.
 */
static inline void pwm_set_cmp(PWM_TypeDef id, unsigned short cmp)
{
	reg_pwm_cmp(id) = cmp;
}

/**
 * @brief     This fuction servers to set pwm cycle time.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @param[in] cycle_tick - variable of the cycle time.
 * @return	  none.
 */
static inline void pwm_set_max(PWM_TypeDef id, unsigned short cycle_tick)
{
	reg_pwm_max(id) = cycle_tick;
}

/**
 * @brief     This fuction servers to set pwm cycle time & count status.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @param[in] max_tick - variable of the cycle time.
 * @param[in] cmp_tick - variable of the CMP.
 * @return	  none.
 */
static inline void pwm_set_max_and_cmp(PWM_TypeDef id, unsigned short max_tick,
		                                  unsigned short cmp_tick)
{
	reg_pwm_cycle(id) = MASK_VAL(FLD_PWM_CMP, cmp_tick, FLD_PWM_MAX, max_tick);
}


/**
 * @brief     This fuction servers to set pwm cycle time & count status.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @param[in] max_tick - variable of the cycle time.
 * @param[in] cmp_tick - variable of the CMP.
 * @return	  none.
 */
static inline void pwm_set_shadow_max_and_cmp(PWM_TypeDef id, unsigned short max_tick,
		                                  	  	  unsigned short cmp_tick)
{
	if(id == PWM0){
		reg_pwm_cmp_shadow = cmp_tick;
		reg_pwm_max_shadow = max_tick;
	}
}


/**
 * @brief     This fuction servers to set the pwm pulse number.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @param[in] pulse_num - variable of the pwm pulse number.
 * @return	  none.
 */
static inline void pwm_set_pulse_num(PWM_TypeDef id, unsigned short pulse_num)
{
	if(id == PWM0)
	{
		reg_pwm_pulse_num = pulse_num;
	}
}


/**
 * @brief     This fuction servers to start the pwm.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @return	  none.
 */
static inline void pwm_start(PWM_TypeDef id)
{
	if(id == PWM0)
	{
		reg_pwm_enable |= FLD_PWM_EN_PWM0;
	}
	else
	{
		BM_SET(reg_pwm_enable, BIT(id));
	}

}


/**
 * @brief     This fuction servers to stop the pwm.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @return	  none.
 */
static inline void pwm_stop(PWM_TypeDef id)
{
	if(id == PWM0)
	{
		reg_pwm_enable &= ~FLD_PWM_EN_PWM0;
	}
	else
	{
		reg_pwm_enable &= ~BIT(id);
	}

}

/**
 * @brief     This fuction servers to revert the PWMx.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @return	  none.
 */
static inline void pwm_revert(PWM_TypeDef id)
{
	reg_pwm_invert |= BIT(id);
}


/**
 * @brief     This fuction servers to revert the PWMx_N.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @return	  none.
 */
static inline void pwm_n_revert(PWM_TypeDef id){
	reg_pwm_n_invert |= BIT(id);
}

/**
 * @brief     This fuction servers to enable the pwm polarity.
 * @param[in] PWM_TypeDef - variable of enum to select the pwm number.
 * @param[in] en: 1 enable. 0 disable.
 * @return	  none.
 */
static inline void pwm_set_pol(PWM_TypeDef id, int en){
	if(en == 0)
	{
		BM_SET(reg_pwm_pol, BIT(id));
	}
	else
	{
		BM_CLR(reg_pwm_pol, BIT(id));
	}
}

/**
 * @brief     This fuction servers to enable the pwm interrupt.
 * @param[in] PWM_Irq_TypeDef - variable of enum to select the pwm interrupt source.
 * @return	  none.
 */
static inline void pwm_set_irq_en(PWM_Irq_TypeDef irq, unsigned char en){
	 if (en)
	 {
	   if(irq == PWM0_IR_FIFO_CNT)
		   reg_pwm_irq_mask1 |= BIT(0);
	   else{
	    	 BM_SET(reg_pwm_irq_mask, irq);
	      }
	     reg_irq_mask|=FLD_IRQ_SW_PWM_EN ;
	 }

	 else
	 {
		   if(irq == PWM0_IR_FIFO_CNT)
			reg_pwm_irq_mask1 &= ~(BIT(0));
		   else{
			   BM_CLR(reg_pwm_irq_mask, irq);
		      }
		   reg_irq_mask &= ~FLD_IRQ_SW_PWM_EN ;
	 }
}

/**
 * *****************sdk*********************
 */
static inline unsigned char pwm_get_irq_status(unsigned char irq){
	if(reg_irq_src & FLD_IRQ_SW_PWM_EN){
		if(irq == PWM0_IR_FIFO_CNT){
			return (reg_pwm_irq_state1 & BIT(0));
		}
		return (reg_pwm_irq_state & irq);
	}
	return 0;
}

static inline void pwm_clear_irq_status(unsigned char  irq){
	if(irq == PWM0_IR_FIFO_CNT){
		reg_pwm_irq_state1 = BIT(0);
	}else{
		reg_pwm_irq_state	= irq;
	}
	reg_irq_src =FLD_IRQ_SW_PWM_EN;
}
/********************sdk*********************/

/**
 * @brief     This fuction serves to set trigger trig_level of interrupt for IR FiFo mode
 * @param[in] none
 * @return	  none
 */
static inline void pwm_ir_fifo_set_irq_trig_level(unsigned char trig_level)
{
	reg_pwm_ir_fifo_irq_trig_level = trig_level;
}


/**
 * @brief     This fuction serves to get the number of data in fifo.
 * @param[in] none
 * @return	  the number of data in fifo
 */
static inline unsigned char pwm_ir_fifo_get_data_num(void)
{
	return (reg_pwm_ir_fifo_data_status	 & 0x0f);
}


/**
 * @brief     This fuction serves to determine whether data in fifo is empty.
 * @param[in] none
 * @return	  yes: 1 ,no: 0;
 */
static inline unsigned char pwm_fifo_is_empty(void)
{
	return (reg_pwm_ir_fifo_data_status	 & FLD_PWM0_IR_FIFO_EMPTY) ? 1:0;
}


/**
 * @brief     This fuction serves to determine whether data in fifo is full.
 * @param[in] none
 * @return	  yes: 1 ,no: 0;
 */
static inline unsigned char pwm_ir_fifo_is_full(void)
{
	return (reg_pwm_ir_fifo_data_status&FLD_PWM0_IR_FIFO_FULL);
}


/**
 * @brief     This fuction serves to write data into FiFo
 * @param[in] pulse_num  - the number of pulse
 * @param[in] use_shadow - determine whether the configuration of shadow cmp and shadow max is used
 * 						   1: use shadow, 0: not use
 * @param[in] carrier_en - enable sending carrier, 1: enable, 0: disable
 * @return	  none
 */
static inline void pwm_ir_fifo_set_data_entry(unsigned short pulse_num, unsigned char use_shadow, unsigned char carrier_en)
{
	static unsigned char index=0;
	unsigned short cfg_data = pulse_num + ((use_shadow&BIT(0))<<14) + ((carrier_en&BIT(0))<<15);
	while(pwm_ir_fifo_is_full());
	reg_pwm_ir_fifo_dat(index) = cfg_data;
	index++;
	index&=0x01;
}


/**
 * @brief     This fuction serves to config the pwm's dma wave form.
 * @param[in] carrier_en - must 1 or 0.
 * @param[in] PWM0_Pulse_TypeDef - type of pwm0's pulse.
 * @param[in] pulse_num - the number of pulse.
 * @return	  none.
 */
static inline unsigned short pwm_ir_dma_fifo_set_waveform(int carrier_en, PWM0_Pulse_TypeDef pulse_type,  unsigned short pulse_num)
{
	return  ( carrier_en<<15 | pulse_type | (pulse_num & 0x3fff) );
}

/**
 * @brief     This fuction servers to set the pwm's dma address.
 * @param[in] pdat - variable of pointer to indicate the address.
 * @return	  none.
 */
static inline void pwm_set_dma_addr(void * pdat)
{
	reg_dma5_addr = (unsigned short)((unsigned int)pdat);
	reg_dma5_ctrl &= ~(FLD_DMA_BUF_SIZE|FLD_DMA_WR_MEM);
	reg_dma5_ctrl |= MASK_VAL(FLD_DMA_BUF_SIZE,0x20);

}


/**
 * @brief     This fuction servers to start the pwm's IRQ sending.
 * @param[in] none.
 * @return	  none.
 */
static inline void pwm_ir_dma_fifo_start_tx(void)
{
	reg_dma_chn_en |= FLD_DMA_CHN_PWM;////DMA Channel enable
	reg_dma_tx_rdy0 |= FLD_DMA_CHN_PWM;
}


