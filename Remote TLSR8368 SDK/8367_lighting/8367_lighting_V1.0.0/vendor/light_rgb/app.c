#include "app_config.h"
#include "../../user_drivers.h"
#include "state.h"
#include "rf_control.h"
#include "time_event.h"
#include "led_control.h"
#include "user_pwm.h"

#define DEBUG       1
/***********************************************************
 * 函数功能：初始化PWM
 * 参       数：
 * 返 回  值：
 **********************************************************/
void set_pwm_init_func(void)
{
	reg_rst1 &= ~FLD_RST1_PWM;
	reg_clk_en1 |= FLD_CLK1_PWM_EN;
#if DEBUG
	pwm_n_revert(PWM1);
	pwm_n_revert(PWM3);
	gpio_set_func(GPIO_PA7,AS_PWM0);
	gpio_set_func(GPIO_PA1,AS_PWM3_N);
	gpio_set_func(GPIO_PA2,AS_PWM1_N);
	gpio_set_func(GPIO_PA3,AS_PWM4);
	gpio_set_func(GPIO_PA4,AS_PWM2);

#else
	pwm_n_revert(PWM2);
	gpio_set_func(GPIO_PB2,AS_PWM0);
	gpio_set_func(GPIO_PB1,AS_PWM1);
	gpio_set_func(GPIO_PB4,AS_PWM2_N);
	gpio_set_func(GPIO_PB3,AS_PWM3);
	gpio_set_func(GPIO_PB5,AS_PWM4);

#endif
	user_PWMInit(PWM0,1000,16000);
	user_PWMInit(PWM1,1000,16000);
	user_PWMInit(PWM2,1000,16000);
	user_PWMInit(PWM3,1000,16000);
	user_PWMInit(PWM4,1000,16000);
	pwm_start(PWM0);
	pwm_start(PWM1);
	pwm_start(PWM2);
	pwm_start(PWM3);
	pwm_start(PWM4);
}
void user_init(void)
{
	rf_init_func();
	fm24c02_init_func(GPIO_PA5,GPIO_PB6);
	set_pwm_init_func();
	led_init_func();
	irq_enable();
}

void main_loop(void)
{
	time_event_process_func();
	rf_packget_pro_func();
	led_pask_process_func();
}
