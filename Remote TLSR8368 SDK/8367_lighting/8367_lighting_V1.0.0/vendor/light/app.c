#include "app_config.h"
#include "../../user_drivers.h"
#include "rf_control.h"
#include "led.h"
#include "time_event.h"
#include "pairing_op.h"
#include "user_pwm.h"
#include "../../user_drivers/i2c_drv.h"

extern unsigned char led_on_cnt;
unsigned char g_state;
unsigned int  sys_run_tick;

#define  DEBUG   1
/***********************************************************
 * �������ܣ���ʼ��PWM
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void set_pwm_init_func(void)
{
//	pwm_set_clk(CLOCK_SYS_CLOCK_HZ,16000);
	reg_rst1 &= ~FLD_RST1_PWM;
	reg_clk_en1 |= FLD_CLK1_PWM_EN;
#if DEBUG
	gpio_set_func(GPIO_PA7,AS_PWM0);
	gpio_set_func(GPIO_PA3,AS_PWM4);
#else
	gpio_set_func(GPIO_PB1,AS_PWM1);
	gpio_set_func(GPIO_PB4,AS_PWM2);
#endif
	user_PWMInit(PWM0,1000,16000);
	user_PWMInit(PWM4,1000,16000);
	PWM_DutyValueSet(PWM0,0);
	PWM_DutyValueSet(PWM4,0);
	pwm_start(PWM0);
	pwm_start(PWM4);
}
/***********************************************************
 * �������ܣ�ϵͳ״̬���
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void check_sys_state_func(void)
{
	if(g_state==PAIRRING_STATE){
		if(timeout_us(sys_run_tick,6000000)){//�ϵ�6sû���յ����Ƽ���ϵͳ��������ģʽ
			g_state=NORMAL_STATE;
		}
	}else if(g_state==CLEARCODE_STATE){//ϵͳ�յ����ƽ�
		if(timeout_us(sys_run_tick,500000)){//�յ����ư����󣬳���500msû���յ���һ�ο��ƽ�
			if(led_on_cnt==1){//ֻ�յ�1�ο��ƽ�������Ϊ�Ƕ���
				led_flash_updata(3);//��˸3��
				pair_id_save_func();//�������
			}
			g_state=NORMAL_STATE;//ϵͳ��������ģʽ
		}
	}
}

void user_init(void)
{
	rf_init_func();
	fm24c02_init_func(GPIO_PA5,GPIO_PB6);
	set_pwm_init_func();
	led_init_func();
	g_state=PAIRRING_STATE;
	sys_run_tick=get_sys_tick();
	irq_enable();
}


void main_loop(void)
{
	rf_packget_pro_func();
	led_task_process_func();
	time_event_process_func();
	check_sys_state_func();
}
