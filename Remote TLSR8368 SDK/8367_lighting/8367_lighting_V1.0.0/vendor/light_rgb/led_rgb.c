//#include "../../common.h"
#include "../../drivers.h"
#include "../../user_drivers.h"
#include "state.h"
#include "frame.h"
#include "led_rgb.h"
#include "user_pwm.h"
/***********************************************************
 * �������ܣ�RGB�ƿ�
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void led_on_rgb_func(void)
{
	led_red_target=led_control.rgb_value[0];
	led_green_target=led_control.rgb_value[1];
	led_blue_target=led_control.rgb_value[2];
	led_red_cur=0;
	led_green_cur=0;
	led_blue_cur=0;
	led_rgb_state_change_flag=1;
}
/***********************************************************
 * �������ܣ�RGB�ƹ�
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void led_rgb_off_func(void)
{
	led_red_target=0;
	led_green_target=0;
	led_blue_target=0;
	led_rgb_state_change_flag=1;
}
/***********************************************************
 * �������ܣ�����RGB�Ƶ�Ŀ�����ȣ�����
 * ��       ����Red_v     ��Ƶ�PWMֵ
 *        Green_v   �̵Ƶ�PWMֵ
 *        Blue_v    ���Ƶ�PWMֵ
 * �� ��  ֵ��
 **********************************************************/
void led_rgb_set_func(unsigned short Red_v,unsigned short Green_v,unsigned short Blue_v)
{
	led_red_target=Red_v;
	led_green_target=Green_v;
	led_blue_target=Blue_v;
	led_rgb_state_change_flag=1;//״̬���±�־
}
/***********************************************************
 * �������ܣ�ֱ������RGB�Ƶ�PWMֵ
 * ��       ����Red_v     ��Ƶ�PWMֵ
 *        Green_v   �̵Ƶ�PWMֵ
 *        Blue_v    ���Ƶ�PWMֵ
 * �� ��  ֵ��
 **********************************************************/
void led_set_rgb_power_func(unsigned short Red_v,unsigned short Green_v,unsigned short Blue_v)
{
	PWM_DutyValueSet(PWM2, Red_v);
	PWM_DutyValueSet(PWM1, Green_v);
	PWM_DutyValueSet(PWM3, Blue_v);
}
