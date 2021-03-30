//#include "../../common.h"
#include "../../drivers.h"
#include "../../user_drivers.h"
#include "led_control.h"
#include "frame.h"
#include "state.h"
#include "led_rgb.h"
#include "led_yl.h"
#include "pairing_op.h"

unsigned int led_yl_updata_tick;     //ɫ�µƸ���ʱ���
unsigned int led_rgb_updata_tick;    //RGB�Ƹ���ʱ���
unsigned char led_rgb_breath_state;  //����ģʽ������״̬
const unsigned short breath_value[14][3]={   //����ģʽ�����Ƶ�����ֵ�ֱ�Ϊ�졢�̡���
		{0,0,0},
		{1000,0,0},
		{0,0,0},
		{1000,1000,0},
		{0,0,0},
		{0,1000,0},
		{0,0,0},
		{0,1000,1000},
		{0,0,0},
		{0,0,1000},
		{0,0,0},
		{1000,0,1000},
		{0,0,0},
		{1000,1000,1000},
};
/***********************************************************
 * �������ܣ�LED������ʼ��
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void led_para_init_func(void)
{
	unsigned char i;
	unsigned char *Pr=(void *)&led_control;
	for(unsigned char i=0;i<sizeof(LED_Control_Info_t);i++)//��ȡeeprom���������
		*Pr++=fm24c02_read_func(i);

	if(led_control.paire_index >= MAX_PAIRED_REMOTER)//���������ֵ����Ĭ��Ϊ0
		led_control.paire_index = 0;

	if(led_control.luminance_index > MAX_LUMINANCE_INDEX)//�����������ֵ��Ĭ��Ϊ���ֵ
		led_control.luminance_index = MAX_LUMINANCE_INDEX;

	if(led_control.chroma_index > MAX_CHROME_INDEX)//�������ɫ��ֵ��Ĭ��Ϊ���ֵ
		led_control.chroma_index = MAX_CHROME_INDEX;

	if(led_control.led_state>=LED_LAST_STATE)//Ĭ��Ϊɫ�µƿ�
		led_control.led_state=LED_YL_ON_STATE;

	for(i=0;i<3;i++){//����RGBĬ�ϵ����
		if(led_control.rgb_value[i]>MAX_VALUE_RGB)
			led_control.rgb_value[i]=MAX_VALUE_RGB;
	}
}
/***********************************************************
 * �������ܣ�LED��ʼ��
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void led_init_func(void)
{
	led_para_init_func();
	if((led_control.led_state==LED_YL_ON_STATE)||(led_control.led_state==LED_OFF_STATE)){//��״̬Ϊɫ�µƿ���Ϊ��ʱĬ��Ϊɫ�µƿ�ģʽ
		led_on_func();
	}else if(led_control.led_state==LED_RGB_ON_STATE){//RGB״̬����Ϊ������ģʽ������Զ�����
		led_on_rgb_func();
	}
}
/***********************************************************
 * �������ܣ�LED��˸ģʽ
 * ��       ����Cnt  ��˸�Ĵ���
 * �� ��  ֵ��
 **********************************************************/
void led_flash_updata_func(unsigned char Cnt)
{
	led_flash_cnt=Cnt;
}
/***********************************************************
 * �������ܣ��ƵĴ������
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void led_pask_process_func(void)
{
	if(led_flash_cnt){//�Ƿ�����˸
		if(timeout_us(led_flash_tick,500000)){//�Ƿ񵽴���˸ʱ��
			led_flash_tick=get_sys_tick();
			if(led_control.led_state==LED_YL_ON_STATE){//����״̬�������ص�
				led_control.led_state=LED_OFF_STATE;
				led_off_func();
			}else if(led_control.led_state==LED_OFF_STATE){//�ص�״̬����������
				led_control.led_state=LED_YL_ON_STATE;
				led_on_func();
				if(led_flash_cnt!=0xff)//����˸������Ϊ0xff�����ʾ���޴�
					led_flash_cnt--;
			}
		}
	}

	if(led_state_change_flag){//ɫ�µ���״̬����
		if(timeout_us(led_yl_updata_tick,3000)){//ÿ3ms����һ��
			led_yl_updata_tick=get_sys_tick();
			if(led_lumina_cur!=led_lumina_target){//�����б仯
				led_lumina_cur=lumina_one_step_updata(led_lumina_target,led_lumina_cur);
			}

			if(led_chroma_cur!=led_chroma_target){//ɫ���б仯
				led_chroma_cur=chroma_one_step_updata(led_chroma_target,led_chroma_cur);
			}

			led_pwm_control_func(led_lumina_cur,led_chroma_cur);//����LED
			if( (led_chroma_cur==led_chroma_target) && (led_lumina_cur==led_lumina_target) ){//�仯���
				led_state_change_flag=0;
				save_led_state_info_func();//����״̬
			}
		}
	}

	if(led_rgb_state_change_flag){//RGB״̬����
		if(timeout_us(led_rgb_updata_tick,3000)){//ÿ3ms����һ��
			led_rgb_updata_tick=get_sys_tick();
			if(led_red_cur!=led_red_target){
				led_red_cur=lumina_one_step_updata(led_red_target,led_red_cur);
			}

			if(led_green_cur!=led_green_target){
				led_green_cur=lumina_one_step_updata(led_green_target,led_green_cur);
			}

			if(led_blue_cur!=led_blue_target){
				led_blue_cur=lumina_one_step_updata(led_blue_target,led_blue_cur);
			}

			led_set_rgb_power_func(led_red_cur,led_green_cur,led_blue_cur);//����PWMֵ
			if(led_red_cur==led_red_target&&led_green_cur==led_green_target&&led_blue_cur==led_blue_target){//�������
				led_rgb_state_change_flag=0;
			}
		}
	}

	if(led_control.led_state==LED_RGB_BREATH_STATE){//����ģʽ
		if(led_rgb_state_change_flag==0){//״̬�����Ƿ����
			led_rgb_breath_state++;//�����������һ״̬
			if(led_rgb_breath_state>13)
				led_rgb_breath_state=0;
			led_red_target=breath_value[led_rgb_breath_state][0];
			led_green_target=breath_value[led_rgb_breath_state][1];
			led_blue_target=breath_value[led_rgb_breath_state][2];
			led_rgb_state_change_flag=1;//״̬���±�־
		}
	}
}
