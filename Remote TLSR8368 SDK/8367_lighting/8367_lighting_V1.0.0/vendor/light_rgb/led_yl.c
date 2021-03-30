//#include "../../common.h"
#include "../../drivers.h"
#include "../../user_drivers.h"
#include "led_yl.h"
#include "state.h"
#include "frame.h"
#include "key_def.h"
#include "user_pwm.h"

const unsigned short led_luminance_value[MAX_LUMINANCE_INDEX+1]={40,67,102,144,193,250,313,384,462,547,640,740,846,1000};
const unsigned char  led_chroma_value[MAX_CHROME_INDEX+1]={0,10,20,30,40,50,60,70,80,90,100};

/***********************************************************
 * �������ܣ�ɫ������ֵ״̬����
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void led_updata_lumi_chrome_func(unsigned short Lumi,unsigned char Chroma)
{
	led_lumina_target=Lumi;
	led_chroma_target=Chroma;
	led_state_change_flag=1;
}
/***********************************************************
 * �������ܣ�����LED��PWM
 * ��       ����Lumina   ����ֵ
 *        Chroma   ɫ��ֵ
 * �� ��  ֵ��
 **********************************************************/
void led_pwm_control_func(int Lumina, int Chroma)
{
	int Whrite_pwm_val, Yellow_pwm_val;

	Whrite_pwm_val = Lumina * Chroma/100;//�׵Ƶ�ռ�ձȣ�����ֵ*����
	Yellow_pwm_val = Lumina - Whrite_pwm_val;//�ƵƵ�ռ�ձ�

	PWM_DutyValueSet(PWM4, Whrite_pwm_val);
	PWM_DutyValueSet(PWM0, Yellow_pwm_val);
}
/***********************************************************
 * �������ܣ����㵱ǰ����ֵ
 * ��       ����target   ����Ŀ��ֵ
 *        cur      ��ǰ����ֵ
 * �� ��  ֵ������������ֵ
 **********************************************************/
unsigned short lumina_one_step_updata(unsigned short target,unsigned short cur)
{
	unsigned short temp=cur>>6;//����ֵԽ�󣬱仯��Խ��
	if(temp==0)
		temp=1;

	if(target>cur){
		cur+=temp;
		if(cur>target)
			cur=target;
	}else{
		cur-=temp;
		if(cur<target)
			cur=target;
	}
	return cur;
}
/***********************************************************
 * �������ܣ�����ɫ��ֵ
 * ��       ����target   ����Ŀ��ֵ
 *        cur      ��ǰ����ֵ
 * �� ��  ֵ��������ɫ��ֵ
 **********************************************************/
unsigned short chroma_one_step_updata(unsigned short target,unsigned short cur)
{
	if(target>cur)
		cur++;
	else
		cur--;
	return cur;
}
/***********************************************************
 * �������ܣ�����
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void led_on_func(void)
{
	if(led_control.led_state!=LED_YL_ON_STATE){//����ǰ״̬Ϊ�ص�
		led_lumina_cur=0;//���ȵ�ǰֵΪ0
		led_chroma_cur=0;//ɫ�µ�ǰֵΪ0
		led_control.led_state=LED_YL_ON_STATE;//led״̬Ϊ����״̬
	}
	led_state_change_flag=1;
	led_lumina_target=led_luminance_value[led_control.luminance_index];//��������Ŀ��ֵ
	led_chroma_cur=led_chroma_target=led_chroma_value[led_control.chroma_index];//����ɫ��Ŀ��ֵ
}
/***********************************************************
 * �������ܣ�LED�ص�
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void led_off_func(void)
{
	led_state_change_flag=1;
	led_lumina_target=0;
//	led_chroma_target=0;
	led_control.led_state=LED_OFF_STATE;
}
/***********************************************************
 * �������ܣ���������
 * ��       ����Type   �������� 1Ϊ��  0Ϊ��
 * �� ��  ֵ��
 **********************************************************/
void led_updata_luminance_func(unsigned char Type)
{
	unsigned short ChromaValue;
	unsigned short LuminaceValue;
	if(Type){//Ϊ1�����
		led_control.luminance_index++;
	}else{
		if(led_control.luminance_index)//��Ϊ0�����Ϊ0�򱣴治��
			led_control.luminance_index--;
	}
	if(led_control.luminance_index>MAX_LUMINANCE_INDEX)//���ȳ������ֵ����Ĭ��Ϊ���ֵ
		led_control.luminance_index=MAX_LUMINANCE_INDEX;
	ChromaValue=led_chroma_value[led_control.chroma_index];
	LuminaceValue=led_luminance_value[led_control.luminance_index];
	led_updata_lumi_chrome_func(LuminaceValue,ChromaValue);
}
/***********************************************************
 * �������ܣ�����ɫ��
 * ��       ����Type   �������� 1Ϊ��  0Ϊ��
 * �� ��  ֵ��
 **********************************************************/
void led_updata_chroma_func(unsigned char Type)
{
	unsigned short ChromaValue;
	unsigned short LuminaceValue;
	if(Type){//Ϊ1�����
		led_control.chroma_index++;
	}else{
		if(led_control.chroma_index)//��Ϊ0�����Ϊ0���ֲ���
			led_control.chroma_index--;
	}
	if(led_control.chroma_index>MAX_CHROME_INDEX)//�������ֵ����Ĭ��Ϊ���ֵ
		led_control.chroma_index=MAX_CHROME_INDEX;
	ChromaValue=led_chroma_value[led_control.chroma_index];
	LuminaceValue=led_luminance_value[led_control.luminance_index];
	led_updata_lumi_chrome_func(LuminaceValue,ChromaValue);//����״̬
}
/***********************************************************
 * �������ܣ�����ɫ������ֵ
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void led_set_lumi_chrome_func(unsigned short Lumi,unsigned short Chroma)
{
	unsigned char i;
	if(Lumi>led_luminance_value[MAX_LUMINANCE_INDEX])//�������ֵ��Ĭ��Ϊ���
		Lumi=led_luminance_value[MAX_LUMINANCE_INDEX];
	if(Chroma>led_chroma_value[MAX_CHROME_INDEX])//�������ֵ��Ĭ��Ϊ���
		Chroma=led_chroma_value[MAX_CHROME_INDEX];
	led_control.chroma_index=Chroma/10;
	for(i=0;i<(MAX_LUMINANCE_INDEX+1);i++)
		if(Lumi<=led_luminance_value[i])
			break;
	led_control.luminance_index=i;
	led_updata_lumi_chrome_func(Lumi,Chroma);
}
/***********************************************************
 * �������ܣ�ң��������ִ��
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void led_event_proc_func(unsigned char Cmd)
{
	if(Cmd==KEY_LUMINANCE_INC_CMD||Cmd==KEY_LUMINANCE_DEC_CMD||Cmd==KEY_CHROME_INC_CMD||Cmd==KEY_CHROME_DEC_CMD)
		if(led_control.led_state==LED_OFF_STATE)  //�ص�״̬������ɫ�����ȵ���
			return;

	switch(Cmd){
		case KEY_NONE_CMD:

			break;

		case KEY_ON_CMD://����
			led_on_func();
			break;

		case KEY_OFF_CMD://�ص�
			led_off_func();
			break;

		case KEY_LUMINANCE_INC_CMD://���ȼ�
			led_updata_luminance_func(1);
			break;

		case KEY_LUMINANCE_DEC_CMD://���ȼ�
			led_updata_luminance_func(0);
			break;

		case KEY_CHROME_INC_CMD://ɫ�¼�
			led_updata_chroma_func(1);
			break;

		case KEY_CHROME_DEC_CMD://ɫ�¼�
			led_updata_chroma_func(0);
			break;

		case KEY_NIGHT_CMD://ҹ��ģʽ
			led_updata_lumi_chrome_func(LOW_LIGHT_LUMINACE,50);
			break;

		default:

			break;
	}
}
