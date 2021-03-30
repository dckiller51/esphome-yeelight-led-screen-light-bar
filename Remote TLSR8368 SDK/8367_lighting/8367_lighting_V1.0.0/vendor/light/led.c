//#include "../../common.h"
#include "../../drivers.h"
#include "../../user_drivers.h"
#include "user_pwm.h"
#include "frame.h"
#include "led.h"

unsigned short led_lumina_cur;
unsigned short led_lumina_target;
unsigned short led_chroma_cur;
unsigned short led_chroma_target;
unsigned char led_state_change_flag;
unsigned int led_change_tick;

unsigned char led_flash_cnt;
unsigned int  led_flash_tick;

const unsigned short led_luminance_value[MAX_LUMINANCE_INDEX+1]={40,67,102,144,193,250,313,384,462,547,640,740,846,1000};
const unsigned char  led_chroma_value[MAX_CHROME_INDEX+1]={0,10,20,30,40,50,60,70,80,90,100};
/***********************************************************
 * �������ܣ�LED������ʼ��
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void led_para_init_func(void)
{
	unsigned char *Pr=(void *)&led_control;
	for(unsigned char i=0;i<sizeof(LED_control_info_t);i++)//��ȡeeprom���������
		*Pr++=fm24c02_read_func(i);

	if(led_control.paire_index >= MAX_PAIRED_REMOTER)//���������ֵ����Ĭ��Ϊ0
		led_control.paire_index = 0;

	if(led_control.luminance_index > MAX_LUMINANCE_INDEX)//�����������ֵ��Ĭ��Ϊ���ֵ
		led_control.luminance_index = MAX_LUMINANCE_INDEX;

	if(led_control.chroma_index > MAX_CHROME_INDEX)//�������ɫ��ֵ��Ĭ��Ϊ���ֵ
		led_control.chroma_index = MAX_CHROME_INDEX;

	if(led_control.seg_index > 3)//��������л�״ֵ̬��Ĭ��Ϊ0
		led_control.seg_index = 0;
}
/***********************************************************
 * �������ܣ��������
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void led_para_save_func(void)
{
	unsigned char *Pr=(void *)&led_control;
	for(unsigned char i=0;i<sizeof(LED_control_info_t);i++)
		fm24c02_write_func(i,*Pr++);
}
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
 * �������ܣ�����
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void led_on_func(void)
{
	led_state_change_flag=1;
	led_lumina_target=led_luminance_value[led_control.luminance_index];//��������Ŀ��ֵ
	led_chroma_target=led_chroma_value[led_control.chroma_index];//����ɫ��Ŀ��ֵ
	if(led_control.led_on==0){//����ǰ״̬Ϊ�ص�
		led_lumina_cur=0;//���ȵ�ǰֵΪ0
		led_chroma_cur=led_chroma_target;//ɫ�µ�ǰֵΪ0
		led_control.led_on=1;//led״̬Ϊ����״̬
	}
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
	led_control.led_on=0;
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
 * �������ܣ�LED��ʼ��
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void led_init_func(void)
{
	relay_pkt.dma_len=sizeof(rf_packet_led_remote_t)-sizeof(relay_pkt.dma_len);
	relay_pkt.rf_len = relay_pkt.dma_len-1;
	relay_pkt.rf_len1 = relay_pkt.dma_len - 2;
	relay_pkt.vid=0x5453;
	led_para_init_func();
	led_lumina_cur=0;
	led_chroma_cur=led_chroma_value[led_control.chroma_index];
	if(led_control.power_on_recover==0xff){//��Ϊ0xffʱ���л�״̬
		led_control.seg_index++;
		led_control.seg_index&=3;
		led_luminace_segment_set_func(led_control.seg_index);//����״̬
	}else{//���л�״̬ʱ����������
		led_on_func();
	}
	led_control.power_on_recover=0xff;
	led_para_save_func();//����״̬
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
 * �������ܣ�����LED
 * ��       ����Lumina  ����ֵ
 *        Chroma  ɫ��ֵ
 * �� ��  ֵ��
 **********************************************************/
void led_power_control_func(int Lumina, int Chroma)
{
	unsigned char i;
	led_lumina_target = led_lumina_cur = Lumina;
	led_chroma_target = led_chroma_cur = Chroma;
	for(i=0;i<(MAX_LUMINANCE_INDEX+1);i++)//�������ȶ�Ӧ���±�ֵ
		if(Lumina<=led_luminance_value[i])
			break;
	led_control.luminance_index = i;
	led_control.chroma_index = (Chroma+5)/10;//ɫ��һ��Ϊ10
	led_pwm_control_func(Lumina, Chroma);

}
/***********************************************************
 * �������ܣ�����led��4��״̬
 * ��       ����seg_index ����Ӧ��״̬
 * �� ��  ֵ��
 **********************************************************/
void led_luminace_segment_set_func(unsigned char seg_index)
{

	switch(seg_index){
		case 0:
//			100% 0%
			led_power_control_func(MAX_LUMINANCE, 100);
			break;
		case 1:
//			0%  100%
			led_power_control_func(MAX_LUMINANCE, 0);
			break;
		case 2:
//			50%  50%
			led_power_control_func(MAX_LUMINANCE, 50);
			break;
		case 3:
//			0.4%  50%
			led_power_control_func(LOW_LIGHT_LUMINACE, 50);
			break;
	}
}
/***********************************************************
 * �������ܣ�ң��������ִ��
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void led_event_proc_func(unsigned char Cmd)
{
	if(led_flash_cnt)return;
	if(Cmd==LED_LUMINANCE_INC_CMD||Cmd==LED_LUMINANCE_DEC_CMD||Cmd==LED_CHROME_INC_CMD||Cmd==LED_CHROME_DEC_CMD)
		if(led_control.led_on==0)  //�ص�״̬������ɫ�����ȵ���
			return;

	switch(Cmd){
		case LED_NONE_CMD:

			break;

		case LED_ON_CMD://����
			led_on_func();
			break;

		case LED_OFF_CMD://�ص�
			led_off_func();
			break;

		case LED_LUMINANCE_INC_CMD://���ȼ�
			led_updata_luminance_func(1);
			break;

		case LED_LUMINANCE_DEC_CMD://���ȼ�
			led_updata_luminance_func(0);
			break;

		case LED_CHROME_INC_CMD://ɫ�¼�
			led_updata_chroma_func(1);
			break;

		case LED_CHROME_DEC_CMD://ɫ�¼�
			led_updata_chroma_func(0);
			break;

		case LED_NIGHT_CMD://ҹ��ģʽ
			led_updata_lumi_chrome_func(LOW_LIGHT_LUMINACE,50);
			break;

		default:

			break;
	}
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
 * �������ܣ���˸��������
 * ��       ����Flash_cnt  ��˸�Ĵ���
 * �� ��  ֵ��
 **********************************************************/
void led_flash_updata(unsigned char Flash_cnt)
{
	led_flash_cnt=Flash_cnt;
}
/***********************************************************
 * �������ܣ�LED������
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void led_task_process_func(void)
{
	if(led_flash_cnt){//led�Ƿ�����˸
		if(timeout_us(led_flash_tick,500000)){//500ms����һ��״̬
			led_flash_tick=get_sys_tick();
			if(led_control.led_on==0){
				led_control.led_on=1;
				led_power_control_func(500,50);
				if(led_flash_cnt!=0xff)//Ϊ0xffʱ�����޴���˸
					led_flash_cnt--;
			}else{
				led_control.led_on=0;
				led_power_control_func(0,50);
			}
		}
	}

	if(led_state_change_flag){//LED״̬�б仯
		if(timeout_us(led_change_tick,5000)){//ÿ5ms����һ��
			led_change_tick=get_sys_tick();
			if(led_lumina_cur!=led_lumina_target){//�����б仯
				led_lumina_cur=lumina_one_step_updata(led_lumina_target,led_lumina_cur);
			}

			if(led_chroma_cur!=led_chroma_target){//ɫ���б仯
				led_chroma_cur=chroma_one_step_updata(led_chroma_target,led_chroma_cur);
			}

			led_pwm_control_func(led_lumina_cur,led_chroma_cur);//����LED
			if( (led_chroma_cur==led_chroma_target) && (led_lumina_cur==led_lumina_target) ){//�仯���
				led_state_change_flag=0;
				led_control.power_on_recover=0;
				led_para_save_func();//����״̬
			}
		}
	}
}
