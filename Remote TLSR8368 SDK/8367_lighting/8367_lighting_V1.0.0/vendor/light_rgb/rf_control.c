//#include "../../common.h"
#include "../../drivers.h"
#include "../../user_drivers.h"
#include "frame.h"
#include "pairing_op.h"
#include "key_def.h"
#include "state.h"
#include "led_control.h"
#include "led_rgb.h"
#include "led_yl.h"


const unsigned char rf_channel[4]={1,24,51,76};

unsigned char g_state;
unsigned int sys_run_tick;
unsigned char pre_package_seq;
unsigned char pre_package_cmd;
void rf_init_func(void)
{
	rf_set_access_code_len(4);
	rf_set_access_code_value(0,0x8e89bed6);
	rf_set_rx_buff(rx_packet,RX_PACKAGE_SIZE,1);
	rf_set_power_level_index(RF_POWER_7dBm);
	rf_set_trx_state(RF_MODE_RX,rf_channel[0]);
	rf_irq_clr_mask(0xffff);                      //�ȹص����е�RF�ж�
	rf_irq_set_mask(FLD_RF_IRQ_RX);//��RF RX�ж�
	irq_set_mask(FLD_IRQ_ZB_RT_EN);//��RF���ж�
	g_state=PAIRING_STATE;
	sys_run_tick=get_sys_tick();
}

/***********************************************************
 * �������ܣ���Ƶ��4��Ƶ��ѭ���л�
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void rf_change_channel_func(void)
{
	static unsigned char Channel_index;
	Channel_index++;
	Channel_index&=3;
	rf_set_trx_state(RF_MODE_RX,rf_channel[Channel_index]);
}
unsigned int debug,debug1;
/***********************************************************
 * �������ܣ��յ�RF���ݺ�����
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void rf_packget_pro_func(void)
{
	if(g_state==PAIRING_STATE){//���ڶ���״̬
		if(g_package_new){//�յ��µ����ݰ�
			g_package_new=0;
			if(g_package_cmd==KEY_PAIRE_CODE_CMD){//��������
				if(led_control.led_state!=LED_YL_ON_STATE){
					led_control.led_state=LED_OFF_STATE;
				}
				led_rgb_off_func();
				save_remote_ID_func(g_package_pid);//����ID
				led_flash_updata_func(3);          //��˸3��
				g_state=NORMAL_STATE;              //�����׼ģʽ
			}else if(g_package_cmd==KEY_CLEAR_CODE_CMD){//��������
				if(led_control.led_state!=LED_YL_ON_STATE){
					led_control.led_state=LED_OFF_STATE;
				}
				led_rgb_off_func();
				clear_remote_ID_func();                 //������б����IDֵ
				led_flash_updata_func(5);               //��˸5��
				g_state=NORMAL_STATE;                   //�����׼ģʽ
			}else if(g_package_cmd!=KEY_NONE_CMD){      //���ǿռ�
				if(paired_ID_match(g_package_pid))      //��IDƥ��
					g_state=NORMAL_STATE;               //�˳�����ģʽ
			}
		}

		if(timeout_us(sys_run_tick,6000000)){      //����6s�Ժ��˳�����ģʽ
			g_state=NORMAL_STATE;
		}
	}else if(g_state==NORMAL_STATE){                    //��������ģʽ
		if(g_package_new){                              //�յ��°�
			g_package_new=0;
			if(paired_ID_match(g_package_pid)){         //ID�Ƿ�ƥ��
				if(g_package_cmd!=pre_package_cmd||g_package_seq!=pre_package_seq){//ID������к��Ƿ���ͬ����������ͬ����Ϊ��ͬһ����������
					if(led_control.led_state==LED_RGB_ON_STATE||led_control.led_state==LED_RGB_BREATH_STATE){//��ǰ�ƴ���RGBģʽ
						if(g_package_cmd==KEY_ON_CMD){//ɫ�µƿ��������˳�RGBģʽ
							led_rgb_off_func();       //�ر�RGB
							led_on_func();            //��ɫ�µ�
							led_control.led_state = LED_YL_ON_STATE;
						}else if(g_package_cmd==KEY_OFF_CMD){
							led_rgb_off_func();       //�ر�RGB
							led_off_func();            //�ر�ɫ�µ�
							led_control.led_state = LED_OFF_STATE;
						}else if(g_package_cmd==KEY_SET_RGB_CMD){//����Ϊ����RGBֵ
							led_control.led_state=LED_RGB_ON_STATE;
							led_red_target=g_package_red;
							led_green_target=g_package_green;
							led_blue_target=g_package_blue;
							led_rgb_state_change_flag=1;//״̬���±�־
						}else if(g_package_cmd==KEY_BREATH_RGB_MODE_CMD){//RGB����ģʽ
							led_control.led_state=LED_RGB_BREATH_STATE;
						}
					}else{//��ǰ״̬Ϊɫ�µ�ģʽ
						if(g_package_cmd<KEY_PAIRE_CODE_CMD){//��������Ϊɫ�µƵİ�������
							if(g_package_cmd!=KEY_SET_CHRO_LUMI_CMD){//������ɫ�µ�
								led_event_proc_func(g_package_cmd);//ִ������
							}else{
								led_set_lumi_chrome_func(g_package_lumi,g_package_chroma);//����ɫ��ֵ
							}
						}else if(g_package_cmd==KEY_SET_RGB_CMD){//����RGB��ֵ
							led_off_func();
							led_control.led_state=LED_RGB_ON_STATE;
							led_rgb_set_func(g_package_red,g_package_green,g_package_blue);
						}else if(g_package_cmd==KEY_BREATH_RGB_MODE_CMD){//RGB����ģʽ
							led_off_func();
							led_control.led_state=LED_RGB_BREATH_STATE;
						}
					}
				}
			}
			pre_package_cmd=g_package_cmd;//��������
			pre_package_seq=g_package_seq;//�������к�
		}
	}
}
