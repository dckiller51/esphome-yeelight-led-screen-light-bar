//#include "../../common.h"
#include "../../drivers.h"
#include "frame.h"
#include "pairing_op.h"
#include "led.h"
#include "rf_control.h"

extern unsigned int  sys_run_tick;
extern unsigned char g_state;
const unsigned char rf_channel[4]={1,24,51,76};

unsigned char led_on_cnt=0;
/***********************************************************
 * �������ܣ�RF��ʼ��
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void rf_init_func(void)
{
//	rf_drv_init(RF_MODE_BLE_1M_NO_PN);//����ͨѶ����
	rf_set_access_code_len(5);
	rf_set_access_code_value(0,0x9539517671);
//	rf_set_access_code_len(4);
//	rf_set_access_code_value(0,0x8e89bed6);
	rf_set_rx_buff(rx_packet,RX_PACKGET_SIZE,1);
	rf_set_power_level_index(RF_POWER_7dBm);
	rf_set_trx_state(RF_MODE_RX,rf_channel[0]);
	rf_irq_clr_mask(0xffff);                      //�ȹص����е�RF�ж�
	rf_irq_set_mask(FLD_RF_IRQ_RX);//��RF RX�ж�
	irq_set_mask(FLD_IRQ_ZB_RT_EN);//��RF���ж�
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
/***********************************************************
 * �������ܣ�ת���м����ݰ�
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void send_relay_pkt(void)
{
	unsigned char i;
	for(i=0;i<4;i++){
		rf_set_trx_state(RF_MODE_TX,rf_channel[i]);
		delay_us(200);
		rf_tx_pkt((void *)&relay_pkt);
		while(!rf_is_tx_finish());
		rf_clr_tx_finish();
	}
	rf_change_channel_func();
}
/***********************************************************
 * �������ܣ��յ�RF���ݺ�����
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void rf_packget_pro_func(void)
{
	if(g_packget_new){//���µ�rf���ݰ�
		g_packget_new=0;
		if(g_state==PAIRRING_STATE){//�Ƿ�Ϊ����״̬
			if(g_packget_cmd==LED_ON_CMD){//����ֵ�Ƿ�Ϊ���ƽ�
				sys_run_tick=get_sys_tick();
				remote_save_grp=g_packget_grp;//�������
				remote_save_pid=g_packget_pid;//����ң����ID
				g_state=CLEARCODE_STATE;//������һ��״̬
				led_on_cnt=1;
			}else if(g_packget_cmd!=LED_NONE_CMD){//��Ϊ���Ƽ�
				if(paired_ID_match(g_packget_pid,g_packget_grp)){//ң������ID������Ƿ�ƥ�䣬��ƥ�䣬���˳����룬��������״̬
					g_state=NORMAL_STATE;
				}
			}
		}else if(g_state==CLEARCODE_STATE){
			if(remote_save_pid==g_packget_pid){//ң����ID�Ƿ�һ��
				if(g_packget_cmd==LED_ON_CMD){//�Ƿ�Ϊ���Ƽ�
					sys_run_tick=get_sys_tick();//���½��������ʱ���
					led_on_cnt++;
					if(led_on_cnt>4){//����4��������
						clear_pared_code_func();
						led_flash_updata(5);
						g_state=NORMAL_STATE;
					}
				}else if(g_packget_cmd!=LED_NONE_CMD){//�ǿ��Ƽ��ռ�ֵ
					if(paired_ID_match(g_packget_pid,g_packget_grp)){//ң������ID������Ƿ�ƥ�䣬��ƥ�䣬���˳����룬��������״̬
						g_state=NORMAL_STATE;
					}
				}
			}
		}else if(g_state==NORMAL_STATE){//����״̬
			send_relay_pkt();
			if(paired_ID_match(g_packget_pid,g_packget_grp)){//ң������ID������Ƿ�ƥ�䣬��ƥ�䣬��ִ������
				if(g_packget_cmd!=LED_SET_CHRO_LUMI_CMD){
					led_event_proc_func(g_packget_cmd);//ִ������
				}else{
					led_set_lumi_chrome_func(g_packget_lumi,g_packget_chrome);//����ɫ��ֵ
				}
			}
		}
	}
}
