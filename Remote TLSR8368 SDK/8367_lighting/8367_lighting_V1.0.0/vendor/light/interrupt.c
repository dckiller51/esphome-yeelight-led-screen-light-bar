//#include "../../common.h"
#include"../../drivers.h"
#include "frame.h"

unsigned int irq_rx;
//unsigned char last_key_cmd;
//unsigned char last_seq;
unsigned int pkt_right_cnt;
typedef struct{
	unsigned int pid;
	unsigned char seq_no;
	unsigned char cmd;
}Last_control_info_t;
Last_control_info_t control_info[16];
unsigned char info_index;
//unsigned char check_pkt_info(unsigned int pid,unsigned char seq,unsigned char cmd)
_attribute_ram_code_ unsigned char check_pkt_info(rf_packet_led_remote_t *pkt)
{
	unsigned char i;
	for(i=0;i<16;i++){
		if(control_info[i].pid==pkt->pid){
			if(control_info[i].cmd!=pkt->control_key||pkt->rf_seq_no!=control_info[i].seq_no){
				control_info[i].cmd = pkt->control_key;
				control_info[i].seq_no = pkt->rf_seq_no;
				return 1;
			}else{
				return 0;
			}
		}
	}
	info_index++;
	info_index&=0x0f;
	control_info[info_index].cmd = pkt->control_key;
	control_info[info_index].seq_no = pkt->rf_seq_no;
	control_info[info_index].pid = pkt->pid;
	return 1;
}
_attribute_ram_code_ __attribute__((optimize("-Os"))) void irq_handler(void)
{
	unsigned short src=rf_irq_get_src();
	unsigned char index;
	if(src&FLD_RF_IRQ_RX&reg_rf_irq_mask){                //�����жϣ�ÿ�ӵ����ݸñ�־����1������������ȷ���
		irq_rx++;
		rf_irq_clr_src(FLD_RF_IRQ_RX);
		index=rf_rx_buffer_get()&1;                       //��ȡ�����λ��
		unsigned char *p=rx_packet+index*RX_PACKGET_SIZE;            //���ջ����ָ���ַ

		if(Rf_RCV_PKT_Valid(p)){                          //У����հ�
			rf_packet_led_remote_t *pkt=(rf_packet_led_remote_t *)(p+8);
			if(pkt->vid==0x5453){//ƥ���ƷID
//				if(last_seq!=pkt->rf_seq_no||last_key_cmd!=pkt->control_key){//���к�������ֵ�Ƿ�һ����������һ����һ����Ϊ��ͬ����
				if(check_pkt_info(pkt)){
					unsigned char *ptr=(unsigned char *)&relay_pkt.pid;
					for(index=0;index<11;index++)
						ptr[index]=p[index+16];
					g_packget_new=1;
					g_packget_pid=pkt->pid;
//					last_seq=pkt->rf_seq_no;
//					last_key_cmd=pkt->control_key;
					g_packget_cmd=pkt->control_key>>4;
					g_packget_grp=pkt->control_key&0x0f;
					g_packget_lumi=pkt->control_key_value[0];
					g_packget_chrome=pkt->control_key_value[1];
					pkt_right_cnt++;
				}
			}
		}
	}
}
