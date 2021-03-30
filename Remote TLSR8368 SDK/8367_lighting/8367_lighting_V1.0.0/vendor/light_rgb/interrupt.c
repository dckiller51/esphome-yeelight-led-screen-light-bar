//#include "../../common.h"
#include"../../drivers.h"
#include "frame.h"
#include "state.h"
#include "key_def.h"
unsigned int irq_rx;
_attribute_ram_code_ __attribute__((optimize("-Os"))) void irq_handler(void)
{
	unsigned short src=rf_irq_get_src();
	unsigned char index;
	if(src&FLD_RF_IRQ_RX&reg_rf_irq_mask){                //�����жϣ�ÿ�ӵ����ݸñ�־����1������������ȷ���
		irq_rx++;
		rf_irq_clr_src(FLD_RF_IRQ_RX);
		index=rf_rx_buffer_get()&1;                       //��ȡ�����λ��
		unsigned char *p=rx_packet+index*RX_PACKAGE_SIZE;            //���ջ����ָ���ַ
		if(Rf_RCV_PKT_Valid(p)){                          //У����հ�
			LED_Package_t *pkt=(LED_Package_t *)(p+8);
			if(pkt->vid==0x5453){//ƥ���ƷID
				g_package_new=1;
				g_package_cmd=pkt->key_control>>4;        //��������ֵ
				g_package_pid=pkt->pid;                   //ң����ID
				g_package_seq=pkt->pkt_seq;               //�������к�
				if(g_package_cmd==KEY_SET_RGB_CMD){       //���������RGB���������value[3]�ֱ���RGB������ֵ
					g_package_red=pkt->value[0];
					g_package_green=pkt->value[1];
					g_package_blue=pkt->value[2];
				}else{                                    //���������ɫ�µƵ����ȼ�ɫ��ֵ����value[0]��value[1]�ֱ��ʾ���ȡ�ɫ��ֵ����������ֵ�����������
					g_package_lumi=pkt->value[0];
					g_package_chroma=pkt->value[1];
				}
			}
		}
	}
}
