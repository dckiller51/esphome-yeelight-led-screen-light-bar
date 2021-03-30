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
	if(src&FLD_RF_IRQ_RX&reg_rf_irq_mask){                //接收中断，每接到数据该标志都置1，不管数据正确与否
		irq_rx++;
		rf_irq_clr_src(FLD_RF_IRQ_RX);
		index=rf_rx_buffer_get()&1;                       //读取缓存的位置
		unsigned char *p=rx_packet+index*RX_PACKAGE_SIZE;            //接收缓存的指针地址
		if(Rf_RCV_PKT_Valid(p)){                          //校验接收包
			LED_Package_t *pkt=(LED_Package_t *)(p+8);
			if(pkt->vid==0x5453){//匹配产品ID
				g_package_new=1;
				g_package_cmd=pkt->key_control>>4;        //包的命令值
				g_package_pid=pkt->pid;                   //遥控器ID
				g_package_seq=pkt->pkt_seq;               //包的序列号
				if(g_package_cmd==KEY_SET_RGB_CMD){       //如果是设置RGB亮度命令，则value[3]分别是RGB的亮度值
					g_package_red=pkt->value[0];
					g_package_green=pkt->value[1];
					g_package_blue=pkt->value[2];
				}else{                                    //如果是设置色温灯的亮度及色温值，则value[0]、value[1]分别表示亮度、色温值，其它命令值忽略这个数组
					g_package_lumi=pkt->value[0];
					g_package_chroma=pkt->value[1];
				}
			}
		}
	}
}
