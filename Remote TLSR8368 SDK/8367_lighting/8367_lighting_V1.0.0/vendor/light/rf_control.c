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
 * 函数功能：RF初始化
 * 参       数：
 * 返 回  值：
 **********************************************************/
void rf_init_func(void)
{
//	rf_drv_init(RF_MODE_BLE_1M_NO_PN);//设置通讯速率
	rf_set_access_code_len(5);
	rf_set_access_code_value(0,0x9539517671);
//	rf_set_access_code_len(4);
//	rf_set_access_code_value(0,0x8e89bed6);
	rf_set_rx_buff(rx_packet,RX_PACKGET_SIZE,1);
	rf_set_power_level_index(RF_POWER_7dBm);
	rf_set_trx_state(RF_MODE_RX,rf_channel[0]);
	rf_irq_clr_mask(0xffff);                      //先关掉所有的RF中断
	rf_irq_set_mask(FLD_RF_IRQ_RX);//打开RF RX中断
	irq_set_mask(FLD_IRQ_ZB_RT_EN);//开RF总中断
}
/***********************************************************
 * 函数功能：跳频，4个频点循环切换
 * 参       数：
 * 返 回  值：
 **********************************************************/
void rf_change_channel_func(void)
{
	static unsigned char Channel_index;
	Channel_index++;
	Channel_index&=3;
	rf_set_trx_state(RF_MODE_RX,rf_channel[Channel_index]);
}
/***********************************************************
 * 函数功能：转发中继数据包
 * 参       数：
 * 返 回  值：
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
 * 函数功能：收到RF数据后处理函数
 * 参       数：
 * 返 回  值：
 **********************************************************/
void rf_packget_pro_func(void)
{
	if(g_packget_new){//有新的rf数据包
		g_packget_new=0;
		if(g_state==PAIRRING_STATE){//是否为对码状态
			if(g_packget_cmd==LED_ON_CMD){//按键值是否为开灯健
				sys_run_tick=get_sys_tick();
				remote_save_grp=g_packget_grp;//保存组别
				remote_save_pid=g_packget_pid;//保存遥控器ID
				g_state=CLEARCODE_STATE;//进入下一个状态
				led_on_cnt=1;
			}else if(g_packget_cmd!=LED_NONE_CMD){//不为开灯键
				if(paired_ID_match(g_packget_pid,g_packget_grp)){//遥控器的ID及组别是否匹配，若匹配，则退出对码，进入正常状态
					g_state=NORMAL_STATE;
				}
			}
		}else if(g_state==CLEARCODE_STATE){
			if(remote_save_pid==g_packget_pid){//遥控器ID是否一致
				if(g_packget_cmd==LED_ON_CMD){//是否为开灯键
					sys_run_tick=get_sys_tick();//更新接收命令的时间点
					led_on_cnt++;
					if(led_on_cnt>4){//超过4次则清码
						clear_pared_code_func();
						led_flash_updata(5);
						g_state=NORMAL_STATE;
					}
				}else if(g_packget_cmd!=LED_NONE_CMD){//非开灯及空键值
					if(paired_ID_match(g_packget_pid,g_packget_grp)){//遥控器的ID及组别是否匹配，若匹配，则退出对码，进入正常状态
						g_state=NORMAL_STATE;
					}
				}
			}
		}else if(g_state==NORMAL_STATE){//正常状态
			send_relay_pkt();
			if(paired_ID_match(g_packget_pid,g_packget_grp)){//遥控器的ID及组别是否匹配，若匹配，则执行命令
				if(g_packget_cmd!=LED_SET_CHRO_LUMI_CMD){
					led_event_proc_func(g_packget_cmd);//执行命令
				}else{
					led_set_lumi_chrome_func(g_packget_lumi,g_packget_chrome);//设置色温值
				}
			}
		}
	}
}
