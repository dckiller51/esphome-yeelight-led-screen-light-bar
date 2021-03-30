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
	rf_irq_clr_mask(0xffff);                      //先关掉所有的RF中断
	rf_irq_set_mask(FLD_RF_IRQ_RX);//打开RF RX中断
	irq_set_mask(FLD_IRQ_ZB_RT_EN);//开RF总中断
	g_state=PAIRING_STATE;
	sys_run_tick=get_sys_tick();
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
unsigned int debug,debug1;
/***********************************************************
 * 函数功能：收到RF数据后处理函数
 * 参       数：
 * 返 回  值：
 **********************************************************/
void rf_packget_pro_func(void)
{
	if(g_state==PAIRING_STATE){//处于对码状态
		if(g_package_new){//收到新的数据包
			g_package_new=0;
			if(g_package_cmd==KEY_PAIRE_CODE_CMD){//对码命令
				if(led_control.led_state!=LED_YL_ON_STATE){
					led_control.led_state=LED_OFF_STATE;
				}
				led_rgb_off_func();
				save_remote_ID_func(g_package_pid);//保存ID
				led_flash_updata_func(3);          //闪烁3次
				g_state=NORMAL_STATE;              //进入标准模式
			}else if(g_package_cmd==KEY_CLEAR_CODE_CMD){//清码命令
				if(led_control.led_state!=LED_YL_ON_STATE){
					led_control.led_state=LED_OFF_STATE;
				}
				led_rgb_off_func();
				clear_remote_ID_func();                 //清除所有保存的ID值
				led_flash_updata_func(5);               //闪烁5次
				g_state=NORMAL_STATE;                   //进入标准模式
			}else if(g_package_cmd!=KEY_NONE_CMD){      //不是空键
				if(paired_ID_match(g_package_pid))      //若ID匹配
					g_state=NORMAL_STATE;               //退出对码模式
			}
		}

		if(timeout_us(sys_run_tick,6000000)){      //开灯6s以后退出对码模式
			g_state=NORMAL_STATE;
		}
	}else if(g_state==NORMAL_STATE){                    //正常控制模式
		if(g_package_new){                              //收到新包
			g_package_new=0;
			if(paired_ID_match(g_package_pid)){         //ID是否匹配
				if(g_package_cmd!=pre_package_cmd||g_package_seq!=pre_package_seq){//ID与包序列号是否相同，两个都相同则认为是同一个包，丢弃
					if(led_control.led_state==LED_RGB_ON_STATE||led_control.led_state==LED_RGB_BREATH_STATE){//当前灯处于RGB模式
						if(g_package_cmd==KEY_ON_CMD){//色温灯开灯命令退出RGB模式
							led_rgb_off_func();       //关闭RGB
							led_on_func();            //打开色温灯
							led_control.led_state = LED_YL_ON_STATE;
						}else if(g_package_cmd==KEY_OFF_CMD){
							led_rgb_off_func();       //关闭RGB
							led_off_func();            //关闭色温灯
							led_control.led_state = LED_OFF_STATE;
						}else if(g_package_cmd==KEY_SET_RGB_CMD){//命令为设置RGB值
							led_control.led_state=LED_RGB_ON_STATE;
							led_red_target=g_package_red;
							led_green_target=g_package_green;
							led_blue_target=g_package_blue;
							led_rgb_state_change_flag=1;//状态更新标志
						}else if(g_package_cmd==KEY_BREATH_RGB_MODE_CMD){//RGB呼吸模式
							led_control.led_state=LED_RGB_BREATH_STATE;
						}
					}else{//当前状态为色温灯模式
						if(g_package_cmd<KEY_PAIRE_CODE_CMD){//按键命令为色温灯的按键命令
							if(g_package_cmd!=KEY_SET_CHRO_LUMI_CMD){//非设置色温灯
								led_event_proc_func(g_package_cmd);//执行命令
							}else{
								led_set_lumi_chrome_func(g_package_lumi,g_package_chroma);//设置色温值
							}
						}else if(g_package_cmd==KEY_SET_RGB_CMD){//设置RGB的值
							led_off_func();
							led_control.led_state=LED_RGB_ON_STATE;
							led_rgb_set_func(g_package_red,g_package_green,g_package_blue);
						}else if(g_package_cmd==KEY_BREATH_RGB_MODE_CMD){//RGB呼吸模式
							led_off_func();
							led_control.led_state=LED_RGB_BREATH_STATE;
						}
					}
				}
			}
			pre_package_cmd=g_package_cmd;//保存命令
			pre_package_seq=g_package_seq;//保存序列号
		}
	}
}
