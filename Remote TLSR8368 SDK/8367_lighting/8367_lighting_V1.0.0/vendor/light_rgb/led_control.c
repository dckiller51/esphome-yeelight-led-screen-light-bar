//#include "../../common.h"
#include "../../drivers.h"
#include "../../user_drivers.h"
#include "led_control.h"
#include "frame.h"
#include "state.h"
#include "led_rgb.h"
#include "led_yl.h"
#include "pairing_op.h"

unsigned int led_yl_updata_tick;     //色温灯更新时间点
unsigned int led_rgb_updata_tick;    //RGB灯更新时间点
unsigned char led_rgb_breath_state;  //呼吸模式所处的状态
const unsigned short breath_value[14][3]={   //呼吸模式各个灯的亮度值分别为红、绿、蓝
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
 * 函数功能：LED参数初始化
 * 参       数：
 * 返 回  值：
 **********************************************************/
void led_para_init_func(void)
{
	unsigned char i;
	unsigned char *Pr=(void *)&led_control;
	for(unsigned char i=0;i<sizeof(LED_Control_Info_t);i++)//读取eeprom保存的数据
		*Pr++=fm24c02_read_func(i);

	if(led_control.paire_index >= MAX_PAIRED_REMOTER)//若超过最大值，则默认为0
		led_control.paire_index = 0;

	if(led_control.luminance_index > MAX_LUMINANCE_INDEX)//超过最大亮度值，默认为最大值
		led_control.luminance_index = MAX_LUMINANCE_INDEX;

	if(led_control.chroma_index > MAX_CHROME_INDEX)//超过最大色温值，默认为最大值
		led_control.chroma_index = MAX_CHROME_INDEX;

	if(led_control.led_state>=LED_LAST_STATE)//默认为色温灯开
		led_control.led_state=LED_YL_ON_STATE;

	for(i=0;i<3;i++){//设置RGB默认的最大
		if(led_control.rgb_value[i]>MAX_VALUE_RGB)
			led_control.rgb_value[i]=MAX_VALUE_RGB;
	}
}
/***********************************************************
 * 函数功能：LED初始化
 * 参       数：
 * 返 回  值：
 **********************************************************/
void led_init_func(void)
{
	led_para_init_func();
	if((led_control.led_state==LED_YL_ON_STATE)||(led_control.led_state==LED_OFF_STATE)){//灯状态为色温灯开，为关时默认为色温灯开模式
		led_on_func();
	}else if(led_control.led_state==LED_RGB_ON_STATE){//RGB状态，若为呼吸灯模式，则会自动触发
		led_on_rgb_func();
	}
}
/***********************************************************
 * 函数功能：LED闪烁模式
 * 参       数：Cnt  闪烁的次数
 * 返 回  值：
 **********************************************************/
void led_flash_updata_func(unsigned char Cnt)
{
	led_flash_cnt=Cnt;
}
/***********************************************************
 * 函数功能：灯的处理程序
 * 参       数：
 * 返 回  值：
 **********************************************************/
void led_pask_process_func(void)
{
	if(led_flash_cnt){//是否有闪烁
		if(timeout_us(led_flash_tick,500000)){//是否到达闪烁时间
			led_flash_tick=get_sys_tick();
			if(led_control.led_state==LED_YL_ON_STATE){//开灯状态结束，关灯
				led_control.led_state=LED_OFF_STATE;
				led_off_func();
			}else if(led_control.led_state==LED_OFF_STATE){//关灯状态结束，开灯
				led_control.led_state=LED_YL_ON_STATE;
				led_on_func();
				if(led_flash_cnt!=0xff)//若闪烁次数设为0xff，则表示无限次
					led_flash_cnt--;
			}
		}
	}

	if(led_state_change_flag){//色温灯有状态更新
		if(timeout_us(led_yl_updata_tick,3000)){//每3ms更新一次
			led_yl_updata_tick=get_sys_tick();
			if(led_lumina_cur!=led_lumina_target){//亮度有变化
				led_lumina_cur=lumina_one_step_updata(led_lumina_target,led_lumina_cur);
			}

			if(led_chroma_cur!=led_chroma_target){//色温有变化
				led_chroma_cur=chroma_one_step_updata(led_chroma_target,led_chroma_cur);
			}

			led_pwm_control_func(led_lumina_cur,led_chroma_cur);//设置LED
			if( (led_chroma_cur==led_chroma_target) && (led_lumina_cur==led_lumina_target) ){//变化完成
				led_state_change_flag=0;
				save_led_state_info_func();//保存状态
			}
		}
	}

	if(led_rgb_state_change_flag){//RGB状态更新
		if(timeout_us(led_rgb_updata_tick,3000)){//每3ms更新一次
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

			led_set_rgb_power_func(led_red_cur,led_green_cur,led_blue_cur);//设置PWM值
			if(led_red_cur==led_red_target&&led_green_cur==led_green_target&&led_blue_cur==led_blue_target){//更新完成
				led_rgb_state_change_flag=0;
			}
		}
	}

	if(led_control.led_state==LED_RGB_BREATH_STATE){//呼吸模式
		if(led_rgb_state_change_flag==0){//状态更新是否结束
			led_rgb_breath_state++;//结束后进入下一状态
			if(led_rgb_breath_state>13)
				led_rgb_breath_state=0;
			led_red_target=breath_value[led_rgb_breath_state][0];
			led_green_target=breath_value[led_rgb_breath_state][1];
			led_blue_target=breath_value[led_rgb_breath_state][2];
			led_rgb_state_change_flag=1;//状态更新标志
		}
	}
}
