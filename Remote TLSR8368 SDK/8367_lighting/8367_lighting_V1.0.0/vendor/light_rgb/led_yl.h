#pragma once

#define MAX_VALUE_RGB        1000      //RGB最大级数
#define MAX_LUMINANCE        1000      //亮度的级数
#define MAX_LUMINANCE_INDEX  13        //亮度的级数
#define MAX_CHROME           100       //亮度值
#define MAX_CHROME_INDEX     10
#define LOW_LIGHT_LUMINACE   20        //小夜灯的值


unsigned short led_lumina_cur;
unsigned short led_lumina_target;
unsigned short led_chroma_cur;
unsigned short led_chroma_target;
unsigned char led_state_change_flag;
unsigned int led_change_tick;


void led_on_func(void);
void led_off_func(void);
void led_event_proc_func(unsigned char Cmd);
void led_set_lumi_chrome_func(unsigned short Lumi,unsigned short Chroma);
void led_pwm_control_func(int Lumina, int Chroma);
unsigned short lumina_one_step_updata(unsigned short target,unsigned short cur);
unsigned short chroma_one_step_updata(unsigned short target,unsigned short cur);
