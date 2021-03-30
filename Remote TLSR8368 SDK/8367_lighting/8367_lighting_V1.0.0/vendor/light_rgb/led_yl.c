//#include "../../common.h"
#include "../../drivers.h"
#include "../../user_drivers.h"
#include "led_yl.h"
#include "state.h"
#include "frame.h"
#include "key_def.h"
#include "user_pwm.h"

const unsigned short led_luminance_value[MAX_LUMINANCE_INDEX+1]={40,67,102,144,193,250,313,384,462,547,640,740,846,1000};
const unsigned char  led_chroma_value[MAX_CHROME_INDEX+1]={0,10,20,30,40,50,60,70,80,90,100};

/***********************************************************
 * 函数功能：色温亮度值状态更新
 * 参       数：
 * 返 回  值：
 **********************************************************/
void led_updata_lumi_chrome_func(unsigned short Lumi,unsigned char Chroma)
{
	led_lumina_target=Lumi;
	led_chroma_target=Chroma;
	led_state_change_flag=1;
}
/***********************************************************
 * 函数功能：设置LED的PWM
 * 参       数：Lumina   亮度值
 *        Chroma   色温值
 * 返 回  值：
 **********************************************************/
void led_pwm_control_func(int Lumina, int Chroma)
{
	int Whrite_pwm_val, Yellow_pwm_val;

	Whrite_pwm_val = Lumina * Chroma/100;//白灯的占空比，亮度值*比例
	Yellow_pwm_val = Lumina - Whrite_pwm_val;//黄灯的占空比

	PWM_DutyValueSet(PWM4, Whrite_pwm_val);
	PWM_DutyValueSet(PWM0, Yellow_pwm_val);
}
/***********************************************************
 * 函数功能：计算当前亮度值
 * 参       数：target   亮度目标值
 *        cur      当前亮度值
 * 返 回  值：计算后的亮度值
 **********************************************************/
unsigned short lumina_one_step_updata(unsigned short target,unsigned short cur)
{
	unsigned short temp=cur>>6;//亮度值越大，变化就越大
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
 * 函数功能：计算色温值
 * 参       数：target   亮度目标值
 *        cur      当前亮度值
 * 返 回  值：计算后的色温值
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
 * 函数功能：开灯
 * 参       数：
 * 返 回  值：
 **********************************************************/
void led_on_func(void)
{
	if(led_control.led_state!=LED_YL_ON_STATE){//若当前状态为关灯
		led_lumina_cur=0;//亮度当前值为0
		led_chroma_cur=0;//色温当前值为0
		led_control.led_state=LED_YL_ON_STATE;//led状态为开灯状态
	}
	led_state_change_flag=1;
	led_lumina_target=led_luminance_value[led_control.luminance_index];//设置亮度目标值
	led_chroma_cur=led_chroma_target=led_chroma_value[led_control.chroma_index];//设置色温目标值
}
/***********************************************************
 * 函数功能：LED关灯
 * 参       数：
 * 返 回  值：
 **********************************************************/
void led_off_func(void)
{
	led_state_change_flag=1;
	led_lumina_target=0;
//	led_chroma_target=0;
	led_control.led_state=LED_OFF_STATE;
}
/***********************************************************
 * 函数功能：更新亮度
 * 参       数：Type   更新类型 1为加  0为减
 * 返 回  值：
 **********************************************************/
void led_updata_luminance_func(unsigned char Type)
{
	unsigned short ChromaValue;
	unsigned short LuminaceValue;
	if(Type){//为1，则加
		led_control.luminance_index++;
	}else{
		if(led_control.luminance_index)//不为0则减，为0则保存不变
			led_control.luminance_index--;
	}
	if(led_control.luminance_index>MAX_LUMINANCE_INDEX)//亮度超过最大值，则默认为最大值
		led_control.luminance_index=MAX_LUMINANCE_INDEX;
	ChromaValue=led_chroma_value[led_control.chroma_index];
	LuminaceValue=led_luminance_value[led_control.luminance_index];
	led_updata_lumi_chrome_func(LuminaceValue,ChromaValue);
}
/***********************************************************
 * 函数功能：更新色温
 * 参       数：Type   更新类型 1为加  0为减
 * 返 回  值：
 **********************************************************/
void led_updata_chroma_func(unsigned char Type)
{
	unsigned short ChromaValue;
	unsigned short LuminaceValue;
	if(Type){//为1，则加
		led_control.chroma_index++;
	}else{
		if(led_control.chroma_index)//不为0则减，为0保持不变
			led_control.chroma_index--;
	}
	if(led_control.chroma_index>MAX_CHROME_INDEX)//超过最大值，则默认为最大值
		led_control.chroma_index=MAX_CHROME_INDEX;
	ChromaValue=led_chroma_value[led_control.chroma_index];
	LuminaceValue=led_luminance_value[led_control.luminance_index];
	led_updata_lumi_chrome_func(LuminaceValue,ChromaValue);//更新状态
}
/***********************************************************
 * 函数功能：设置色温亮度值
 * 参       数：
 * 返 回  值：
 **********************************************************/
void led_set_lumi_chrome_func(unsigned short Lumi,unsigned short Chroma)
{
	unsigned char i;
	if(Lumi>led_luminance_value[MAX_LUMINANCE_INDEX])//超过最大值，默认为最大
		Lumi=led_luminance_value[MAX_LUMINANCE_INDEX];
	if(Chroma>led_chroma_value[MAX_CHROME_INDEX])//超过最大值，默认为最大
		Chroma=led_chroma_value[MAX_CHROME_INDEX];
	led_control.chroma_index=Chroma/10;
	for(i=0;i<(MAX_LUMINANCE_INDEX+1);i++)
		if(Lumi<=led_luminance_value[i])
			break;
	led_control.luminance_index=i;
	led_updata_lumi_chrome_func(Lumi,Chroma);
}
/***********************************************************
 * 函数功能：遥控器命令执行
 * 参       数：
 * 返 回  值：
 **********************************************************/
void led_event_proc_func(unsigned char Cmd)
{
	if(Cmd==KEY_LUMINANCE_INC_CMD||Cmd==KEY_LUMINANCE_DEC_CMD||Cmd==KEY_CHROME_INC_CMD||Cmd==KEY_CHROME_DEC_CMD)
		if(led_control.led_state==LED_OFF_STATE)  //关灯状态不进行色温亮度调节
			return;

	switch(Cmd){
		case KEY_NONE_CMD:

			break;

		case KEY_ON_CMD://开灯
			led_on_func();
			break;

		case KEY_OFF_CMD://关灯
			led_off_func();
			break;

		case KEY_LUMINANCE_INC_CMD://亮度加
			led_updata_luminance_func(1);
			break;

		case KEY_LUMINANCE_DEC_CMD://亮度减
			led_updata_luminance_func(0);
			break;

		case KEY_CHROME_INC_CMD://色温加
			led_updata_chroma_func(1);
			break;

		case KEY_CHROME_DEC_CMD://色温减
			led_updata_chroma_func(0);
			break;

		case KEY_NIGHT_CMD://夜灯模式
			led_updata_lumi_chrome_func(LOW_LIGHT_LUMINACE,50);
			break;

		default:

			break;
	}
}
