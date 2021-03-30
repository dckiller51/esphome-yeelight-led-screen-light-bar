//#include "../../common.h"
#include "../../drivers.h"
#include "../../user_drivers.h"
#include "user_pwm.h"
#include "frame.h"
#include "led.h"

unsigned short led_lumina_cur;
unsigned short led_lumina_target;
unsigned short led_chroma_cur;
unsigned short led_chroma_target;
unsigned char led_state_change_flag;
unsigned int led_change_tick;

unsigned char led_flash_cnt;
unsigned int  led_flash_tick;

const unsigned short led_luminance_value[MAX_LUMINANCE_INDEX+1]={40,67,102,144,193,250,313,384,462,547,640,740,846,1000};
const unsigned char  led_chroma_value[MAX_CHROME_INDEX+1]={0,10,20,30,40,50,60,70,80,90,100};
/***********************************************************
 * 函数功能：LED参数初始化
 * 参       数：
 * 返 回  值：
 **********************************************************/
void led_para_init_func(void)
{
	unsigned char *Pr=(void *)&led_control;
	for(unsigned char i=0;i<sizeof(LED_control_info_t);i++)//读取eeprom保存的数据
		*Pr++=fm24c02_read_func(i);

	if(led_control.paire_index >= MAX_PAIRED_REMOTER)//若超过最大值，则默认为0
		led_control.paire_index = 0;

	if(led_control.luminance_index > MAX_LUMINANCE_INDEX)//超过最大亮度值，默认为最大值
		led_control.luminance_index = MAX_LUMINANCE_INDEX;

	if(led_control.chroma_index > MAX_CHROME_INDEX)//超过最大色温值，默认为最大值
		led_control.chroma_index = MAX_CHROME_INDEX;

	if(led_control.seg_index > 3)//超过最大切换状态值，默认为0
		led_control.seg_index = 0;
}
/***********************************************************
 * 函数功能：保存参数
 * 参       数：
 * 返 回  值：
 **********************************************************/
void led_para_save_func(void)
{
	unsigned char *Pr=(void *)&led_control;
	for(unsigned char i=0;i<sizeof(LED_control_info_t);i++)
		fm24c02_write_func(i,*Pr++);
}
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
 * 函数功能：开灯
 * 参       数：
 * 返 回  值：
 **********************************************************/
void led_on_func(void)
{
	led_state_change_flag=1;
	led_lumina_target=led_luminance_value[led_control.luminance_index];//设置亮度目标值
	led_chroma_target=led_chroma_value[led_control.chroma_index];//设置色温目标值
	if(led_control.led_on==0){//若当前状态为关灯
		led_lumina_cur=0;//亮度当前值为0
		led_chroma_cur=led_chroma_target;//色温当前值为0
		led_control.led_on=1;//led状态为开灯状态
	}
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
	led_control.led_on=0;
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
 * 函数功能：LED初始化
 * 参       数：
 * 返 回  值：
 **********************************************************/
void led_init_func(void)
{
	relay_pkt.dma_len=sizeof(rf_packet_led_remote_t)-sizeof(relay_pkt.dma_len);
	relay_pkt.rf_len = relay_pkt.dma_len-1;
	relay_pkt.rf_len1 = relay_pkt.dma_len - 2;
	relay_pkt.vid=0x5453;
	led_para_init_func();
	led_lumina_cur=0;
	led_chroma_cur=led_chroma_value[led_control.chroma_index];
	if(led_control.power_on_recover==0xff){//当为0xff时，切换状态
		led_control.seg_index++;
		led_control.seg_index&=3;
		led_luminace_segment_set_func(led_control.seg_index);//设置状态
	}else{//不切换状态时，正常开灯
		led_on_func();
	}
	led_control.power_on_recover=0xff;
	led_para_save_func();//保存状态
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
 * 函数功能：设置LED
 * 参       数：Lumina  亮度值
 *        Chroma  色温值
 * 返 回  值：
 **********************************************************/
void led_power_control_func(int Lumina, int Chroma)
{
	unsigned char i;
	led_lumina_target = led_lumina_cur = Lumina;
	led_chroma_target = led_chroma_cur = Chroma;
	for(i=0;i<(MAX_LUMINANCE_INDEX+1);i++)//查找亮度对应的下标值
		if(Lumina<=led_luminance_value[i])
			break;
	led_control.luminance_index = i;
	led_control.chroma_index = (Chroma+5)/10;//色温一级为10
	led_pwm_control_func(Lumina, Chroma);

}
/***********************************************************
 * 函数功能：设置led的4种状态
 * 参       数：seg_index ：对应的状态
 * 返 回  值：
 **********************************************************/
void led_luminace_segment_set_func(unsigned char seg_index)
{

	switch(seg_index){
		case 0:
//			100% 0%
			led_power_control_func(MAX_LUMINANCE, 100);
			break;
		case 1:
//			0%  100%
			led_power_control_func(MAX_LUMINANCE, 0);
			break;
		case 2:
//			50%  50%
			led_power_control_func(MAX_LUMINANCE, 50);
			break;
		case 3:
//			0.4%  50%
			led_power_control_func(LOW_LIGHT_LUMINACE, 50);
			break;
	}
}
/***********************************************************
 * 函数功能：遥控器命令执行
 * 参       数：
 * 返 回  值：
 **********************************************************/
void led_event_proc_func(unsigned char Cmd)
{
	if(led_flash_cnt)return;
	if(Cmd==LED_LUMINANCE_INC_CMD||Cmd==LED_LUMINANCE_DEC_CMD||Cmd==LED_CHROME_INC_CMD||Cmd==LED_CHROME_DEC_CMD)
		if(led_control.led_on==0)  //关灯状态不进行色温亮度调节
			return;

	switch(Cmd){
		case LED_NONE_CMD:

			break;

		case LED_ON_CMD://开灯
			led_on_func();
			break;

		case LED_OFF_CMD://关灯
			led_off_func();
			break;

		case LED_LUMINANCE_INC_CMD://亮度加
			led_updata_luminance_func(1);
			break;

		case LED_LUMINANCE_DEC_CMD://亮度减
			led_updata_luminance_func(0);
			break;

		case LED_CHROME_INC_CMD://色温加
			led_updata_chroma_func(1);
			break;

		case LED_CHROME_DEC_CMD://色温减
			led_updata_chroma_func(0);
			break;

		case LED_NIGHT_CMD://夜灯模式
			led_updata_lumi_chrome_func(LOW_LIGHT_LUMINACE,50);
			break;

		default:

			break;
	}
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
 * 函数功能：闪烁次数更新
 * 参       数：Flash_cnt  闪烁的次数
 * 返 回  值：
 **********************************************************/
void led_flash_updata(unsigned char Flash_cnt)
{
	led_flash_cnt=Flash_cnt;
}
/***********************************************************
 * 函数功能：LED任务处理
 * 参       数：
 * 返 回  值：
 **********************************************************/
void led_task_process_func(void)
{
	if(led_flash_cnt){//led是否有闪烁
		if(timeout_us(led_flash_tick,500000)){//500ms更新一次状态
			led_flash_tick=get_sys_tick();
			if(led_control.led_on==0){
				led_control.led_on=1;
				led_power_control_func(500,50);
				if(led_flash_cnt!=0xff)//为0xff时，无限次闪烁
					led_flash_cnt--;
			}else{
				led_control.led_on=0;
				led_power_control_func(0,50);
			}
		}
	}

	if(led_state_change_flag){//LED状态有变化
		if(timeout_us(led_change_tick,5000)){//每5ms更新一次
			led_change_tick=get_sys_tick();
			if(led_lumina_cur!=led_lumina_target){//亮度有变化
				led_lumina_cur=lumina_one_step_updata(led_lumina_target,led_lumina_cur);
			}

			if(led_chroma_cur!=led_chroma_target){//色温有变化
				led_chroma_cur=chroma_one_step_updata(led_chroma_target,led_chroma_cur);
			}

			led_pwm_control_func(led_lumina_cur,led_chroma_cur);//设置LED
			if( (led_chroma_cur==led_chroma_target) && (led_lumina_cur==led_lumina_target) ){//变化完成
				led_state_change_flag=0;
				led_control.power_on_recover=0;
				led_para_save_func();//保存状态
			}
		}
	}
}
