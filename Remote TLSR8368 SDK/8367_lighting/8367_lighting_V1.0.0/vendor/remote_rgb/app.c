#include "app_config.h"

#include "../../drivers.h"
#include "frame.h"
#include "key_def.h"
#include "rf_control.h"

#define  DEBUG          1
#define LED_PIN         GPIO_SWSC7
unsigned long firmwareVersion;
const unsigned int gpio_row[]={GPIO_PC2,GPIO_PC3,GPIO_PC1,GPIO_PB0,GPIO_PD2};//矩阵行的IO
const unsigned int gpio_column[]={GPIO_PB3,GPIO_PB4,GPIO_PB5};//矩阵列的IO
#if 0
const unsigned char key_table[4][4] = {//按键表格
		{KEY_ON_CMD<<4,             KEY_CHROME_DEC_CMD<<4,  KEY_PAIRE_CODE_CMD<<4,      KEY_CLEAR_CODE_CMD<<4 },
		{KEY_LUMINANCE_INC_CMD<<4,  KEY_NONE_CMD,           KEY_LUMINANCE_DEC_CMD<<4,   KEY_NONE_CMD          },
		{KEY_NONE_CMD,              KEY_NONE_CMD,           KEY_NONE_CMD,               KEY_NONE_CMD          },
		{KEY_OFF_CMD<<4,            KEY_CHROME_INC_CMD<<4,  KEY_BREATH_RGB_MODE_CMD<<4, KEY_NIGHT_CMD<<4      },
};
#else
//const unsigned char key_table[4][4] = {
//		{KEY_BREATH_RGB_MODE_CMD<<4,  KEY_LUMINANCE_DEC_CMD<<4,  KEY_NONE_CMD,           KEY_NONE_CMD},
//		{KEY_NIGHT_CMD<<4,            KEY_CLEAR_CODE_CMD<<4,     KEY_NONE_CMD,           KEY_NONE_CMD},
//		{KEY_CHROME_DEC_CMD<<4,       KEY_PAIRE_CODE_CMD<<4,     KEY_CHROME_INC_CMD<<4,  KEY_NONE_CMD},
//		{KEY_ON_CMD<<4,               KEY_LUMINANCE_INC_CMD<<4,  KEY_OFF_CMD<<4,         KEY_NONE_CMD},
//};
const unsigned char key_table[5][3] = {//按键表格
		{KEY_LUMINANCE_DEC_CMD<<4,  KEY_CLEAR_CODE_CMD<<4,      KEY_PAIRE_CODE_CMD<<4  },
		{KEY_CHROME_DEC_CMD<<4,     KEY_BREATH_RGB_MODE_CMD<<4, KEY_NIGHT_CMD<<4       },
		{KEY_NONE_CMD,              KEY_NONE_CMD,               KEY_NONE_CMD           },
		{KEY_NONE_CMD,              KEY_NONE_CMD,               KEY_CHROME_INC_CMD<<4  },
		{KEY_OFF_CMD<<4,            KEY_LUMINANCE_INC_CMD<<4,   KEY_ON_CMD<<4          }
};
#endif
unsigned int key_down_cnt;
unsigned int key_up_cnt;
unsigned int key_lumi_chroma_cnt;
unsigned char pre_key;
unsigned char loop;
/*******************************************************************
 * 函数功能：GPIO初始化
 * 参       数：
 * 返 回 值：
 ******************************************************************/
void gpio_init_func(void)
{
	unsigned char i;
	for(i=0;i<5;i++){
		gpio_set_func(gpio_row[i],AS_GPIO);     //IO设为普通IO
		gpio_set_output_en(gpio_row[i],LEVEL_LOW); //输出使能关掉
		gpio_set_input_en(gpio_row[i],LEVEL_HIGH);  //输入使能关掉
		gpio_write(gpio_row[i],LEVEL_LOW);        //IO输出设为低电平
		gpio_set_up_down_resistor(gpio_row[i],GPIO_PULL_NONE);        //IO设为悬浮状态
	}

	for(i=0;i<3;i++){
		gpio_set_func(gpio_column[i],AS_GPIO);        //IO设为普通IO
		gpio_set_output_en(gpio_column[i],LEVEL_LOW);    //输出使能关掉
		gpio_set_input_en(gpio_column[i],LEVEL_HIGH);       //使能输入
		gpio_set_up_down_resistor(gpio_column[i],GPIO_PULL_UP_1M);          //设置上拉1M电阻
		gpio_write(gpio_column[i],LEVEL_LOW);           //输出设为0
		pm_set_gpio_wakeup(gpio_column[i],LEVEL_LOW,1);       //设置IO低电平唤醒，第一参数为IO，第而参数为唤醒电平，第三参数为使能
	}

	gpio_set_func(LED_PIN,AS_GPIO);        //IO设为普通IO
	gpio_set_output_en(LED_PIN,LEVEL_HIGH);    //输出使能关掉
	gpio_set_input_en(LED_PIN,LEVEL_LOW);       //使能输入
	gpio_write(LED_PIN,1);
	gpio_set_up_down_resistor(LED_PIN,GPIO_PULL_NONE);        //IO设为悬浮状态
}
/*******************************************************************
 * 函数功能：进入deepsleep前设置参数
 * 参       数：
 * 返 回 值：
 ******************************************************************/
void set_wakeup_func(void)
{
	unsigned char i;
	for(i=0;i<5;i++)
		gpio_set_up_down_resistor(gpio_row[i],GPIO_PULL_DN_100K);      //设置行IO下拉100K，当按键按下时，列IO为低电平，则可唤醒MCU
//	analog_write(0x3a, current_active_group);         //进入deepsleep前保存组别值
	analog_write(0x3b, led_remote.pkt_seq);         //进入deepsleep前保存包的序列号
	gpio_write(LED_PIN,0);
}
/*******************************************************************
 * 函数功能：发送数据包的数据初始化
 * 参       数：
 * 返 回 值：
 ******************************************************************/
void package_data_init_func(void)
{
	led_remote.dma_len=sizeof(LED_Package_t)-sizeof(led_remote.dma_len);
	led_remote.rf_len=led_remote.dma_len-1;
	led_remote.rf_len1=led_remote.dma_len-2;
	led_remote.vid=0x5453;
//	led_remote.pid=0x87654321;
	led_remote.pid = otp_read(PID_ADDR) | otp_read(PID_ADDR+1)<<8 | otp_read(PID_ADDR+2)<<16 | otp_read(PID_ADDR+3)<<24;
	led_remote.pkt_seq=analog_read(0x3b);
}
/*******************************************************************
 * 函数功能：按键扫描
 * 参       数：
 * 返 回 值：返回对应的按键值
 ******************************************************************/
unsigned char remote_key_scan_func(void)
{
	unsigned char i,j;
	for(i=0;i<5;i++){
		gpio_set_up_down_resistor(gpio_row[i],GPIO_PULL_DN_100K);//设置行IO为下拉100K
		delay_us(10);
		for(j=0;j<3;j++){
			if(gpio_read(gpio_column[j])==0){//有按键按下
				gpio_set_up_down_resistor(gpio_row[i],GPIO_PULL_NONE);//悬浮
				return key_table[i][j];//查表，返回表值
			}
		}
		gpio_set_up_down_resistor(gpio_row[i],GPIO_PULL_NONE);//若无相应按键按下，则悬浮
		delay_us(10);
	}
	return 0;
}

void user_init(void)
{
	gpio_init_func();
	rf_init_func();
	package_data_init_func();
	irq_enable();
}

void main_loop(void)
{
	unsigned char Cur_key;
	Cur_key=remote_key_scan_func();
	loop++;
	loop&=0x7;
	if(Cur_key){
		if(loop==0)
			gpio_write(LED_PIN,0);
		else if(loop == 4)
			gpio_write(LED_PIN,1);
		if(Cur_key!=pre_key){
			key_down_cnt++;
			key_up_cnt=0;
			if(key_down_cnt>4){
				key_down_cnt=0;
				key_lumi_chroma_cnt=0;
				pre_key=Cur_key;
				led_remote.pkt_seq++;
			}
		}else{
			led_remote.key_control=pre_key;
			if((pre_key>>4)==KEY_LUMINANCE_INC_CMD||(pre_key>>4)==KEY_LUMINANCE_DEC_CMD||(pre_key>>4)==KEY_CHROME_INC_CMD||(pre_key>>4)==KEY_CHROME_DEC_CMD){
				key_lumi_chroma_cnt++;
				if(key_lumi_chroma_cnt&0x20){
					key_lumi_chroma_cnt=0;
					led_remote.pkt_seq++;
				}
			}
		}
	}else{
		key_down_cnt=0;
		key_up_cnt++;
		key_lumi_chroma_cnt=0;
		gpio_write(LED_PIN,0);
		if(key_up_cnt>20){
#if 0
			delay_us(100000);
#else
			set_wakeup_func();
			pm_sleep_wakeup(DEEPSLEEP_MODE,PM_WAKEUP_PAD,0);
#endif

		}
	}
	send_package_data_func();
#if 0
	delay_us(10000);
#else
	pm_sleep_wakeup(SUSPEND_MODE,PM_WAKEUP_TIMER,get_sys_tick()+10*CLOCK_SYS_CLOCK_1MS);//进入suspend 10ms
#endif
}
