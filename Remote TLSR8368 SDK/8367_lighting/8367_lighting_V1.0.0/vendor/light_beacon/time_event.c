//#include "../../common.h"
#include "../../drivers.h"
#include "rf_control.h"

unsigned int channel_change_tick;
/***********************************************************
 * 函数功能：时间处理函数
 * 参       数：
 * 返 回  值：
 **********************************************************/
void time_event_process_func(void)
{
	if(timeout_us(channel_change_tick,20000)){//每20ms跳一次频
		channel_change_tick=get_sys_tick();
		rf_change_channel_func();
	}
}
