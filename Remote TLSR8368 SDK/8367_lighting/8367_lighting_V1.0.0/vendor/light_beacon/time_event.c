//#include "../../common.h"
#include "../../drivers.h"
#include "rf_control.h"

unsigned int channel_change_tick;
/***********************************************************
 * �������ܣ�ʱ�䴦����
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void time_event_process_func(void)
{
	if(timeout_us(channel_change_tick,20000)){//ÿ20ms��һ��Ƶ
		channel_change_tick=get_sys_tick();
		rf_change_channel_func();
	}
}
