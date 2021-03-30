//#include "../../common.h"
#include "../../drivers.h"
#include "../../user_drivers.h"
#include "frame.h"
#include "pairing_op.h"
/***********************************************************
 * 函数功能：ID及组别查询匹配
 * 参       数：pid   遥控器ID
 *        grp   组别
 * 返 回  值：匹配成功，返回1，不成功，返回0
 **********************************************************/
unsigned char paired_ID_match(unsigned int pid,unsigned char grp)
{
	unsigned char i;
	for(i=0;i<MAX_PAIRED_REMOTER;i++){
		if(pid==led_control.pared_remote[i].pid)
			if((grp==led_control.pared_remote[i].group_id)||grp==0xf)
				return 1;
	}
	return 0;
}
/***********************************************************
 * 函数功能：写ID及组别到eeprom
 * 参       数：position   相应位置的偏移量
 * 返 回  值：
 **********************************************************/
void write_position_detect(unsigned int position)
{
	char *ptr;
	led_control.pared_remote[position].group_id = remote_save_grp;
	led_control.pared_remote[position].pid = remote_save_pid;
	ptr = (char*)&led_control.pared_remote[position];
	for(unsigned int j=0;j<sizeof(Pairing_info_t);j++){
		fm24c02_write_func(EEPROM_INFO_START + position*sizeof(Pairing_info_t)+j, *(ptr+j));
	}
	fm24c02_write_func(EEPROM_ID_INDEX_ADDR,led_control.paire_index);
}
/***********************************************************
 * 函数功能：清码
 * 参       数：
 * 返 回  值：
 **********************************************************/
void clear_pared_code_func(void)
{
	unsigned char i;
	for(i=0;i<MAX_PAIRED_REMOTER;i++){
		led_control.pared_remote[i].pid=0xffff;
		led_control.pared_remote[i].group_id=0;
	}
	led_control.paire_index=0;
	led_control.power_on_recover=0;
	led_para_save_func();
}
/***********************************************************
 * 函数功能：保存id
 * 参       数：
 * 返 回  值：
 **********************************************************/
void pair_id_save_func(void)
{
	unsigned char i;
	unsigned char temp;
	for(i=0;i<MAX_PAIRED_REMOTER;i++){
		if(remote_save_pid==led_control.pared_remote[i].pid){//ID匹配
			if(led_control.pared_remote[i].group_id!=remote_save_grp){//组别匹配
				write_position_detect(i);//保存
				return;
			}else//若已保存，则返回
				return;
		}
	}

	if(i==MAX_PAIRED_REMOTER){//已保存数据中没有匹配的
		temp=led_control.paire_index;//保存的下标
		led_control.paire_index++;
		if(led_control.paire_index>=MAX_PAIRED_REMOTER)//是否超过最大保存值
			led_control.paire_index=0;//超过后默认为0
		write_position_detect(temp);//保存
	}
}
