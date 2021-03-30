//#include "../../common.h"
#include "../../drivers.h"
#include "../../user_drivers.h"
#include "frame.h"
#include "pairing_op.h"
/***********************************************************
 * �������ܣ�ID������ѯƥ��
 * ��       ����pid   ң����ID
 *        grp   ���
 * �� ��  ֵ��ƥ��ɹ�������1�����ɹ�������0
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
 * �������ܣ�дID�����eeprom
 * ��       ����position   ��Ӧλ�õ�ƫ����
 * �� ��  ֵ��
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
 * �������ܣ�����
 * ��       ����
 * �� ��  ֵ��
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
 * �������ܣ�����id
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void pair_id_save_func(void)
{
	unsigned char i;
	unsigned char temp;
	for(i=0;i<MAX_PAIRED_REMOTER;i++){
		if(remote_save_pid==led_control.pared_remote[i].pid){//IDƥ��
			if(led_control.pared_remote[i].group_id!=remote_save_grp){//���ƥ��
				write_position_detect(i);//����
				return;
			}else//���ѱ��棬�򷵻�
				return;
		}
	}

	if(i==MAX_PAIRED_REMOTER){//�ѱ���������û��ƥ���
		temp=led_control.paire_index;//������±�
		led_control.paire_index++;
		if(led_control.paire_index>=MAX_PAIRED_REMOTER)//�Ƿ񳬹���󱣴�ֵ
			led_control.paire_index=0;//������Ĭ��Ϊ0
		write_position_detect(temp);//����
	}
}
