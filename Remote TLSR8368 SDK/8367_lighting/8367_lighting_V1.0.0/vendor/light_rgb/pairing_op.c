//#include "../../common.h"
#include "../../drivers.h"
#include "../../user_drivers.h"
#include "frame.h"
/***********************************************************
 * �������ܣ���IDֵд��eeprom
 * ��       ����Position   ID���ڵ��±�ֵ
 *        Id         д���idֵ
 * �� ��  ֵ��
 **********************************************************/
void write_id_direct(unsigned char Position,unsigned int Id)
{
	if(Position<MAX_PAIRED_REMOTER){
		fm24c02_write_func(REMOTE_ID_ADDR+Position*4,Id);
		fm24c02_write_func(REMOTE_ID_ADDR+Position*4+1,Id>>8);
		fm24c02_write_func(REMOTE_ID_ADDR+Position*4+2,Id>>16);
		fm24c02_write_func(REMOTE_ID_ADDR+Position*4+3,Id>>24);
	}
}
/***********************************************************
 * �������ܣ�������д�뵽eeprom
 * ��       ����Addr     eeprom�ĵ�ַ
 *       Data      д�������ֵ
 * �� ��  ֵ��
 **********************************************************/
void write_data_direct_func(unsigned char Addr,unsigned char Data)
{
	fm24c02_write_func(Addr,Data);
}
/***********************************************************
 * �������ܣ�����ң����ID
 * ��       ����Id    �����Idֵ
 * �� ��  ֵ���ɹ�����1  ���ɹ�����0
 **********************************************************/
unsigned char save_remote_ID_func(unsigned int Id)
{
	unsigned char i;
	for(i=0;i<MAX_PAIRED_REMOTER;i++){
		if(Id==led_control.remote_id[i])
			return 1;
	}
	write_id_direct(led_control.paire_index,Id);
	led_control.remote_id[led_control.paire_index]=Id;
	led_control.paire_index++;
	if(led_control.paire_index>=MAX_PAIRED_REMOTER)
		led_control.paire_index=0;
	write_data_direct_func(PAIRE_INDEX_ADDR,led_control.paire_index);
	return 1;
}
/***********************************************************
 * �������ܣ���������ң����IDֵ
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void clear_remote_ID_func(void)
{
	unsigned char i;
	for(i=0;i<MAX_PAIRED_REMOTER;i++){
		led_control.remote_id[i]=0xffffffff;
		write_id_direct(i,0xffffffff);
	}
	led_control.paire_index=0;
	write_data_direct_func(PAIRE_INDEX_ADDR,led_control.paire_index);
}
/***********************************************************
 * �������ܣ���ѯID�Ƿ���ѱ����idƥ��
 * ��       ����
 * �� ��  ֵ��ƥ��ɹ�����1  ���ɹ�����0
 **********************************************************/
unsigned char paired_ID_match(unsigned int Id)
{
	unsigned char i;
	for(i=0;i<MAX_PAIRED_REMOTER;i++){
		if(Id==led_control.remote_id[i])
			return 1;
	}
	return 0;
}
/***********************************************************
 * �������ܣ�����ƵĿ�����Ϣ
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void save_led_control_info_func(void)
{
	unsigned char *Ptr=(void *)&led_control;
	unsigned char i;
	for(i=0;i<sizeof(LED_Control_Info_t);i++){
		fm24c02_write_func(i,*Ptr++);
	}
}
/***********************************************************
 * �������ܣ�����Ƶ�״̬��Ϣ
 * ��       ����
 * �� ��  ֵ��
 **********************************************************/
void save_led_state_info_func(void)
{
	unsigned char *Ptr=(void *)&led_control.luminance_index;
	unsigned char i;
	unsigned char Len=sizeof(LED_Control_Info_t)-MAX_PAIRED_REMOTER-1;

	for(i=0;i<Len;i++){
		fm24c02_write_func(LUMI_INDEX_ADDR+i,*Ptr++);
	}
}
