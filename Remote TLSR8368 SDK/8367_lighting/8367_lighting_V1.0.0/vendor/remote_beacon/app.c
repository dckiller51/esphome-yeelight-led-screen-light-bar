#include "app_config.h"
#include "../../drivers.h"
#include "frame.h"
#include "rf_control.h"
#include "key_def.h"


#define  DEBUG          0
#define  LED_PIN        GPIO_SWSC7
#define  LED_OFF        gpio_write(LED_PIN,0)
#define  LED_ON         gpio_write(LED_PIN,1)
const unsigned int gpio_row[]={GPIO_PC2,GPIO_PC3,GPIO_PC1,GPIO_PB0,GPIO_PD2};//�����е�IO
const unsigned int gpio_column[]={GPIO_PB3,GPIO_PB4,GPIO_PB5};//�����е�IO
unsigned char pre_key;//��¼�ϴΰ������µ�ֵ
unsigned char current_active_group;//ɫ�����ȵ���ʱ�����
unsigned char key_down_cnt;//�������¼���ֵ
unsigned char key_up_cnt;//�����������ֵ
unsigned char key_off_cnt;//�صƽ����µļ���ֵ���̰�Ϊ���ƣ�����Ϊҹ��
unsigned char key_lumi_chro_cnt;//ɫ�����Ȱ������µļ���ֵ��һֱ������ɫ��ֵ��������
unsigned char led_night_cmd_flag;//ҹ������ִ�б�־
unsigned int loop;
#if 0
const unsigned char key_table[4][4] = {//�������
		{((KEY_ON<<4)|GROUP_ALL),                 ((KEY_CHROMA_DECREASE<<4)|GROUP_ALL), ((KEY_ON<<4)|GROUP_1),                  ((KEY_OFF<<4)|GROUP_1)},
		{((KEY_LUMINANT_INCREASE<<4)|GROUP_ALL),  KEY_NONE,                             ((KEY_LUMINANT_DECREASE<<4)|GROUP_ALL),  KEY_NONE},
		{((KEY_ON<<4)|GROUP_4),                   ((KEY_OFF<<4)|GROUP_4),               ((KEY_ON<<4)|GROUP_3),                  ((KEY_OFF<<4)|GROUP_3)},
		{((KEY_OFF<<4)|GROUP_ALL),                ((KEY_CHROMA_INCREASE<<4)|GROUP_ALL), ((KEY_ON<<4)|GROUP_2),                  ((KEY_OFF<<4)|GROUP_2)},
};
#else
const unsigned char key_table[5][3] = {//�������
		{((KEY_LUMINANT_DECREASE<<4)|GROUP_ALL),((KEY_OFF<<4)|GROUP_2),((KEY_ON<<4)|GROUP_2)},
		{((KEY_CHROMA_DECREASE<<4)|GROUP_ALL),((KEY_OFF<<4)|GROUP_1),((KEY_ON<<4)|GROUP_1)},
		{((KEY_ON<<4)|GROUP_3),((KEY_OFF<<4)|GROUP_3),((KEY_OFF<<4)|GROUP_4)},
		{KEY_NONE,((KEY_ON<<4)|GROUP_4),((KEY_CHROMA_INCREASE<<4)|GROUP_ALL)},
		{((KEY_OFF<<4)|GROUP_ALL),((KEY_LUMINANT_INCREASE<<4)|GROUP_ALL),((KEY_ON<<4)|GROUP_ALL)}
};
#endif
/*******************************************************************
 * �������ܣ�GPIO��ʼ��
 * ��       ����
 * �� �� ֵ��
 ******************************************************************/
void gpio_init_func(void)
{
	unsigned char i;
	gpio_set_func(LED_PIN,AS_GPIO);     //IO��Ϊ��ͨIO
	gpio_set_output_en(LED_PIN,LEVEL_HIGH); //���ʹ�ܹص�
	gpio_set_input_en(LED_PIN,LEVEL_LOW);  //����ʹ�ܹص�
	for(i=0;i<5;i++){
		gpio_set_func(gpio_row[i],AS_GPIO);     //IO��Ϊ��ͨIO
		gpio_set_output_en(gpio_row[i],LEVEL_LOW); //���ʹ�ܹص�
		gpio_set_input_en(gpio_row[i],LEVEL_LOW);  //����ʹ�ܹص�
		gpio_write(gpio_row[i],LEVEL_LOW);        //IO�����Ϊ�͵�ƽ
		gpio_set_up_down_resistor(gpio_row[i],GPIO_PULL_NONE);        //IO��Ϊ����״̬
	}

	for(i=0;i<3;i++){
		gpio_set_func(gpio_column[i],AS_GPIO);        //IO��Ϊ��ͨIO
		gpio_set_output_en(gpio_column[i],LEVEL_LOW);    //���ʹ�ܹص�
		gpio_set_input_en(gpio_column[i],LEVEL_HIGH);       //ʹ������
		gpio_set_up_down_resistor(gpio_column[i],GPIO_PULL_UP_1M);          //��������1M����
		gpio_write(gpio_column[i],LEVEL_LOW);           //�����Ϊ0
		pm_set_gpio_wakeup(gpio_column[i],0,1);       //����IO�͵�ƽ���ѣ���һ����ΪIO���ڶ�����Ϊ���ѵ�ƽ����������Ϊʹ��
	}
//	analog_write(0x0b,0xff);
//	analog_write(0x0c,0xff);
}
/*******************************************************************
 * �������ܣ�����deepsleepǰ���ò���
 * ��       ����
 * �� �� ֵ��
 ******************************************************************/
void set_wakeup_func(void)
{
	unsigned char i;
	for(i=0;i<5;i++)
		gpio_set_up_down_resistor(gpio_row[i],GPIO_PULL_DN_100K);      //������IO����100K������������ʱ����IOΪ�͵�ƽ����ɻ���MCU
	analog_write(0x3a, current_active_group);         //����deepsleepǰ�������ֵ
	analog_write(0x3b, led_remote.rf_seq_no);         //����deepsleepǰ����������к�
}
/*******************************************************************
 * �������ܣ��������ݰ������ݳ�ʼ��
 * ��       ����
 * �� �� ֵ��
 ******************************************************************/
void package_data_init_func(void)
{
	unsigned char i;
	led_remote.dma_len = sizeof(rf_packet_led_remote_t)-sizeof(led_remote.dma_len);//���ð���dma����
	led_remote.rf_len = led_remote.dma_len-2;
	led_remote.type = 0x42;
	led_remote.data_type = 0xff;
	led_remote.data_len = led_remote.rf_len - 7;
	for(i=0;i<6;i++)
		led_remote.mac[i]=0x12+i;
//	led_remote.rf_len1 = led_remote.dma_len-2;
	led_remote.vid = 0x5453;//����VIDֵ��Ŀǰ������Ϊ0x5453���ͻ����Զ���
//	led_remote.pid = 0x12345678;//����ң����ID��һ����ù��뷽ʽ
	led_remote.pid = otp_read(PID_ADDR) | otp_read(PID_ADDR+1)<<8 | otp_read(PID_ADDR+2)<<16 | otp_read(PID_ADDR+3)<<24;
	current_active_group = analog_read(0x3a);//���ϴα�������ֵ����Ϊ��һ���ϵ磬��Ϊ0
	led_remote.rf_seq_no = analog_read(0x3b);//���ϴΰ�������ֵ����Ϊ��һ���ϵ磬��Ϊ0
}
/*******************************************************************
 * �������ܣ�����ɨ��
 * ��       ����
 * �� �� ֵ�����ض�Ӧ�İ���ֵ
 ******************************************************************/
unsigned char remote_key_scan_func(void)
{
	unsigned char i,j;
	for(i=0;i<5;i++){
		gpio_set_up_down_resistor(gpio_row[i],GPIO_PULL_DN_100K);//������IOΪ����100K
//		gpio_set_up_down_resistor(gpio_row[i],GPIO_PULL_DN_100K);//������IOΪ����100K
		delay_us(20);
		for(j=0;j<3;j++){
			if(gpio_read(gpio_column[j])==0){//�а�������
				gpio_set_up_down_resistor(gpio_row[i],GPIO_PULL_NONE);//����
				return key_table[i][j];//������ر�ֵ
			}
		}
		gpio_set_up_down_resistor(gpio_row[i],GPIO_PULL_NONE);//������Ӧ�������£�������
		delay_us(20);
	}
	return 0;
}
void user_init(void)
{
	gpio_init_func();
	rf_init_func();
	package_data_init_func();
	irq_enable();//��ϵͳ���ж�
}

void main_loop(void)
{
	unsigned short Cur_key=0;
	loop++;
	Cur_key=remote_key_scan_func();//������ֵ
	if((Cur_key>>4)== KEY_ON){//�Ƿ�Ϊ���ƽ������Ƽ��򱣴���𣬸�ɫ�����Ȱ���ʹ��
	    current_active_group = Cur_key&0xf;
	}else if(((Cur_key>>4)!= KEY_OFF) && Cur_key){//Ϊɫ�����Ȱ���ʱ��ʹ�ñ�������
	    Cur_key = (Cur_key&0xf0)|current_active_group;
	}

	if(Cur_key){//�а�������
		if(loop&0x08)
			LED_ON;
		else
			LED_OFF;
		if(Cur_key!=pre_key){//����ֵ�Ƿ�仯
			key_down_cnt++;
			if(key_down_cnt>4){//4������
				key_up_cnt=0;
				key_lumi_chro_cnt=0;
				key_off_cnt=0;
				pre_key=Cur_key;
				led_remote.rf_seq_no++;//�������кŸ��£�����Ϊʱ������
			}
		}
		if((pre_key>>4)!=KEY_OFF){
			led_remote.control_key=pre_key;
			if((pre_key>>4)==KEY_LUMINANT_INCREASE||(pre_key>>4)==KEY_LUMINANT_DECREASE||(pre_key>>4)==KEY_CHROMA_INCREASE||(pre_key>>4)==KEY_CHROMA_DECREASE){
				key_lumi_chro_cnt++;
				if(key_lumi_chro_cnt&0x20){//ɫ�����Ȱ�������ʱ��ÿ320ms����1��
					key_lumi_chro_cnt=0;
					led_remote.rf_seq_no++;//�������кŸ��£�����Ϊʱ������
				}
			}
		}else{
			key_lumi_chro_cnt=0;
			key_off_cnt++;
			if(key_off_cnt&0x80){//�صƽ�����1.28s������ҹ��ģʽ
				led_night_cmd_flag=1;//ҹ���������־
				key_off_cnt=0x80;
				led_remote.control_key = (KEY_QUICK_LOW_LIGHT<<4) | (pre_key&0x0f);//ҹ������
			}
		}
	}else{//��������
		key_up_cnt++;
		key_down_cnt=0;
		key_lumi_chro_cnt=0;
		key_off_cnt=0;
		LED_OFF;
		if(key_up_cnt>4){
			if(((pre_key>>4)==KEY_OFF)&&(led_night_cmd_flag==0)){//Ϊ�صư�������δ����ҹ������
				if(key_up_cnt<15){
					led_remote.control_key = pre_key;//��������15�ιص�����ֵ
				}else if(key_up_cnt<25){
					led_remote.control_key = KEY_NONE;//����10�ΰ�������ֵ
					pre_key = KEY_NONE;               //����������������İ���ֵ
				}else{
#if DEBUG
					delay_us(100000);
#else
					set_wakeup_func();
					pm_sleep_wakeup(DEEPSLEEP_MODE,PM_WAKEUP_PAD,0);//����deepsleepģʽ������PAD����
#endif
				}
			}else{
				led_night_cmd_flag=0;
				if(key_up_cnt<15){//����15�ΰ�������ֵ
					led_remote.control_key = KEY_NONE;
					pre_key = KEY_NONE;
				}else{
#if DEBUG
					delay_us(100000);
#else
					set_wakeup_func();
					pm_sleep_wakeup(DEEPSLEEP_MODE,PM_WAKEUP_PAD,0);
#endif
				}
			}
		}
	}
	send_package_data_func();//��������
#if DEBUG
	delay_us(10000);
#else
	pm_sleep_wakeup(SUSPEND_MODE,PM_WAKEUP_TIMER,get_sys_tick()+10*CLOCK_SYS_CLOCK_1MS);//����suspend 10ms
#endif
}
