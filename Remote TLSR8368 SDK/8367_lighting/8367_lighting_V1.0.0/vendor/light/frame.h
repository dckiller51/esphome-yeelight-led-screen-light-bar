#pragma once

#define MAX_PAIRED_REMOTER  8
#define RX_PACKGET_SIZE     64
unsigned char rx_packet[RX_PACKGET_SIZE*2] __attribute__((aligned(4)));

typedef struct{
	unsigned int  pid; //product ID 遥控器ID
	unsigned char group_id;//group_id = 1, 2, 4, 8, f遥控器对应的组ID
}Pairing_info_t;

typedef struct{
	Pairing_info_t pared_remote[MAX_PAIRED_REMOTER];    //must the first one.
	unsigned char  paire_index;
	unsigned char  luminance_index;//亮度下标
	unsigned char  chroma_index;//色温下标
	unsigned char  led_on;   //when power on "1" indicate the LED's off by the switch

	unsigned char  power_on_recover;//上电状态是否切换标志
	unsigned char  seg_index;//状态切换值

}LED_control_info_t;

typedef struct{
	unsigned int dma_len;				// 0~3 DMA length
	unsigned char  rf_len;					// 4 rf data length = 0x10
	unsigned char  rf_len1;
	unsigned short  vid;					// 5~6 vendor ID
	unsigned int  pid;					// 7~10 product ID

	unsigned char   control_key;			// 11 function control key
	unsigned char   rf_seq_no; 			// 12 rf sequence total number, save this value in 3.3v analog register.

	unsigned short  button_keep_counter;	// 13~14 sequence number in one certain channel.
	unsigned short  control_key_value[3];	// 15, 16, 17, 18
	unsigned char   reserved;
}rf_packet_led_remote_t;		//rf data packet from remoter end.

LED_control_info_t led_control;
rf_packet_led_remote_t relay_pkt __attribute__((aligned(4)));
unsigned char g_packget_new;
unsigned char g_packget_cmd;
unsigned int g_packget_pid;
unsigned char g_packget_grp;
unsigned short g_packget_lumi;
unsigned short g_packget_chrome;
unsigned int remote_save_pid;
unsigned char  remote_save_grp;
