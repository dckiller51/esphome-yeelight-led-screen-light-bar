#pragma once

typedef struct{
	unsigned int   dma_len;				// 0~3 DMA length
	unsigned char  rf_len;					// 4 rf data length = 0x10
	unsigned char  rf_len1;
	unsigned short vid;					// 5~6 vendor ID
	unsigned int   pid;					// 7~10 product ID

	unsigned char  control_key;			// 11 function control key
	unsigned char  rf_seq_no; 			// 12 rf sequence total number, save this value in 3.3v analog register.

	unsigned short button_keep_counter;	// 13~14 sequence number in one certain channel.
	unsigned short control_key_value[3];	// 15, 16, 17, 18
	unsigned char  reserved;
}rf_packet_led_remote_t;		//rf data packet from remoter end.

rf_packet_led_remote_t  led_remote __attribute__((aligned(4)));
