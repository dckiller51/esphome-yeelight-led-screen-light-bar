#pragma once

#if BEACON_MODE
typedef struct{
	unsigned int   dma_len;
	unsigned char  type;
	unsigned char  rf_len;
	unsigned char  data_len;
	unsigned char  data_type;
	unsigned short vid;
	unsigned int   pid;
	unsigned char  pkt_seq;
	unsigned char  key_control;
	unsigned short value[3];
}LED_Package_t;
#else
typedef struct{
	unsigned int   dma_len;
	unsigned char  rf_len;
	unsigned char  rf_len1;
	unsigned short vid;
	unsigned int   pid;
	unsigned char  pkt_seq;
	unsigned char  key_control;
	unsigned short value[3];
}LED_Package_t;
#endif
LED_Package_t led_remote __attribute__((aligned(4)));
