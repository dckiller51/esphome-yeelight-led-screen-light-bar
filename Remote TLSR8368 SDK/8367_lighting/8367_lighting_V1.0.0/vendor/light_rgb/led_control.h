#pragma once

unsigned char led_flash_cnt;         //led��˸����
unsigned int  led_flash_tick;        //led��˸ʱ���


void led_init_func(void);
void led_flash_updata_func(unsigned char Cnt);
void led_pask_process_func(void);
