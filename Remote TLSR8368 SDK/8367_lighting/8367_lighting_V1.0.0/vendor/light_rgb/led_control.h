#pragma once

unsigned char led_flash_cnt;         //led闪烁次数
unsigned int  led_flash_tick;        //led闪烁时间点


void led_init_func(void);
void led_flash_updata_func(unsigned char Cnt);
void led_pask_process_func(void);
