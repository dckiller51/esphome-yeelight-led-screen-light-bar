#pragma once


unsigned short led_blue_target;
unsigned short led_blue_cur;
unsigned short led_red_target;
unsigned short led_red_cur;
unsigned short led_green_target;
unsigned short led_green_cur;
unsigned char led_rgb_state_change_flag;
unsigned int led_rgb_change_tick;


void led_on_rgb_func(void);
void led_rgb_off_func(void);
void led_rgb_set_func(unsigned short Red_v,unsigned short Green_v,unsigned short Blue_v);
void led_set_rgb_power_func(unsigned short Red_v,unsigned short Green_v,unsigned short Blue_v);
