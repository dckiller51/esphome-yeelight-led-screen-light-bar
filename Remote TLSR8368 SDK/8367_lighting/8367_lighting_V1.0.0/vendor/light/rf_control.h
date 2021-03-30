#pragma once

void rf_init_func(void);
void rf_change_channel_func(void);
void rf_packget_pro_func(void);

typedef enum{
	PAIRRING_STATE=0,
	CLEARCODE_STATE,
	NORMAL_STATE,
	LAST_SYS_STATE,
}Sys_run_state;


