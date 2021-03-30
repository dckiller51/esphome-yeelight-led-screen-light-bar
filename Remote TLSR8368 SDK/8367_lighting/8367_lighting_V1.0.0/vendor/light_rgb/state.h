#pragma once

typedef enum{
	LED_OFF_STATE=0,
	LED_YL_ON_STATE,
	LED_RGB_ON_STATE,
	LED_RGB_BREATH_STATE,
	LED_LAST_STATE,
}LED_State_e;

typedef enum{
	PAIRING_STATE=0,
	NORMAL_STATE,
}LED_Run_state_e;
