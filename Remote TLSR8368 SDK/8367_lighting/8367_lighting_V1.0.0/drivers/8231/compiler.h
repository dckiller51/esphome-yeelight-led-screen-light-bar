
#pragma once
#define _attribute_packed_		__attribute__((packed))
#define _attribute_aligned_(s)	__attribute__((aligned(s)))
#define _attribute_session_(s)	__attribute__((section(s)))
#define _attribute_ram_code_  	_attribute_session_(".ram_code")
#define _attribute_ota_code_  	_attribute_session_(".ota")//for SDK ota
#define _attribute_custom_code_  	_attribute_session_(".custom") volatile
#define _attribute_no_inline_   __attribute__((noinline)) 
#define _attribute_retention_data_    __attribute__((section(".retention_data")))
#define _inline_ 				inline				//   C99 meaning

/*******for SDK************/
#ifndef BLE_SDK_EN
#define BLE_SDK_EN   					0
#endif
