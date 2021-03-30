#pragma once

#define	MCU_CORE_8231 		1


#if(CHIP_TYPE == CHIP_TYPE_8231)
	#define MCU_CORE_TYPE   MCU_CORE_8231
#else
	#define MCU_CORE_TYPE	1000
#endif


