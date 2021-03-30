
#ifdef MCU_STARTUP_8231

#ifndef __LOAD_RAM_SIZE__
#define	__LOAD_RAM_SIZE__		0xc
#endif

	.code	16
@********************************************************************************************************
@                                           MACROS AND DEFINIITIONS
@********************************************************************************************************
@.include "version.in"

					@ Mode, correspords to bits 0-5 in CPSR
	.equ MODE_BITS,		0x1F	@ Bit mask for mode bits in CPSR
	.equ IRQ_MODE, 		0x12	@ Interrupt Request mode
	.equ SVC_MODE, 		0x13	@ Supervisor mode 

	.equ IRQ_STK_SIZE,	0x100
	.equ __LOAD_RAM, 	__LOAD_RAM_SIZE__
	
@********************************************************************************************************
@                                            TC32 EXCEPTION VECTORS
@********************************************************************************************************

	.section	.vectors,"ax"
	.global		__reset
	.global	 	__irq
	.global 	__start
	.global		__LOAD_RAM

__start:					@ MUST,  referenced by boot.link

	.extern irq_handler

	.extern  _ramcode_size_div_16_
	.extern  _ramcode_size_div_256_
	.extern  _ramcode_size_div_16_align_256_
	.extern  _ictag_start_
	.extern  _ictag_end_

	.org 0x0
	tj	__reset
	@.word	(BUILD_VERSION)
	.org 0x8
	.word	(0x544c4e4b)
	.word	(0x00880000 + _ramcode_size_div_16_align_256_)

	.org 0x10
	tj		__irq
	.org 0x18
	.word	(_bin_size_)
@********************************************************************************************************
@                                   LOW-LEVEL INITIALIZATION
@********************************************************************************************************
	.extern  main


	.org 0x20
__reset:
@	tloadr     	r0, DAT0 + 36
@	tmov		r1, #1024		@ set sws to GPIO
@	tstorer 	r1, [r0, #0]

@	tloadr     	r0, DAT0 + 40		@**** enable watchdog at the very first time
@	tloadr     	r1, DAT0 + 44
@	tstorer		r1, [r0, #0]
@********************************************************************************************************
@                              		 FILL .DATA AND .BSS WITH 0xFF
@********************************************************************************************************
	tloadr	r0, FLL_D
	tloadr	r1, FLL_D+4
	tloadr	r2, FLL_D+8

FLL_STK:
	tcmp	r1, r2
	tjge	FLL_STK_END
	tstorer r0, [r1, #0]
	tadd    r1, #4
	tj		FLL_STK
FLL_STK_END:
@********************************************************************************************************
@                              		 UPDATE SP UNDER IRQ/SVC MODE
@********************************************************************************************************
	tloadr	r0, DAT0
	tmcsr	r0			
	tloadr	r0, DAT0 + 8
	tmov	r13, r0  

	tloadr	r0, DAT0 + 4
	tmcsr	r0	
	tloadr	r0, DAT0 + 12
	tmov	r13, r0  
@********************************************************************************************************
@                                    .BSS INITIALIZATION FOR 0
@********************************************************************************************************
	tmov	r0, #0
	tloadr	r1, DAT0 + 16
	tloadr	r2, DAT0 + 20

ZERO:
	tcmp	r1, r2
	tjge	ZERO_END
	tstorer	r0, [r1, #0]
	tadd    r1, #4
	tj		ZERO
ZERO_END:
@********************************************************************************************************
@                                    IC TAG INITIALIZATION
@********************************************************************************************************
	tloadr	r1, DAT0 + 28
	tloadr	r2, DAT0 + 32

ZERO_TAG:
	tcmp	r1, r2
	tjge	ZERO_TAG_END
	tstorer	r0, [r1, #0]
	tadd    r1, #4
	tj		ZERO_TAG
ZERO_TAG_END:
@********************************************************************************************************
@                                    IC CACHE INITIALIZATION
@********************************************************************************************************
SETIC:
	tloadr     	r1, DAT0 + 24
	tloadr      r0, DAT0 + 36					@ IC tag start
	tstorerb	r0, [r1, #0]
	tadd    	r0, #1							@ IC tag end
	tstorerb	r0, [r1, #1]
	@tmov		r0, #0;
	@tstorerb	r0, [r1, #2]

/*
#if FLASH_WAKEUP_EN

FLASH_WAKEUP_BEGIN:
	tloadr      r0,DATA_I+24
	tmov		r1,#0
	tstorerb    r1,[r0,#1]
	tmov        r1,#171						@Flash deep cmd: 0xAB
	tstorerb    r1,[r0,#0]
	tmov		r2,#0
	tmov        r3,#6
TNOP:
	tadd        r2,#1
	tcmp        r2,r3
	tjle        TNOP
	tmov		r1,#1
	tstorerb    r1,[r0,#1]
FLASH_WAKEUP_END:

#endif
*/

@********************************************************************************************************
@                                    RETENTION DATA INITIALIZATION
@********************************************************************************************************
#if 1
COPY_RET_DATA:
	tloadr		r1, DATA_I					@ r1 = _rstored_
	tloadr		r2, DATA_I+4				@ r2 = _retention_data_start_
	tloadr		r3, DATA_I+8				@ r3 = _retention_data_end_
COPY_RET_DATA_BEGIN:
	tcmp		r2, r3						@
	tjge		COPY_RET_DATA_END			@ r2>=r3 jump to COPY_RET_DATA_END
	tloadr		r0, [r1, #0]				@ r0 = *(unsigned int*)(_rstored_)
	tstorer 	r0, [r2, #0]				@ *(unsigned int*)(_retention_data_start_) = r0
	tadd    	r1, #4						@ r1 = r1 + 4
	tadd		r2, #4						@ r2 = r2 + 4
	tj			COPY_RET_DATA_BEGIN			@ jump to COPY_RET_DATA_BEGIN
COPY_RET_DATA_END:
#endif



@********************************************************************************************************
@                                    DATA INITIALIZATION
@********************************************************************************************************
	tloadr		r1, DATA_I+12
	tloadr		r2, DATA_I+16
	tloadr		r3, DATA_I+20
COPY_DATA:
	tcmp		r2, r3
	tjge		COPY_DATA_END
	tloadr		r0, [r1, #0]
	tstorer 	r0, [r2, #0]
	tadd    	r1, #4
	tadd		r2, #4
	tj			COPY_DATA
COPY_DATA_END:

#if 0
SETSPISPEED:
	tloadr     	r1, DAT0 + 36
	tmov		r0, #0xbb		@0x0b for fast read; 0xbb for dual dat/adr
	tstorerb	r0, [r1, #0]
	tmov		r0, #3			@3 for dual dat/adr
	tstorerb	r0, [r1, #1]
#endif

	tjl	main
END:	tj	END

	.balign	4
DAT0:
	.word	0x12			   			@IRQ    @0
	.word	0x13			   			@SVC    @4
	.word	(irq_stk + IRQ_STK_SIZE)
	.word	(0x80a000)		    		@12  stack end
	.word	(_start_bss_)               @16
	.word	(_end_bss_)                 @20
	.word	(0x80060c)                  @24
	.word	_ictag_start_               @28		@ IC tag start
	.word	_ictag_end_	            	@32		@ IC tag end
	.word	_ramcode_size_div_256_		@36
@	.word	(0x808000 + __LOAD_RAM * 0x100)                  @28		@ IC tag start
@	.word	(0x808000 + (__LOAD_RAM + 1) * 0x100)            @32		@ IC tag end
@	.word	(0x80000e)    				@36
@	.word	(0x80058c)                  @36		gpio
@	.word	(0x800620)                  @40		watchdog
@	.word	(0x802c01)                  @44		watchdog

DATA_I:
	.word   (_rstored_)                 @0
	.word   (_retention_data_start_)    @4
    .word   (_retention_data_end_)      @8
	.word	_dstored_
	.word	_start_data_
	.word	_end_data_
	//.word	(0x0080000c)                @24

FLL_D:
	.word	0xffffffff
	.word	(_start_data_)
	.word	(0x80a000)

	.align 4
__irq:
	tpush    	{r14}
	tpush    	{r0-r7}
	tmrss    	r0
	
	tmov		r1, r8
	tmov		r2, r9
	tmov		r3, r10
	tmov		r4, r11
	tmov		r5, r12
	tpush		{r0-r5}
	
	tjl      	irq_handler

	tpop		{r0-r5}
	tmov		r8, r1
	tmov		r9, r2
	tmov		r10,r3
	tmov		r11,r4
	tmov		r12,r5

	tmssr    	r0
	tpop		{r0-r7}
	treti    	{r15}

ASMEND:

	.section .bss
	.align 4
	.lcomm irq_stk, IRQ_STK_SIZE
	.end

#endif
