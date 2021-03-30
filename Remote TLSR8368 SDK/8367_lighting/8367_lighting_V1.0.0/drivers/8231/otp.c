
#include "bsp.h"
#include "register.h"
#include "timer.h"
#if 0
static void otp_init_clock(void){
	reg_dcdc_clk = 0x1f;    // open DCDC 6.75 clk to 8M
	delay_us(300);
	reg_dcdc_clk = 0x13;    // set DCDC 6.75 clk to 60M
	delay_us(300);
//	reg_otp_ctrl = FLD_OTP_OEN_PWDN | FLD_OTP_WEN_PWDN | FLD_OTP_PCEN_PWDN | FLD_OTP_CLK | FLD_OTP_OEN | FLD_OTP_FAST_CLK;
}
void otp_init_readwrite(void){
	/* open vpp 6.75V */
	rega_poweron_dcdc_6p75();
	delay_us(300);
	otp_init_clock();
}
void otp_init_read(void){
	otp_init_clock();
}
#endif
void otp_write(unsigned short addr, unsigned char value){
	write_reg8(0x800012, 0x7c);
	reg_otp_addr_para = addr;
	write_reg8(0x80001a, 0x02);
	reg_otp_byte_dat = value;
#if (SYS_CLOCK_RC && (DUT_8368_OTP||DUT_8368_OTP_NORMAL_READ))
	sleep_100us_offset();
#else
	delay_us(100);
#endif
	reg_otp_byte_dat = value;
}
#if 0
void otp_stop_write(void)
{
	rega_powerdn_dcdc_6p75();
}

u8 otp_read(u16 addr){
	volatile u8 value;
	addr = ((addr-4) & 0x3fff);
	write_reg8(0x800012, 0x7c);
	reg_otp_addr_para = addr;
#if(1)
	write_reg8(0x80001a, 0x06); //maginal read
#else
	write_reg8(0x80001a, 0x00);
#endif

	value = reg_otp_byte_dat;
	delay_us(1);
	value = read_reg8(0x800019);
	delay_us(1);
	return read_reg8(0x800019);
}

u8 otp_normal_read(u16 addr){
	volatile u8 value;
	addr = ((addr-4) & 0x3fff);
	write_reg8(0x800012, 0x7e);   //core12_<1> OTP  fast clk
	reg_otp_addr_para = addr;
#if(0)
	write_reg8(0x80001a, 0x06); //maginal read
#else
	write_reg8(0x80001a, 0x00); //normal read
#endif

	value = reg_otp_byte_dat;
	delay_us(1);
	value = read_reg8(0x800019);
	delay_us(1);
	return read_reg8(0x800019);
}
#else
//by congqing
unsigned char otp_read(unsigned short addr){
	volatile unsigned char value;
	addr = ((addr) & 0x3fff);
	write_reg8(0x800012, 0x7c);
	reg_otp_addr_para = addr;
#if(1)
	write_reg8(0x80001a, 0x06); //maginal read
#else
	write_reg8(0x80001a, 0x00);
#endif
	
	//value = reg_otp_byte_dat;
	//delay_us(1);
	value = read_reg8(0x800019);
	delay_us(1);
	return read_reg8(0x800019);
}

unsigned char otp_normal_read(unsigned short addr){
	volatile unsigned char value;
	addr = ((addr) & 0x3fff);
	write_reg8(0x800012, 0x7e);   //core12_<1> OTP  fast clk
	reg_otp_addr_para = addr;
#if(0)
	write_reg8(0x80001a, 0x06); //maginal read
#else
	write_reg8(0x80001a, 0x00); //normal read
#endif

	//value = reg_otp_byte_dat;
	//delay_us(1);
	value = read_reg8(0x800019);
	delay_us(1);
	return read_reg8(0x800019);
}
#endif
#if 0
void rega_poweron_dcdc_6p75(void)
{
	u8 v = 0x14;
	analog_write(rega_dcdc_ctrl, v);
	delay_us(1000);
	analog_write(rega_dcdc_ctrl, 0x54);
}

void rega_powerdn_dcdc_6p75(void)
{
	u8 v = 0x0c;
	analog_write(rega_dcdc_ctrl, v);
}
#endif
