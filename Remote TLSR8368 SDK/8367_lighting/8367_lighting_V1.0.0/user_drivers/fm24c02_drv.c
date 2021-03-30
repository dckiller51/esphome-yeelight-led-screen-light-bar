//#include "../common.h"
#include "../drivers.h"
#include "fm24c02_drv.h"

void fm24c02_init_func(unsigned int scl,unsigned int sda)
{
	extern unsigned int i2c_pin_scl;
	extern unsigned int i2c_pin_sda;
	i2c_pin_scl = scl;
	i2c_pin_sda = sda;
	gpio_set_up_down_resistor(i2c_pin_scl,GPIO_PULL_UP_10K);
	gpio_set_up_down_resistor(i2c_pin_sda,GPIO_PULL_UP_10K);
//	i2c_master_init(FM24C02ID,40);
//	i2c_set_pin(PinGrp);
//	CFG_PC4_MSD();
//	CFG_PC5_MCK();
	i2c_init();
}

unsigned int fm24c02_read_func(int address)
{
	unsigned char dat;
	dat=i2c_read(0xa0,address);
	return dat;
}

void fm24c02_write_func(int address, int data)
{
	static unsigned int Prev_wr_timing=0;

	while(!timeout_us(Prev_wr_timing, 5000)){
	};
	i2c_write(0xa0,address, data);
	Prev_wr_timing = get_sys_tick();

}

