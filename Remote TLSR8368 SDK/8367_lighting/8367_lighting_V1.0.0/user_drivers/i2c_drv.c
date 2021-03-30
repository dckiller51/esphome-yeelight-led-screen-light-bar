
//#include "../../proj/tl_common.h"
#include "../drivers.h"
#include "i2c_drv.h"


static inline void i2c_wait(void){
}

void i2c_long_wait(void){
	volatile unsigned int i;

	for(i=10;i!=0;i--)
		asm("tnop");
}


static inline int i2c_scl_in(void){
	return gpio_read(i2c_pin_scl);
}


static inline int i2c_sda_in(void){
	return gpio_read(i2c_pin_sda);
}

static inline void i2c_scl_init(void){
	gpio_set_func(i2c_pin_scl, AS_GPIO);
	gpio_set_input_en(i2c_pin_scl, 1);
}

static inline void i2c_sda_init(void){
	gpio_set_func(i2c_pin_sda, AS_GPIO);
	gpio_set_input_en(i2c_pin_sda, 1);
}

static inline void i2c_scl_idle(void){
	gpio_set_output_en(i2c_pin_scl, 0);
	gpio_write(i2c_pin_scl, 0);
}

static inline void i2c_sda_idle(void){
	gpio_set_output_en(i2c_pin_sda, 0);
	gpio_write(i2c_pin_sda, 0);
}

void i2c_init(){


	i2c_scl_init();
	i2c_sda_init();
	i2c_sda_idle();
	i2c_scl_idle();

}

/*
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
\\ void i2c_start(void)
\\   Sets clock high, then data high.  This will do a stop if data was low.
\\   Then sets data low, which should be a start condition.
\\   After executing, data is left low, while clock is left high
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
*/
void i2c_start(void)
{
	i2c_scl_init();	
	i2c_sda_init();	
	i2c_sda_idle();	
	i2c_scl_idle();		
	i2c_sda_out(1);		//sda: 1
	i2c_scl_out(1);		//scl: 1
	i2c_sda_out(0);		//sda: 0
	i2c_wait();

}

/*
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
\\ void i2c_stop(void)    
\\  puts data low, then clock low,
\\  then clock high, then data high.
\\  This should cause a stop, which
\\  should idle the bus, I.E. both clk and data are high.
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
*/
void i2c_stop(void)
{
	i2c_sda_out(0);
	i2c_wait();
	i2c_scl_out(0);
	i2c_wait();
	i2c_scl_out(1);
	i2c_wait();
	i2c_sda_out(1);
}

static void i2c_wirte_bit (int bit)
{
	i2c_scl_out(0);
	i2c_sda_out(bit);
	i2c_long_wait();
	i2c_scl_out(1);
}

// Read a bit from I2C bus
static int i2c_read_bit(void) {
	i2c_wirte_bit(1);
	return i2c_sda_in();
}

int i2c_write_byte(unsigned char dat){
	int i = 0x80;
	while(i){
		i2c_wirte_bit((dat & i));
		i = i >> 1;
	}
	return i2c_read_bit();
}

unsigned char i2c_read_byte(int last){
	unsigned char dat = 0;
	unsigned int i;
	for(i=0;i<8;i++){
		i2c_wirte_bit(1);
		if(i2c_sda_in()){
			dat = (dat << 1) | 0x01;
		}else{
			dat = dat << 1;
		}
	}
	i2c_wirte_bit(last);
	return dat;
}

void i2c_write(unsigned char id, unsigned char addr, unsigned char dat)
{
	i2c_start();
	i2c_write_byte(id);
	i2c_write_byte(addr);
	i2c_write_byte(dat);
	i2c_stop();
}

unsigned char i2c_read(unsigned char id, unsigned char addr)
{
	unsigned char dat;
	i2c_burst_read (id, addr, &dat, 1);
	return dat;
}

void i2c_burst_read(unsigned char id, unsigned char addr,unsigned char *p,unsigned char n)
{
	i2c_start();

	i2c_write_byte (id);
	i2c_write_byte (addr);
	i2c_sda_out(1);
	i2c_scl_out(0);
	i2c_long_wait();
	i2c_scl_out(1);
	i2c_sda_out(0);
	
	i2c_write_byte (id | 1);

	for (int k = 0; k < n; ++k) {
		*p++ = i2c_read_byte( k == (n-1) );
	}
	i2c_stop();
	
}


