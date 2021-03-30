
#pragma once

//#include "../../proj/common/types.h"



volatile unsigned int i2c_pin_scl;
volatile unsigned int i2c_pin_sda;

// Pulling the line to ground is considered a logical zero while letting the line float is a logical one.   http://en.wikipedia.org/wiki/I%C2%B2C
static inline void i2c_scl_out(int v){
	gpio_set_output_en(i2c_pin_scl, (!v));
}

// Pulling the line to ground is considered a logical zero while letting the line float is a logical one.   http://en.wikipedia.org/wiki/I%C2%B2C
static inline void i2c_sda_out(int v){
	gpio_set_output_en(i2c_pin_sda, (!v));
}


void i2c_init(void);
void i2c_write(unsigned char id, unsigned char addr, unsigned char dat);
unsigned char i2c_read(unsigned char id, unsigned char addr);
void i2c_burst_read(unsigned char id, unsigned char addr, unsigned char *p, unsigned char n);

