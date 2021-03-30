
#pragma once
#ifndef _OTP_H_
#define _OTP_H_


//void otp_init_readwrite(void);
//void otp_init_read(void);
//void otp_init(void);
void otp_write(unsigned short addr, unsigned char  value);
void otp_stop_write(void);
unsigned char otp_read(unsigned short addr);
unsigned char otp_normal_read(unsigned short addr);

#endif
