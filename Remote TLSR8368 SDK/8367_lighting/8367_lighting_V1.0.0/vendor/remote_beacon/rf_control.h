#pragma once

#define RX_PACKGET_SIZE     64
unsigned char rx_packet[RX_PACKGET_SIZE*2] __attribute__((aligned(4)));

void rf_init_func(void);
void send_package_data_func(void);
