#pragma once

typedef enum{
	NORDIC_SB_MODE=0,
	NORDIC_ESB_MODE
}Nordic_modeTypeDef;


unsigned char rf_rx_buffer_get(void);
void rf_set_access_code_len(unsigned char len);
void rf_set_access_code_value(unsigned char pipe,unsigned long long accesscode);
void rf_set_nordic_mode(Nordic_modeTypeDef mode,unsigned char len);
void swap_data(unsigned char *data,unsigned char len);
void rf_set_nordic_address(unsigned char pipe,unsigned long long accesscode);
void rf_start_nordic_ptx  (unsigned char* RF_TxAddr ,unsigned int RF_StartTick,unsigned int RxTimeoutUs,unsigned int Retry);
void rf_start_nordic_prx  (unsigned char* RF_TxAddr ,unsigned int RF_StartTick);
void rf_set_nordic_timing  (unsigned short retry_delay,unsigned short rx_delay,unsigned short tx_delay);
void rf_set_retry_delay(unsigned short timing);
void rf_set_rxstl(unsigned short timing);
void rf_set_txstl(unsigned short timing);
unsigned int Rf_RCV_PKT_Valid(unsigned char* p);

