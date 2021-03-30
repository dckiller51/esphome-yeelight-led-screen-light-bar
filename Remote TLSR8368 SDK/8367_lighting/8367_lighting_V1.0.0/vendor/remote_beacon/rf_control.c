//#include "../../common.h"
#include "../../drivers.h"
#include "frame.h"
#include "rf_control.h"


const unsigned char rf_channel[4]={1,24,51,76};


void rf_init_func(void)
{
#if 0
//	rf_drv_init(RF_MODE_BLE_1M_NO_PN);
	rf_set_access_code_len(4);
//	rf_set_access_code_value(0,0x8e89bed6);
	rf_set_ble_crc_adv();
	rf_set_ble_access_code_adv();
	rf_set_rx_buff(rx_packet,RX_PACKGET_SIZE,1);
	rf_set_power_level_index(RF_POWER_7dBm);
	rf_set_trx_state(RF_MODE_RX,rf_channel[0]);
	rf_irq_clr_mask(0xffff);                      //�ȹص����е�RF�ж�
	rf_irq_set_mask(FLD_RF_IRQ_RX);//��RF RX�ж�
	irq_set_mask(FLD_IRQ_ZB_RT_EN);//��RF���ж�
#else
	rf_set_ble_crc_adv();
	rf_set_ble_access_code_adv();
	rf_set_power_level_index(RF_POWER_7dBm);
	rf_set_ble_channel(37);
	rf_set_tx_rx_off();
	rf_irq_clr_mask(0xffff);                      //�ȹص����е�RF�ж�
	rf_irq_set_mask(FLD_RF_IRQ_RX);//��RF RX�ж�
	irq_set_mask(FLD_IRQ_ZB_RT_EN);//��RF���ж�
	irq_enable();//��ϵͳ���ж�
	READ_REG16(0xf04)=0x40;
	rf_set_tx_on();
#endif
}

void send_package_data_func(void)
{
	unsigned char i;
	for(i=0;i<3;i++){
//		rf_set_trx_state(RF_MODE_TX,rf_channel[i]);
		rf_set_ble_channel(37+i);
		delay_us(200);
		rf_start_stx((void *)&led_remote,get_sys_tick()+32);
		while(!rf_is_tx_finish());
		rf_clr_tx_finish();
	}
}
