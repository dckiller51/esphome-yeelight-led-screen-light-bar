#pragma once
#include "frame.h"


unsigned char paired_ID_match(unsigned int pid,unsigned char grp);
void clear_pared_code_func(void);
void pair_id_save_func(void);

#define EEPROM_INFO_START            0
#define EEPROM_PAIR_ID_ADDR          EEPROM_INFO_START
#define EEPROM_ID_INDEX_ADDR         EEPROM_PAIR_ID_ADDR+MAX_PAIRED_REMOTER*5
#define EEPROM_LUMINACE_INDEX_ADDR   EEPROM_ID_INDEX_ADDR+1
#define EEPROM_CHROMA_INDEX_ADDR     EEPROM_LUMINACE_INDEX_ADDR+1
#define EEPROM_LED_ON_ADDR           EEPROM_CHROMA_INDEX_ADDR+1
#define EEPROM_POWER_ON_RECOVER_ADDR EEPROM_LED_ON_ADDR+1
#define EEPROM_SEG_INDEX_ADDR        EEPROM_POWER_ON_RECOVER_ADDR+1
