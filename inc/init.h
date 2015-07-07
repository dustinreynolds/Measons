/*
 * init.h
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 */

#ifndef INIT_H_
#define INIT_H_

#include "stdint.h"
#include "packet_eeprom.h"

void init_setup_configuration(config_t config);
void init_USART2(void);
void init_HSI(void);
void init_3v3Reg(void);
void init_3v3RegOn(bool state);
void init_search_new_hardware(config_t * config);
//void init_RCC_Configuration(void);
//void init_GPIO_Configuration(void);

#endif /* INIT_H_ */
