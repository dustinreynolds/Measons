/*
 * init.c
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 */

#include "stm32l1xx.h"
#include "packet_eeprom.h"
#include "onewire.h"

void init_setup_configuration(config_t config) {
	if (config.status_leds.statusLed1Enabled) {
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIOB->BSRRH |= GPIO_Pin_4;
	}
	if (config.status_leds.statusLed2Enabled) {
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIOB->BSRRH |= GPIO_Pin_5;
	}
	if (config.status_leds.statusLed3Enabled) {
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIOB->BSRRH |= GPIO_Pin_6;
	}
	if (config.status_leds.statusLed4Enabled) {
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIOA->BSRRH |= GPIO_Pin_8;
	}
	if (config.nOW > 0) {
		uint8_t i = 0;
		for (i = 0; i < config.nOW; i++) {
			if (config.onewire[i].bus > 0) {
				onewire_Init((config.onewire[i].bus - 1));
			}
		}
	}

	if (config.nI2C > 0){
		// Do something for each I2C device
		uint8_t i = 0;
		for (i = 0; i < config.nI2C; i++){

			if (config.i2c[i].bus > 0){
				i2c_init(config.i2c[i].bus >> 1);
			}
		}
	}
}

void init_search_new_hardware(config_t * config) {
	uint8_t i = 0;
	if (config->nOW > 0) {
		uint8_t j, k;
		uint8_t bus_list[OW_MAX_SENSORS];
		uint8_t bus_ctr = 0;

		memset(bus_list, 0xff, sizeof(bus_list));

		//Disable all onewire sensors
		for (i = 0; i < config->nOW; i++){
			config->onewire[i].enabled = 0;
		}
		init_3v3RegOn(true);
		delayms(1000);
		for (i = 0; i < config->nOW; i++) {
			uint8_t rom[8];
			uint8_t ow_continue = 0;
			int result;
			uint8_t buffer[100];
			if (config->onewire[i].bus == 0) {
				continue;
			}

			for (k=0; k < OW_MAX_SENSORS; k++){
				//look to see if we have scanned this bus before
				if (bus_list[k] == 0xFF){
					break;
				}
				if (bus_list[k] == config->onewire[i].bus){
					ow_continue = 1;
					break;
				}
			}
			if (ow_continue == 1){ //we have
				continue;
			}
			bus_list[bus_ctr++] = config->onewire[i].bus;

			result = OWFirst((config->onewire[i].bus-1));
			j = 0;
			while (result) {
				onewire_read_latest_ROM(&rom);
				//sprintf(buffer, "%d ROM ID = %d,%d,%d,%d,%d,%d,%d,%d\r\n", j,
				//		rom[0], rom[1], rom[2],
				//		rom[3], rom[4], rom[5],
				//		rom[6], rom[7]);
				//uart_OutString(USART2, buffer);

				//Compare this rom id vs id's stored
				for (k = 0; k < config->nOW; k++){
					if (config->onewire[k].bus == config->onewire[i].bus ){
						//on same bus
						if (    (config->onewire[k].unique_id[0] == rom[0]) &&
								(config->onewire[k].unique_id[1] == rom[1]) &&
								(config->onewire[k].unique_id[2] == rom[2]) &&
								(config->onewire[k].unique_id[3] == rom[3]) &&
								(config->onewire[k].unique_id[4] == rom[4]) &&
								(config->onewire[k].unique_id[5] == rom[5]) &&
								(config->onewire[k].unique_id[6] == rom[6]) &&
								(config->onewire[k].unique_id[7] == rom[7]) ){
							//exact match on this bus, we knew about this device before.
							config->onewire[k].enabled = 1;
							break; //break out of the for loop, scan for different ROM id
						}
					}else{
						//on different bus, sensor moved around
						if (    (config->onewire[k].unique_id[0] == rom[0]) &&
								(config->onewire[k].unique_id[1] == rom[1]) &&
								(config->onewire[k].unique_id[2] == rom[2]) &&
								(config->onewire[k].unique_id[3] == rom[3]) &&
								(config->onewire[k].unique_id[4] == rom[4]) &&
								(config->onewire[k].unique_id[5] == rom[5]) &&
								(config->onewire[k].unique_id[6] == rom[6]) &&
								(config->onewire[k].unique_id[7] == rom[7]) ){
							//exact match on this id, we knew about this device before on a different bus
							config->onewire[k].bus = config->onewire[i].bus;
							config->onewire[k].enabled = 1;
							break; //break out of the for loop, scan for different ROM id
						}
					}
				}
				if (k == config->nOW){
					//new device discovered!
					config->onewire[config->nOW].bus = config->onewire[i].bus;
					config->onewire[config->nOW].enabled = 1;
					config->onewire[config->nOW].type = 0x00;
					for (k = 0; k < 8; k++){
						config->onewire[config->nOW].unique_id[k] = rom[k];
					}
					config->nOW++;  //now there is one more device on this bus
				}

				result = OWNext((config->onewire[i].bus-1));
				j++;
			}
		}
	}
	/* This i2c loop doesn't quite search for new i2c devices... */
	for (i = 0; i < config->nI2C; i++) {
		init_3v3RegOn(true);
		delayms(1000);
		if (config->i2c[i].bus > 0){
			i2c_init(config->i2c[i].bus >> 1);

			if (config->i2c[i].type == I2C_TYPE_RTC_DS3231){
				uint8_t result;
				i2c_ds3231_reset(config->i2c[i].bus >> 1);
				result = i2c_ds3231_init(config->i2c[i].bus >> 1);

				if (result == 0){
					config->i2c[i].enabled = 1;
					config->rtc.i2c_rtc_enabled = 1;
					config->rtc.i2c_rtc_number = i;
				}
			}
		}
	}

	init_3v3RegOn(false);

}

void init_flash(void){
	GPIO_InitTypeDef GPIO_InitStructure;


	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	/* Initialize GPIO for SPI, making sure all CS pins are set to output high */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	/* Flash CS */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIOC->BSRRH |= GPIO_Pin_8;
}

void init_rfm69hw(void){
	GPIO_InitTypeDef GPIO_InitStructure;

	/* CS for RF Radio just in case it's present */
	//GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIOB->BSRRL |= GPIO_Pin_12;
}

void init_USART2(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/*-------------------------- GPIO Configuration ----------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // PA.2 USART2_TX, PA.3 USART1_RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect USART pins to AF */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
}

void init_3v3Reg(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIOC->BSRRH |= GPIO_Pin_3;
}

void init_3v3RegOn(bool state) {
	if (state) {
		GPIOC->BSRRL |= GPIO_Pin_3;
	} else {
		GPIOC->BSRRH |= GPIO_Pin_3;
	}
}

void init_GPIO_Configuration(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	/*-------------------------- GPIO Configuration ----------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // PA.2 USART2_TX, PA.3 USART1_RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect USART pins to AF */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	/*-------------------------- GPIO Configuration ----------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect USART pins to AF */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
}

void init_HSI(void) {
	FLASH_SetLatency(FLASH_Latency_1);

	RCC_PLLConfig(RCC_PLLSource_HSI, RCC_PLLMul_6, RCC_PLLDiv_3); //32 MHz
	RCC_PLLCmd(ENABLE);

	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {
	}
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_PCLK1Config(RCC_HCLK_Div1);
	RCC_PCLK2Config(RCC_HCLK_Div1);
}
