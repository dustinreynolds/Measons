/*
 * main.c
 *
 *  Created on: Mar 21, 2015
 *      Author: Dustin
 *
 * Copyright (c) 2015, Dustin Reynolds
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of [project] nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "stdio.h"
#include "stm32l1xx.h"
#include "uart.h"
#include "packet_eeprom.h"
#include "string.h"
#include "timer.h"
#include "init.h"
#include "spi.h"
#include "packet_00.h"
#include "sx1231h.h"
#include "flash.h"

typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
#define DATA_EEPROM_START_ADDR     0x08080000
#define DATA_EEPROM_END_ADDR       0x08083FFF
#define DATA_EEPROM_PAGE_SIZE      0x8

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;
__IO TestStatus DataMemoryProgramStatus = PASSED;
uint32_t NbrOfPage = 0, j = 0, Address = 0;

int main(void)
{
	USART_TypeDef * USARTx = USART2;
	parser_t eeprom_parser;
	config_t eeprom_config;
	static uint8_t role_setup = 0;

	packet_eeprom_init_config(&eeprom_config);
	//init_RCC_Configuration();

	init_HSI();

	timer_TIM2_Configuration();

	//Initialize FLASH GPIOs - always present
	init_flash();

	//initialize RFM69 GPIOs - always present
	init_rfm69hw();

	//Initialize SPI DMA and peripheral
	spi_SPI2_Configuration();

	sx1231h_init(SPI_RFM69_1);

	//initialize 3.3vctl regulator - always present
	init_3v3Reg();

	init_3v3RegOn(false);

	init_USART2();

	uart_Configuration(USARTx, UART_POLLING);
	uart_OutString(USARTx,"Welcome to Nucleo L152RE\r\n");

	if (!flash_present()){
		uart_OutString(USARTx,"Flash is not responding...\r\n");
	}
	//Read eeprom settings to determine features
	packet_eeprom_load_configuration(&eeprom_config);
	packet_eeprom_print_configuration(USARTx, eeprom_config);

	//initialize corresponding GPIOs
	init_setup_configuration(eeprom_config);

	init_search_new_hardware(&eeprom_config);

	Address = DATA_EEPROM_START_ADDR;
	packet_eeprom_save_configuration(&Address, eeprom_config); //rewrite eeprom with newly discovered sensors.
	//init_GPIO_Configuration();
	GPIOB->BSRRL |= GPIO_Pin_4;
	GPIOB->BSRRL |= GPIO_Pin_5;
	GPIOB->BSRRL |= GPIO_Pin_6;
	GPIOA->BSRRL |= GPIO_Pin_8;
	delayms(1000);
	GPIOB->BSRRH |= GPIO_Pin_4;
	GPIOB->BSRRH |= GPIO_Pin_5;
	GPIOB->BSRRH |= GPIO_Pin_6;
	GPIOA->BSRRH |= GPIO_Pin_8;

	role_init(eeprom_config);
	//scan for new hardware not described in config (new sensors)
	//should modify to determine if new hardware is actually detected, and then
	// call save configuration
	//init_search_new_hardware(&eeprom_config);
	//packet_eeprom_print_configuration(USARTx, eeprom_config);
	//Address = DATA_EEPROM_START_ADDR;
	//packet_eeprom_save_configuration(&Address, eeprom_config); //rewrite eeprom with newly discovered sensors.

	packet_parser_init(&eeprom_parser);

	while(1){
		uint16_t Data;
		packet_eeprom_t packet_result;

		/*Only get a character if it is ready*/
		if(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) != RESET){

			Data = USART_ReceiveData(USARTx); // Collect Char

			while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET); // Wait for Empty

			//USART_SendData(USARTx, Data); // Echo Char
			//Address = DATA_EEPROM_START_ADDR;
			packet_result = packet_eeprom_parser((uint8_t) Data, &eeprom_parser);

			if (packet_result == PKT_SUCCESS){
				uint8_t dataBuffer[100];
				uint16_t pos = 0;

				if (eeprom_parser.packet.id == 0x00){
					if (eeprom_parser.packet.sub_id == 0x00){
						//switch address to 0x00
						Address = DATA_EEPROM_START_ADDR;
					}
					if (eeprom_parser.packet.sub_id == 0x01){
						//write each of these packets to eeprom, incrementing address
						packet_eeprom_prepare_packet(&eeprom_parser,dataBuffer, &pos);
						packet_eeprom_write(dataBuffer,pos, &Address);
					}
					if (eeprom_parser.packet.sub_id == 0x02){
						packet_eeprom_t result;
						//read all stored packets in eeprom
						Address = DATA_EEPROM_START_ADDR;
						memset(dataBuffer, 0, sizeof(dataBuffer));
						packet_parser_init(&eeprom_parser);

						//loop through eeprom here
						while (1){
							result = packet_eeprom_read_packet(&Address, &eeprom_parser);

							if (result == PKT_NO_MORE_PACKETS){
								break;
							}
							packet_eeprom_prepare_packet(&eeprom_parser,dataBuffer, &pos);
							//packet_eeprom_write(dataBuffer,pos, &Address);
							uart_OutBuffer(USARTx, dataBuffer,pos);
						}
					}
					if (eeprom_parser.packet.sub_id == 0x03){
						//line feed on output
						uart_OutString(USARTx, "\r\n");
					}
					if (eeprom_parser.packet.sub_id == 0x04){
						//clear last position
						packet_eeprom_write_empty(&Address);
					}
					if (eeprom_parser.packet.sub_id == 0x05){
						uint8_t i;
						//packet embedded within this packet
						eeprom_parser.packet.id = eeprom_parser.packet.payload[0];
						eeprom_parser.packet.sub_id = eeprom_parser.packet.payload[1];
						//shift packet down by 2 bytes
						for (i = 2; i < eeprom_parser.packet.length; i++ ){
							eeprom_parser.packet.payload[i-2] = eeprom_parser.packet.payload[i];
						}
						eeprom_parser.packet.length = eeprom_parser.packet.length - 2;

						//prepare packet and write it
						packet_eeprom_prepare_packet(&eeprom_parser,dataBuffer, &pos);
						packet_eeprom_write(dataBuffer,pos, &Address);
					}
					if (eeprom_parser.packet.sub_id == 0x06){ //redo init
						//Read eeprom settings to determine features
						packet_eeprom_init_config(&eeprom_config);
						packet_parser_init(&eeprom_parser);
						packet_eeprom_load_configuration(&eeprom_config);
						//packet_eeprom_print_configuration(USARTx, eeprom_config);

						//initialize corresponding GPIOs
						init_setup_configuration(eeprom_config);

						//scan for new hardware not described in config (new sensors)
						init_search_new_hardware(&eeprom_config);
						packet_eeprom_print_configuration(USARTx, eeprom_config);

						Address = DATA_EEPROM_START_ADDR;
						packet_eeprom_save_configuration(&Address, eeprom_config); //rewrite eeprom with newly discovered sensors.

						packet_parser_init(&eeprom_parser);

						role_init(eeprom_config);
					}
					if (eeprom_parser.packet.sub_id == 0x07){

						if (eeprom_config.rtc.i2c_rtc_enabled){
							if (eeprom_config.i2c[eeprom_config.rtc.i2c_rtc_number].type == I2C_TYPE_RTC_DS3231){
								uint8_t response;
								float temperature;
								init_3v3RegOn(true);
								delayms(10);

								response = i2c_ds3231_read_temperature((eeprom_config.i2c[eeprom_config.rtc.i2c_rtc_number].bus >> 1),&temperature);
								if (response == 0){
									char buffer[30];
									printSerial(USARTx, "DS3231 Present, current temperature %f,", temperature);
									eeprom_config.i2c[eeprom_config.rtc.i2c_rtc_number].enabled = 1;
								}else{
									eeprom_config.i2c[eeprom_config.rtc.i2c_rtc_number].enabled = 0;
								}

								init_3v3RegOn(false);
							}
						}
					}
					if (eeprom_parser.packet.sub_id == 0x08){
						if (eeprom_config.rtc.i2c_rtc_enabled){
							DS3231_time_t time;
							DS3231_date_t date;
							uint8_t result;
							//set RTC date
							init_3v3RegOn(true);
							delayms(500);

							date.year = eeprom_parser.packet.payload[0];
							date.month = eeprom_parser.packet.payload[1];
							date.date = eeprom_parser.packet.payload[2];
							date.day = eeprom_parser.packet.payload[3]; //1 to 7 day of week

							time.is_ampm_time = eeprom_parser.packet.payload[4];
							time.am_pm = eeprom_parser.packet.payload[5];
							time.hours = eeprom_parser.packet.payload[6];
							time.minutes = eeprom_parser.packet.payload[7];
							time.seconds = eeprom_parser.packet.payload[8];

							result = i2c_ds3231_set_time_date(eeprom_config.i2c[eeprom_config.rtc.i2c_rtc_number].bus>>1, time, date);

							delayms(500);

							//Read it back
							i2c_ds3231_read_time_date(eeprom_config.i2c[eeprom_config.rtc.i2c_rtc_number].bus>>1, &time, &date);
							printSerial(USARTx, "DS3231 Date and Time ");
							printSerial(USARTx, " YY:MM:DD HH-MM-SS %02d:%02d:%02d %02d-%02d-%02d\r\n",
									date.year, date.month, date.date, time.hours, time.minutes, time.seconds);

							init_3v3RegOn(false);
						}
					}
					if (eeprom_parser.packet.sub_id == 0x09){
						DS3231_time_t time;
						DS3231_date_t date;

						init_3v3RegOn(true);
						delayms(10);
						i2c_ds3231_read_time_date(eeprom_config.i2c[eeprom_config.rtc.i2c_rtc_number].bus>>1, &time, &date);
						printSerial(USARTx, "DS3231 Date and Time ");
						printSerial(USARTx, " YY:MM:DD HH-MM-SS %02d:%02d:%02d %02d-%02d-%02d\r\n",
								date.year, date.month, date.date, time.hours, time.minutes, time.seconds);

						init_3v3RegOn(false);
					}
				}

				//uart_OutBuffer(USARTx, dataBuffer, pos);
			}else if (packet_result == PKT_FAILURE){
				char buffer[40];
				sprintf(buffer,"CRC Fail, Calc %04x, Recv %04x\r\n",
						eeprom_parser.crc,
						(eeprom_parser.packet.crc[0] << 8) |eeprom_parser.packet.crc[1]);
				uart_OutString(USARTx, buffer);
			}
		}

		//Check role
		role_check(&eeprom_config);
	}
}
