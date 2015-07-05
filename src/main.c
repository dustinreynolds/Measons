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

typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
#define DATA_EEPROM_START_ADDR     0x08080000
#define DATA_EEPROM_END_ADDR       0x080803FF
#define DATA_EEPROM_PAGE_SIZE      0x8
#define DATA_32                    0x12345678
#define FAST_DATA_32               0x55667799

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;
__IO TestStatus DataMemoryProgramStatus = PASSED;
uint32_t NbrOfPage = 0, j = 0, Address = 0;

int main(void)
{
	USART_TypeDef * USARTx = USART2;
	parser_t eeprom_parser;

	//init_RCC_Configuration();

	init_HSI();

	timer_TIM2_Configuration();

	init_USART2();

	uart_Configuration(USARTx, UART_POLLING);
	uart_OutString(USARTx,"Welcome to Nucleo L152RE\r\n");

	//Read eeprom settings to determine features

	//initialize corresponding GPIOs

	//init_GPIO_Configuration();





	Address = DATA_EEPROM_START_ADDR;
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
			packet_result = packet_eeprom_parser(Data, &eeprom_parser);

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
							result = packet_eeprom_read_packet(USARTx, &Address, &eeprom_parser);

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
				}

				//uart_OutBuffer(USARTx, dataBuffer, pos);
			}else if (packet_result == PKT_FAILURE){
				char buffer[40];
				sprintf(buffer,"CRC Fail, Calc %04x, Recv %04x\r\n",
						eeprom_parser.crc,
						(eeprom_parser.packet.crc[1] << 8) |eeprom_parser.packet.crc[0]);
				uart_OutString(USARTx, buffer);
			}
		}
	}
}
