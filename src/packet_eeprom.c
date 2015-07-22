/*
 * packet_eeprom.c
 *
 *  Created on: Jun 28, 2015
 *      Author: dustin
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
#include "string.h"
#include "stdbool.h"
#include "stm32l1xx.h"
#include "packet_eeprom.h"
#include "crc_16bit_algorithm_dnp.h"
#include "uart.h"
#include "packet_00.h"

#define DATA_EEPROM_START_ADDR     0x08080000
#define DATA_EEPROM_END_ADDR       0x080803FF
#define DATA_EEPROM_PAGE_SIZE      0x8
#define DATA_32                    0x12345678
#define FAST_DATA_32               0x55667799

void packet_eeprom_init_config(config_t * config){
	memset(config, 0, sizeof(config_t));
}

void packet_eeprom_print_configuration(USART_TypeDef * USARTx, config_t config){
	packet_00_print_config(USARTx, config);
}

void packet_eeprom_set_configuration(parser_t pkt, config_t * config){
	if (pkt.packet.id == 0x00){
		packet_00_config(pkt,config);
	}
}

void packet_eeprom_save_configuration(uint32_t * Address, config_t config){
	packet_00_save_config_to_eeprom(Address, config);
}

void packet_eeprom_load_configuration(config_t * config){
	parser_t eeprom_parser;
	packet_eeprom_t result;
	uint32_t Address;

	//read all stored packets in eeprom
	Address = DATA_EEPROM_START_ADDR;
	packet_parser_init(&eeprom_parser);

	//loop through eeprom here
	while (1){
		result = packet_eeprom_read_packet(&Address, &eeprom_parser);

		if (result == PKT_NO_MORE_PACKETS){
			break;
		}

		//Do something with each packet here
		packet_eeprom_set_configuration(eeprom_parser, config);
	}
}

void _packet_find_crc_with_stx(USART_TypeDef * USARTx){
	parser_t dummy;
	uint32_t i;
	uint16_t dumpos;
	uint8_t dataBuffer[100];

	for (i = 0; i < 10000; i++){
		dummy.packet.id = 0xD0;
		dummy.packet.sub_id = 0x05;
		dummy.packet.length = 0x04;
		dummy.packet.payload[3] = i & 0xFF;
		dummy.packet.payload[2] = (i >> 8) & 0xFF;
		dummy.packet.payload[1] = (i>>16) & 0xFF;
		dummy.packet.payload[0] = (i>>24) & 0xFF;

		packet_eeprom_prepare_packet(&dummy, dataBuffer,&dumpos);
		if (((dummy.crc & 0xFF) == 0xCA) | ((dummy.crc & 0xFF) == 0xC0)){
			uart_OutBuffer(USARTx, dataBuffer, dumpos);
			//break;
		}else if (((dummy.crc & 0xFF00) == 0xCA00) | ((dummy.crc & 0xFF00) == 0xC000)){
			uart_OutBuffer(USARTx, dataBuffer, dumpos);
			//break;
		}
	}
}

void packet_eeprom_byte_stuff(uint8_t byte, uint8_t * buffer, uint16_t * index){
	if (byte == STX || byte == ESC){
		buffer[*index] = ESC;
		*index = *index + 1;
		buffer[*index] = byte ^ ESC;
		*index = *index + 1;
	}else{
		buffer[*index] = byte;
		*index = *index + 1;
	}
}

packet_eeprom_t packet_eeprom_read_packet(uint32_t *address, parser_t * details){
	uint16_t i, j = 0;
	bool keep_processing = true;
	packet_eeprom_t packet_result;
	union {
		uint32_t u32;
		uint8_t u8[4];
	} data;

	data.u32 = *(__IO uint32_t*)*address;
	if (data.u8[0] != STX){
		return PKT_NO_MORE_PACKETS;
	}

	//feed each byte into packet parser. finished when encounter STX or when
	//packet is good.

	while (keep_processing){
		data.u32 = *(__IO uint32_t*)*address;
		for (i = 0; i < 4; i++){
			packet_result = packet_eeprom_parser(data.u8[i], details);

			if (packet_result == PKT_SUCCESS){
				*address = *address + 4;

				return PKT_SUCCESS;
			}else if (packet_result == PKT_FAILURE){
				return PKT_FAILURE;
			}
		}

		*address = *address + 4;

		j += 4;
		if (j > MAX_PAYLOAD_SIZE){
			return PKT_FAILURE;
		}
	}
	return PKT_FAILURE;
}

void packet_eeprom_write_empty(const uint32_t *address){
	uint32_t data = 0x00000000;
	__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;

	//enable flash writes here
	DATA_EEPROM_Unlock();

	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
				| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);

	do{
		FLASHStatus = DATA_EEPROM_ProgramWord(*address, data);

		if(FLASHStatus == FLASH_COMPLETE){
			//*address = *address;
			//delayms(1);
		}else{
			FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
					| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR);
		}
	}while (FLASHStatus != FLASH_COMPLETE);
	DATA_EEPROM_Lock();
}

void packet_eeprom_write(uint8_t * buffer, uint16_t size, uint32_t *address){
	uint32_t data;
	uint16_t i, j;
	__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;

	//enable flash writes here
	DATA_EEPROM_Unlock();

	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
				| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);

	//handle all 4 byte multiples
	for (i = 0, j = 0; i < (size/4); i++){
		data = (buffer[j+3] << 24) | (buffer[j+2] << 16) | (buffer[j+1] << 8) | buffer[j];

		do{
			FLASHStatus = DATA_EEPROM_ProgramWord(*address, data);

			if(FLASHStatus == FLASH_COMPLETE){
				*address = *address + 4;
			}else{
				FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
						| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR);
			}
		}while (FLASHStatus != FLASH_COMPLETE);
		j += 4;
	}
	//handle remainder
	if (j < size){
		data = buffer[j++];
		if (j<size){
			data = data | (buffer[j++] << 8);
			if(j<size){
				data = data | (buffer[j++] << 16);
				if(j<size){
					data = data | (buffer[j++] << 24);
				}
			}
		}
		do{
			FLASHStatus = DATA_EEPROM_ProgramWord(*address, data);

			if(FLASHStatus == FLASH_COMPLETE){
				*address = *address + 4;
			}else{
				FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
						| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR);
			}
		}while (FLASHStatus != FLASH_COMPLETE);
	}

	DATA_EEPROM_Lock();
}

void packet_eeprom_prepare_packet(parser_t * details, uint8_t * sendBuffer, uint16_t * pos){
	uint8_t i = 0;
	*pos = 0;

	//  Need to Stuff bytes
	sendBuffer[*pos] = STX;
	*pos = *pos +1;

	packet_eeprom_byte_stuff(details->packet.id, sendBuffer, pos);
	packet_eeprom_byte_stuff(details->packet.sub_id, sendBuffer, pos);
	packet_eeprom_byte_stuff(details->packet.length, sendBuffer, pos);

	for (i = 0; i < details->packet.length; i++){
		packet_eeprom_byte_stuff(details->packet.payload[i], sendBuffer, pos);
	}
	// Calculate CRC on stuffed bytes
	details->crc = crc_16bit_algorithm_dnp_calculate_full(sendBuffer, *pos, 0xFFFF, 0xFFFF);

	// Stuff CRC if needed
	packet_eeprom_byte_stuff((details->crc >> 8),sendBuffer, pos);
	packet_eeprom_byte_stuff((details->crc & 0xFF), sendBuffer, pos);
}

void packet_parser_init(parser_t * details){
	details->state = PKT_HEADER;
	details->byte_stuff = false;
}

packet_eeprom_t packet_eeprom_parser(uint8_t byte, parser_t * details){

	if (byte == STX){
		details->crc = crc_16bit_algorithm_dnp_update(0xFFFF, byte);
		details->packet.stx = STX;
		details->state = PKT_ID;
		return details->state;
	}

	//Add byte to current crc
	if (details->state > PKT_HEADER && details->state < PKT_CRC1){
		details->crc = crc_16bit_algorithm_dnp_update(details->crc, byte);
	}

	//Do byte stuffing
	if(details->byte_stuff){
		if (byte == ESC_NULL || byte == ESC_A){
			byte = ESC | byte;
			details->byte_stuff = false;
		}else{
			//Invalid case!
			details->state = PKT_HEADER;
			details->byte_stuff = false;
			return details->state;
		}
	}else if (byte == ESC){
		details->byte_stuff = true;
		return details->state;
	}

	//Process state
	if (details->state == PKT_ID){
		details->packet.id = byte;
		details->state = PKT_SUB_ID;
	}else if (details->state == PKT_SUB_ID){
		details->packet.sub_id = byte;
		details->state = PKT_LENGTH;
	}else if (details->state == PKT_LENGTH){
		details->received_length = 0;
		details->packet.length = byte;
		if (details->packet.length == 0x00){
			details->state = PKT_CRC1;
		}else{
			details->state = PKT_PAYLOAD;
		}
	}else if (details->state == PKT_PAYLOAD){
		details->packet.payload[details->received_length++] = byte;
		if (details->received_length == details->packet.length){
			details->state = PKT_CRC1;
		}
	}else if (details->state == PKT_CRC1){
		details->packet.crc[0] = byte;
		details->state = PKT_CRC2;
	}else if (details->state == PKT_CRC2){
		details->packet.crc[1] = byte;
		details->crc = details->crc ^ 0xFFFF;

		if (details->crc == ((details->packet.crc[0] << 8) | details->packet.crc[1])){
			details->state = PKT_SUCCESS;
		}else{
			details->state = PKT_FAILURE;
		}
	}else{
		details->state = PKT_HEADER;
	}
	return details->state;
}
