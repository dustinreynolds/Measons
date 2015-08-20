/*
 * packet_00.c
 *
 *  Created on: Jul 6, 2015
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

#include "stdbool.h"
#include "stm32l1xx.h"
#include "i2c.h"
#include "packet_eeprom.h"
#include "packet_00.h"


void packet_00_print_config(USART_TypeDef * USARTx, config_t config){
	printSerial(USARTx, "Config Version: %04x %05d\r\n", config.version, config.version);

	printSerial(USARTx, "Role: %02x\r\n", config.role.role);
	if (config.role.role == ROLE_MCC){
		printSerial(USARTx, "Number of boxes: %d, size of each box: %d\r\n",config.role.mcc_num_mail_box, config.role.mcc_mail_box_size);
	}
	printSerial(USARTx, "Status LEDs: %01d %01d %01d %01d\r\n", config.status_leds.statusLed1Enabled,
			config.status_leds.statusLed2Enabled,
			config.status_leds.statusLed3Enabled,
			config.status_leds.statusLed4Enabled);

	if (config.nOW > 0) {
		uint8_t i = 0;
		for (i = 0; i < config.nOW; i++) {
			printSerial(USARTx, "Onewire device %d on Bus %d, enabled %d, type %d,",
					i, config.onewire[i].bus, config.onewire[i].enabled, config.onewire[i].type);
			printSerial(USARTx, " ROM ID: %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
					config.onewire[i].unique_id[0],
					config.onewire[i].unique_id[1],
					config.onewire[i].unique_id[2],
					config.onewire[i].unique_id[3],
					config.onewire[i].unique_id[4],
					config.onewire[i].unique_id[5],
					config.onewire[i].unique_id[6],
					config.onewire[i].unique_id[7]);
		}
	}
	if (config.nI2C > 0){
		uint8_t i = 0;
		for (i = 0; i < config.nI2C; i++){
			printSerial(USARTx, "I2C device %d on Bus %d, enabled %d, type %d, i2c addr %d\r\n",
					i,
					config.i2c[i].bus,
					config.i2c[i].enabled,
					config.i2c[i].type,
					config.i2c[i].i2c_id);
		}
	}
}

void packet_00_config(parser_t pkt, config_t * config){
	//set something...
	if (pkt.packet.sub_id == PKT_00_VERSION){
		//read configuration version
		config->version = (pkt.packet.payload[1] << 8) | pkt.packet.payload[0];
	}
	if (pkt.packet.sub_id == PKT_00_ROLE){
		config->role.role = pkt.packet.payload[0];

		//do role specific byte parsing here...
		if (config->role.role == ROLE_MCC){
			uint8_t i;
			config->role.mcc_num_mail_box = pkt.packet.payload[1];
			config->role.mcc_mail_box_size = pkt.packet.payload[2];
			//now allocate correct number of mail boxes
			for (i = 0; i < config->role.mcc_num_mail_box; i++){
				if (config->role.mcc_mail[i] != 0){
					//no need to allocate memory
				}else {
					//malloc memory according to size
					config->role.mcc_mail[i] = malloc(config->role.mcc_mail_box_size);
				}
			}
		}
	}
	if (pkt.packet.sub_id == PKT_00_STATUS_LEDS){
		config->status_leds.statusLed1Enabled = (pkt.packet.payload[0] > 0);
		config->status_leds.statusLed2Enabled = (pkt.packet.payload[1] > 0);
		config->status_leds.statusLed3Enabled = (pkt.packet.payload[2] > 0);
		config->status_leds.statusLed4Enabled = (pkt.packet.payload[3] > 0);
	}
	if (pkt.packet.sub_id == PKT_00_ONEWIRE){
		uint8_t i;
		if (config->nOW < OW_MAX_SENSORS){
			config->onewire[config->nOW].bus = pkt.packet.payload[0];
			config->onewire[config->nOW].enabled = 0;
			config->onewire[config->nOW].type = pkt.packet.payload[2];
			for (i = 0; i < 8; i++){
				config->onewire[config->nOW].unique_id[i] = pkt.packet.payload[3+i];
			}
			config->nOW++;
		}
	}
	if (pkt.packet.sub_id == PKT_00_I2C){
		if (config->nI2C < I2C_MAX_SENSORS){
			config->i2c[config->nI2C].bus = pkt.packet.payload[0];
			config->i2c[config->nI2C].type = pkt.packet.payload[1];
			config->i2c[config->nI2C].i2c_id = pkt.packet.payload[2];
			config->i2c[config->nI2C].enabled = 0; /* Device is disabled until verified*/
		}
		config->nI2C++;
	}
}

eeprom_00_error_t packet_00_make_packet(eeprom_id_00_t pktSubid, uint8_t deviceNum, parser_t * pkt, config_t config){
	uint8_t i = 0;
	pkt->packet.id = 0x00;
	pkt->packet.sub_id = pktSubid;
	if (pktSubid == PKT_00_VERSION){
		pkt->packet.payload[i++] = config.version & 0xFF;
		pkt->packet.payload[i++] = config.version >> 8;
	}else if (pktSubid == PKT_00_ROLE){
		pkt->packet.payload[i++] = config.role.role;
		pkt->packet.payload[i++] = config.role.mcc_num_mail_box;
		pkt->packet.payload[i++] = config.role.mcc_mail_box_size;
	}else if (pktSubid == PKT_00_STATUS_LEDS){
		pkt->packet.payload[i++] = config.status_leds.statusLed1Enabled;
		pkt->packet.payload[i++] = config.status_leds.statusLed2Enabled;
		pkt->packet.payload[i++] = config.status_leds.statusLed3Enabled;
		pkt->packet.payload[i++] = config.status_leds.statusLed4Enabled;
	}else if (pktSubid == PKT_00_ONEWIRE){
		uint8_t j = 0;
		if (deviceNum > config.nOW){
			return ERROR_00_INVALID_DEVICE_NUM;
		}
		pkt->packet.payload[i++] = config.onewire[deviceNum].bus;
		pkt->packet.payload[i++] = config.onewire[deviceNum].enabled;
		pkt->packet.payload[i++] = config.onewire[deviceNum].type;
		for (j = 0; j < 8; j++){
			pkt->packet.payload[i++] = config.onewire[deviceNum].unique_id[j];
		}
	}else if (pktSubid == PKT_00_I2C){
		if (deviceNum > config.nI2C){
			return ERROR_00_INVALID_DEVICE_NUM;
		}
		pkt->packet.payload[i++] = config.i2c[deviceNum].bus;
		pkt->packet.payload[i++] = config.i2c[deviceNum].type;
		pkt->packet.payload[i++] = config.i2c[deviceNum].i2c_id;
	}else {
		return ERROR_00_INVALID_SUB_ID;
	}
	pkt->packet.length = i;
	return ERROR_00_NO_ERROR;
}

//this function will save this config structure to EEPROM
eeprom_00_error_t packet_00_save_config_to_eeprom(uint32_t * Address, config_t config){
	uint8_t j = 0;
	eeprom_00_error_t error;
	parser_t packet;
	uint8_t buffer[100];
	uint16_t pos;
	for(j = 0; j < 255; j++){
		if (j == PKT_00_ONEWIRE){
			continue;
		}
		if (j == PKT_00_I2C){
			continue;
		}
		error = packet_00_make_packet(j,0,&packet,config);
		if (error != ERROR_00_NO_ERROR){
			continue;
		}
		packet_eeprom_prepare_packet(&packet,buffer,&pos);
		packet_eeprom_write(buffer,pos, Address);
	}
	//handle configuration which is spread over multiple packets
	for (j = 0; j < config.nOW; j++){
		if (config.onewire[j].enabled == 0){
			continue;// don't save sensors that are no longer present
		}
		error = packet_00_make_packet(PKT_00_ONEWIRE,j,&packet,config);
		if (error != ERROR_00_NO_ERROR){
			return error;
		}
		packet_eeprom_prepare_packet(&packet,buffer,&pos);
		packet_eeprom_write(buffer,pos, Address);
	}

	for (j = 0; j < config.nI2C; j++){
		error = packet_00_make_packet(PKT_00_I2C, j, &packet, config);
		if (error != ERROR_00_NO_ERROR){
			return error;
		}
		packet_eeprom_prepare_packet(&packet,buffer, &pos);
		packet_eeprom_write(buffer,pos, Address);
	}

	//finish it by writing an empty packet
	packet_eeprom_write_empty(Address);
	return ERROR_00_NO_ERROR;
}
