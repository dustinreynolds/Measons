/*
 * role.c
 *
 *  Created on: Jul 23, 2015
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

#include "spi.h"
#include "uart.h"
#include "timer.h"
#include "init.h"
#include "sx1231h.h"
#include "packet_sx1231h.h"
#include "packet_eeprom.h"
#include "role.h"

error_role_t role_init(config_t config){
	uint8_t result;

	//common init for all devices
	result = sx1231h_init(SPI_RFM69_1);
	if (result == ERROR){
		return ERROR_ROLE_RFM69_NOT_DETECTED;
	}

	//Do special rfm69 config based on config here...

	//special processing for each role
	if (config.role.role == ROLE_MCC){
		//setup RF for receive
		sx1231h_receiveFrameStart(SPI_RFM69_1);
	}else if (config.role.role == ROLE_GENERIC){
		//setup RF for transmit
		sx1231h_set_power(SPI_RFM69_1,17);
	}else {
		//setup RF for transmit
	}
	return ERROR_ROLE_NO_ERROR;
}

void role_check(config_t * config){
	//Perform main role
	if (config->role.role == ROLE_MCC){
		uint8_t result;
		uint8_t receiveBuff[100];
		uint8_t receiveSize;
		//listen for packets
		static uint16_t wait_timeout;
		static sx1231h_check_t rfm_status = SX1231H_CHECK_NOT_READY;
		packet_sx1231h_error_t packet_error;
		packet_sx1231h_t packet;

		if (wait_timeout == 0){
			sx1231h_receiveFrameStart(SPI_RFM69_1);
			wait_timeout++;
		}else if (wait_timeout < 1000){
			rfm_status = sx1231h_receiveWaitCheck(SPI_RFM69_1);

			if (rfm_status == SX1231H_CHECK_READY){
				//receive packet
				result = sx1231h_receiveFrameWait(SPI_RFM69_1, receiveBuff, &receiveSize);
				if (result != OK){
					printSerial(USART2, "MCC RX wait error %d\r\n",result);
					wait_timeout = 0;
					return;
				}

				packet_error = packet_sx1231h_parse(receiveBuff, receiveSize, &packet);

				if (packet_error != PACKET_SX1231H_NO_ERROR){
					//do something
					printSerial(USART2, "role_check Pkt error %d\r\n",packet_error);
					wait_timeout = 0;
					return;
				}

				//Now execute this received packet
				role_mcc_execute_packet(config, &packet);

				wait_timeout = 0;
			}
			wait_timeout++;
		}else{
			//timed out
			//printSerial(USART2, "role_check MCC timeout\r\n");
			wait_timeout = 0;
		}

	}
	else if (config->role.role == ROLE_GENERIC){
		static uint32_t wait_timeout;
		onewire_error_t ow_result;
		uint8_t result;
		uint8_t i = 0, j;
		//send packets

		//send temperature
		if (wait_timeout == 0){
			//send temperature
			init_3v3RegOn(true);
			delayms(200);

			for (i = 0; i < config->nOW; i++){
				uint8_t presence;
				uint8_t rom[8];
				uint16_t temperature = 0x0000;

//				//search for all active sensors (only check the temperature of present ones
//				presence = onewire_sendResetBasic((config->onewire[i].bus - 1));
//
//				if (presence == 0){
//					//no body home
//					config->onewire[i].enabled = 0;
//					continue;
//				}
//
//				result = OWFirst((config->onewire[i].bus-1));
//
//				config->onewire[i].enabled = 0;
//
//				while (result){
//					onewire_read_latest_ROM(&rom);
//
//					if (    (config->onewire[i].unique_id[0] == rom[0]) &&
//							(config->onewire[i].unique_id[1] == rom[1]) &&
//							(config->onewire[i].unique_id[2] == rom[2]) &&
//							(config->onewire[i].unique_id[3] == rom[3]) &&
//							(config->onewire[i].unique_id[4] == rom[4]) &&
//							(config->onewire[i].unique_id[5] == rom[5]) &&
//							(config->onewire[i].unique_id[6] == rom[6]) &&
//							(config->onewire[i].unique_id[7] == rom[7]) ){
//						//exact match on this bus, we knew about this device before.
//						config->onewire[i].enabled = 1; //break out of the for loop, scan for different ROM id
//					}
//					result = OWNext((config->onewire[i].bus-1));
//				}
//
				if (config->onewire[i].enabled == 0){
					continue;
				}

				//check temperature
				//onewire_trigger_temp(config->onewire[i].bus - 1);

				//ow_result = onewire_read_stored_temp((config->onewire[i].bus - 1), config->onewire[i].unique_id, &temperature);
				ow_result = onewire_read_temp((config->onewire[i].bus - 1), config->onewire[i].unique_id, &temperature);

				if (ow_result == ONEWIRE_NO_ERROR ){
					packet_sx1231h_t p;
					uint8_t sendBuffer[30];
					uint8_t sendSize;
					//prepare packet
					p.id = PACKET_SX1231H_ID_SENSOR;
					p.sub_id = PACKET_SX1231H_ID_00_SUB_ID_TEMPERATURE;
					p.sender_id = 01; //put config id here
					for (j = 0; j < 8; j++){
						p.payload[j] = config->onewire[i].unique_id[j];
					}
					p.payload[j++] = temperature >> 8;
					p.payload[j++] = temperature & 0xFF;

					p.length = j;

					packet_sx1231h_fill_packet(sendBuffer, &sendSize, &p);

					//transmit packet
					result = sx1231h_sendFrameStart(SPI_RFM69_1, sendBuffer, sendSize);
					if (result == ERROR){
						wait_timeout = 0;
					}
					//wait
					result = sx1231h_sendFrameWait(SPI_RFM69_1);
					if (result != OK){
						printSerial(USART2, "Failure to send %02d\r\n",result);
						break; //give up on sending this batch
					}
				}else{
					printSerial(USART2, "OneWire error %02d", ow_result);
					config->onewire[i].enabled = 0;
				}
			}
			init_3v3RegOn(false);
			wait_timeout++;
		}else if(wait_timeout < 1200000){ //change this to a RTC or timer based mechanism
			//do nothing
			wait_timeout++;
		}else {
			uint32_t Address;
			//packet_eeprom_init_config(config); //note that this wipes out the configuration
			//packet_eeprom_load_configuration(config);

			//init_setup_configuration(*config);

			init_search_new_hardware(config);

			//Address = 0x08080000;
			//packet_eeprom_save_configuration(&Address, *config);

			wait_timeout = 0;
		}

	}
}

void role_mcc_execute_packet(config_t * config, packet_sx1231h_t * packet){
	//do something with the packet
	if (packet->id == PACKET_SX1231H_ID_SENSOR){

		if (packet->id == PACKET_SX1231H_ID_00_SUB_ID_TEMPERATURE){
			DS3231_time_t time;
			DS3231_date_t date;
			float temp;
			if (config->rtc.i2c_rtc_enabled == 1){
				init_3v3RegOn(true);
				delayms(10);
				i2c_ds3231_read_time_date(config->i2c[config->rtc.i2c_rtc_number].bus>>1, &time, &date);
				init_3v3RegOn(false);
			}else{
				date.year = 0;
				date.month = 0;
				date.date = 0;
				date.day = 0;
				time.is_ampm_time = 0;
				time.am_pm = 0;
				time.hours = 0;
				time.minutes = 0;
				time.seconds = 0;
			}
			printSerial(USART2, "sender_id.packet_id.sub_id.TempID0.TempID1.TempID2.TempID3.TempID4.TempID5.TempID6.TempID7.Temp.YY.MM.DD.HH.MM.SS,");

			printSerial(USART2,"%02x,%02x,%02x,", packet->sender_id, packet->id, packet->sub_id);
			//display wireless temperature reading
			temp = (uint16_t)((uint16_t)packet->payload[8] << 8) | packet->payload[9];
			printSerial(USART2,"%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%03.3f,",
					packet->payload[0],
					packet->payload[1],
					packet->payload[2],
					packet->payload[3],
					packet->payload[4],
					packet->payload[5],
					packet->payload[6],
					packet->payload[7],
					(((float)temp/16.0)*9.0/5.0 + 32.0));
			printSerial(USART2, "%02d,%02d,%02d,%02d,%02d,%02d\r\n",
					date.year, date.month, date.date, time.hours, time.minutes, time.seconds);

		}
	}
}
