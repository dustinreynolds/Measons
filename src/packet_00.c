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
#include "packet_eeprom.h"
#include "packet_00.h"

void packet_00_config(parser_t pkt, config_t * config){
	//set something...
	if (pkt.packet.sub_id == PKT_00_VERSION){
		//read configuration version
		config->version = (pkt.packet.payload[1] << 8) | pkt.packet.payload[0];
	}
	if (pkt.packet.sub_id == PKT_00_ROLE){
		config->role = pkt.packet.payload[0];

		//do role specific byte parsing here...
	}
	if (pkt.packet.sub_id == PKT_00_STATUS_LEDS){
		config->status_leds.statusLed1Enabled = (pkt.packet.payload[0] > 0);
		config->status_leds.statusLed2Enabled = (pkt.packet.payload[1] > 0);
		config->status_leds.statusLed3Enabled = (pkt.packet.payload[2] > 0);
		config->status_leds.statusLed4Enabled = (pkt.packet.payload[3] > 0);
	}
	if (pkt.packet.sub_id == PKT_00_ONEWIRE){
		uint8_t i;
		if (config->number_onewire_devices <=255){
			config->onewire[config->number_onewire_devices].bus = pkt.packet.payload[0];
			config->onewire[config->number_onewire_devices].enabled = pkt.packet.payload[1];
			config->onewire[config->number_onewire_devices].type = pkt.packet.payload[2];
			for (i = 0; i < 8; i++){
				config->onewire[config->number_onewire_devices].unique_id[i] = pkt.packet.payload[3+i];
			}
			config->number_onewire_devices++;
		}
	}
}
