/*
 * packet_00.h
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

#ifndef PACKET_00_H_
#define PACKET_00_H_

typedef enum {
	PKT_00_VERSION = 0x00,
	PKT_00_ROLE = 0x01,
	PKT_00_STATUS_LEDS = 0x10,
	PKT_00_ONEWIRE = 0x11,
	PKT_00_PIEZO = 0x12,
	PKT_00_RGB_PORT = 0x13,
	PKT_00_I2C = 0x14,
	PKT_00_PC_SERIAL_LINK = 0x15,
	PKT_00_SWITCH_BUS = 0x16,
	PKT_00_RELAY = 0x17,
	PKT_00_WATER_SENSOR = 0x20,
	PKT_00_LIGHT_SENSOR = 0x21,
	PKT_00_PIR_MOTION = 0x22
}eeprom_id_00_t;

typedef enum {
	ERROR_00_NO_ERROR = 0x00,
	ERROR_00_INVALID_DEVICE_NUM = 0x01,
	ERROR_00_INVALID_SUB_ID,
}eeprom_00_error_t;

void packet_00_config(parser_t pkt, config_t * config);
eeprom_00_error_t packet_00_make_packet(eeprom_id_00_t pktSubid, uint8_t deviceNum, parser_t * pkt, config_t config);
eeprom_00_error_t packet_00_save_config_to_eeprom(uint32_t *Address, config_t config);
#endif /* PACKET_00_H_ */
