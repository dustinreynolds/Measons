/*
 * packet_sx1231h.h
 *
 *  Created on: Aug 3, 2015
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

#ifndef PACKET_SX1231H_H_
#define PACKET_SX1231H_H_

#include <stdint.h>
#include "stdbool.h"
#include "onewire.h"

#define PACKET_SX1231H_STX			0xCB
/*stx, id, subid, length, sender_id, crc[2] */
#define PACKET_SX1231H_OVERHEAD		6
#define MAX_SX1231H_PAYLOAD_SIZE    250

typedef struct {
	uint8_t stx;
	uint8_t id;
	uint8_t sub_id;
	uint8_t length; //includes sender_id + payload
	uint8_t sender_id; //who sent this packet!
	uint8_t payload[MAX_SX1231H_PAYLOAD_SIZE];
	union{
		uint16_t crc16;
		uint8_t crc8[2];
	};
} packet_sx1231h_t;

typedef enum {
	PACKET_SX1231H_NO_ERROR = 0x00,
	PACKET_SX1231H_STX_ERROR,
	PACKET_SX1231H_SIZE_ERROR,
	PACKET_SX1231H_CRC_BAD,
} packet_sx1231h_error_t;

typedef enum {
	PACKET_SX1231H_ID_MANAGEMENT 				= 0x00,
	PACKET_SX1231H_ID_SENSOR					= 0x01,
} packet_sx1231h_id_t;

typedef enum {
	//id PACKET_SX1231H_ID_SENSOR
	PACKET_SX1231H_ID_00_SUB_ID_TEMPERATURE			= 0x01,
};

packet_sx1231h_error_t packet_sx1231h_parse(uint8_t * buffer, uint8_t size, packet_sx1231h_t * packet);
void packet_sx1231h_fill_packet(uint8_t * buffer, uint8_t * index, packet_sx1231h_t * p);
#endif /* PACKET_SX1231H_H_ */
