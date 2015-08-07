/*
 * packet_sx1231h.c
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

#include <stdint.h>
#include "packet_sx1231h.h"

packet_sx1231h_error_t packet_sx1231h_parse(uint8_t * buffer, uint8_t size, packet_sx1231h_t * p){
	uint8_t i, j;
	//We know exactly the byte order
	if (buffer[0] != PACKET_SX1231H_STX){
		return PACKET_SX1231H_STX_ERROR;
	}

	i = 0;
	p->stx = buffer[i++];
	p->id = buffer[i++];
	p->sub_id = buffer[i++];
	p->length = buffer[i++];
	p->sender_id = buffer[i++];

	if (p->length + PACKET_SX1231H_OVERHEAD > size){  //size or length incorrect
		return PACKET_SX1231H_SIZE_ERROR;
	}

	for (j = 0; j < p->length; j++){
		p->payload[j] = buffer[i++];
	}

	p->crc8[1] = buffer[i++];
	p->crc8[0] = buffer[i++];

	//check CRC
	//return PACKET_SX1231H_CRC_BAD;

	//if CRC good, return no error
	return PACKET_SX1231H_NO_ERROR;
}

void packet_sx1231h_fill_packet(uint8_t * buffer, uint8_t * index, packet_sx1231h_t * p){
	//fill in missing fields, calculate crc
	uint8_t i;
	uint8_t j = 0;
	*index = 0;

	p->stx = PACKET_SX1231H_STX;
	p->crc16 = crc_16bit_algorithm_dnp_update(0xFFFF, PACKET_SX1231H_STX);

	p->crc16 = crc_16bit_algorithm_dnp_update(p->crc16, p->id);
	p->crc16 = crc_16bit_algorithm_dnp_update(p->crc16, p->sub_id);
	p->crc16 = crc_16bit_algorithm_dnp_update(p->crc16, p->length);
	p->crc16 = crc_16bit_algorithm_dnp_update(p->crc16, p->sender_id);

	for (i = 0; i < p->length; i++){
		p->crc16 = crc_16bit_algorithm_dnp_update(p->crc16, p->payload[i]);
	}

	p->crc16 = p->crc16 ^ 0xFFFF;

	buffer[j++] = PACKET_SX1231H_STX;
	buffer[j++] = p->id;
	buffer[j++] = p->sub_id;
	buffer[j++] = p->length;
	buffer[j++] = p->sender_id;
	for (i = 0; i < p->length; i++){
		buffer[j++] =  p->payload[i];
	}
	buffer[j++] = p->crc8[1];
	buffer[j++] = p->crc8[0];
	*index = j;
}
