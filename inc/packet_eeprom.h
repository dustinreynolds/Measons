/*
 * packet_eeprom.h
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

#ifndef PACKET_EEPROM_H_
#define PACKET_EEPROM_H_

#include "stdbool.h"
#include "onewire.h"
#include "i2c.h"

// Bunch of packets with CA or C0 in CRC, with byte stuffing
// CA D0 05 04 00 00 00 A5 C0 0A 3A CA D0 05 04 00 00 00 EB C0 00 93 CA D0 05 04 00 00 01 31 C6 C0 0A CA D0 05 04 00 00 01 64 96 C0 00 CA D0 05 04 00 00 01 B5 C0 0A 1B CA D0 05 04 00 00 01 B8 67 C0 0A CA D0 05 04 00 00 01 ED 37 C0 00 CA D0 05 04 00 00 01 FB C0 00 B2 CA D0 05 04 00 00 02 85 C0 0A 78 CA D0 05 04 00 00 02 CB C0 00 D1 CA D0 05 04 00 00 03 04 52 C0 0A CA D0 05 04 00 00 03 51 02 C0 00 CA D0 05 04 00 00 03 8D F3 C0 0A CA D0 05 04 00 00 03 95 C0 0A 59 CA D0 05 04 00 00 03 D8 A3 C0 00 CA D0 05 04 00 00 03 DB C0 00 F0 CA D0 05 04 00 00 04 1C FC C0 00 CA D0 05 04 00 00 04 49 AC C0 0A CA D0 05 04 00 00 04 95 5D C0 00 CA D0 05 04 00 00 04 AB C0 00 17 CA D0 05 04 00 00 04 C0 0A 43 C0 0A CA D0 05 04 00 00 04 E5 C0 0A BE CA D0 05 04 00 00 05 BB C0 00 36 CA D0 05 04 00 00 05 F5 C0 0A 9F CA D0 05 04 00 00 06 29 68 C0 00 CA D0 05 04 00 00 06 7C 38 C0 0A CA D0 05 04 00 00 06 8B C0 00 55 CA D0 05 04 00 00 06 A0 C9 C0 00 CA D0 05 04 00 00 06 C5 C0 0A FC CA D0 05 04 00 00 06 F5 99 C0 0A CA D0 05 04 00 00 07 9B C0 00 74 CA D0 05 04 00 00 07 D5 C0 0A DD CA D0 05 04 00 00 08 24 C0 0A 32 CA D0 05 04 00 00 08 6A C0 00 9B CA D0 05 04 00 00 09 28 77 C0 00 CA D0 05 04 00 00 09 34 C0 0A 13 CA D0 05 04 00 00 09 7A C0 00 BA CA D0 05 04 00 00 09 7D 27 C0 0A CA D0 05 04 00 00 09 A1 D6 C0 00 CA D0 05 04 00 00 09 F4 86 C0 0A CA D0 05 04 00 00 0A 04 C0 0A 70 CA D0 05 04 00 00 0A 4A C0 00 D9 CA D0 05 04 00 00 0B 14 C0 0A 51 CA D0 05 04 00 00 0B 1D E3 C0 00 CA D0 05 04 00 00 0B 48 B3 C0 0A CA D0 05 04 00 00 0B 5A C0 00 F8 CA D0 05 04 00 00 0B 94 42 C0 00 CA D0 05 04 00 00 0B C1 12 C0 0A CA D0 05 04 00 00 0C 05 4D C0 0A CA D0 05 04 00 00 0C 2A C0 00 1F CA D0 05 04 00 00 0C 50 1D C0 00 CA D0 05 04 00 00 0C 64 C0 0A B6 CA D0 05 04 00 00 0C 8C EC C0 0A CA D0 05 04 00 00 0C D9 BC C0 00 CA D0 05 04 00 00 0D 3A C0 00 3E CA D0 05 04 00 00 0D 74 C0 0A 97 CA D0 05 04 00 00 0E 0A C0 00 5D CA D0 05 04 00 00 0E 30 D9 C0 0A CA D0 05 04 00 00 0E 44 C0 0A F4 CA D0 05 04 00 00 0E 65 89 C0 00 CA D0 05 04 00 00 0E B9 78 C0 0A CA D0 05 04 00 00 0E EC 28 C0 00 CA D0 05 04 00 00 0F 1A C0 00 7C CA D0 05 04 00 00 0F 54 C0 0A D5 CA D0 05 04 00 00 10 32 E7 C0 0A CA D0 05 04 00 00 10 67 B7 C0 00 CA D0 05 04 00 00 10 B7 C0 0A 0B CA D0 05 04 00 00 10 BB 46 C0 0A CA D0 05 04 00 00 10 EE 16 C0 00 CA D0 05 04 00 00 10 F9 C0 00 A2 CA D0 05 04 00 00 11 A7 C0 0A 2A CA D0 05 04 00 00 11 E9 C0 00 83 CA D0 05 04 00 00 12 07 73 C0 0A CA D0 05 04 00 00 12 52 23 C0 00 CA D0 05 04 00 00 12 8E D2 C0 0A CA D0 05 04 00 00 12 97 C0 0A 49 CA D0 05 04 00 00 12 C0 00 C0 0A 9A CA D0 05 04 00 00 12 D9 C0 00 E0 CA D0 05 04 00 00 12 DB 82 C0 00 CA D0 05 04 00 00 13 87 C0 0A 68 CA D0 05 04 00 00 13 C9 C0 00 C1 CA D0 05 04 00 00 14 B9 C0 00 26 CA D0 05 04 00 00 14 F7 C0 0A 8F CA D0 05 04 00 00 15 1F DD C0 00 CA D0 05 04 00 00 15 4A 8D C0 0A CA D0 05 04 00 00 15 96 7C C0 00 CA D0 05 04 00 00 15 A9 C0 00 07 CA D0 05 04 00 00 15 C3 2C C0 0A CA D0 05 04 00 00 15 E7 C0 0A AE CA D0 05 04 00 00 16 99 C0 00 64 CA D0 05 04 00 00 16 D7 C0 0A CD CA D0 05 04 00 00 17 2A 49 C0 00 CA D0 05 04 00 00 17 7F 19 C0 0A CA D0 05 04 00 00 17 89 C0 00 45 CA D0 05 04 00 00 17 A3 E8 C0 00 CA D0 05 04 00 00 17 C7 C0 0A EC CA D0 05 04 00 00 17 F6 B8 C0 0A CA D0 05 04 00 00 18 2B 56 C0 00 CA D0 05 04 00 00 18 36 C0 0A 03 CA D0 05 04 00 00 18 78 C0 00 AA CA D0 05 04 00 00 18 7E 06 C0 0A CA D0 05 04 00 00 18 A2 F7 C0 00 CA D0 05 04 00 00 18 F7 A7 C0 0A CA D0 05 04 00 00 19 26 C0 0A 22 CA D0 05 04 00 00 19 68 C0 00 8B CA D0 05 04 00 00 1A 16 C0 0A 41 CA D0 05 04 00 00 1A 1E C2 C0 00 CA D0 05 04 00 00 1A 4B 92 C0 0A CA D0 05 04 00 00 1A 58 C0 00 E8 CA D0 05 04 00 00 1A 97 63 C0 00 CA D0 05 04 00 00 1A C2 33 C0 0A CA D0 05 04 00 00 1B 06 C0 0A 60 CA D0 05 04 00 00 1B 48 C0 00 C9 CA D0 05 04 00 00 1C 38 C0 00 2E CA D0 05 04 00 00 1C 76 C0 0A 87 CA D0 05 04 00 00 1D 06 6C C0 0A CA D0 05 04 00 00 1D 28 C0 00 0F CA D0 05 04 00 00 1D 53 3C C0 00 CA D0 05 04 00 00 1D 66 C0 0A A6 CA D0 05 04 00 00 1D 8F CD C0 0A CA D0 05 04 00 00 1D DA 9D C0 00 CA D0 05 04 00 00 1E 18 C0 00 6C CA D0 05 04 00 00 1E 56 C0 0A C5 CA D0 05 04 00 00 1F 08 C0 00 4D CA D0 05 04 00 00 1F 33 F8 C0 0A CA D0 05 04 00 00 1F 46 C0 0A E4 CA D0 05 04 00 00 1F 66 A8 C0 00 CA D0 05 04 00 00 1F BA 59 C0 0A CA D0 05 04 00 00 1F EF 09 C0 00 CA D0 05 04 00 00 20 81 C0 0A 58 CA D0 05 04 00 00 20 CF C0 00 F1 CA D0 05 04 00 00 21 02 10 C0 0A CA D0 05 04 00 00 21 57 40 C0 00 CA D0 05 04 00 00 21 8B B1 C0 0A CA D0 05 04 00 00 21 91 C0 0A 79 CA D0 05 04 00 00 21 DE E1 C0 00 CA D0 05 04 00 00 21 DF C0 00 D0 CA D0 05 04 00 00 22 A1 C0 0A 1A CA D0 05 04 00 00 22 EF C0 00 B3 CA D0 05 04 00 00 23 37 84 C0 0A CA D0 05 04 00 00 23 62 D4 C0 00 CA D0 05 04 00 00 23 B1 C0 0A 3B CA D0 05 04 00 00 23 BE 25 C0 0A CA D0 05 04 00 00 23 EB 75 C0 00 CA D0 05 04 00 00 23 FF C0 00 92 CA D0 05 04 00 00 24 2F 2A C0 00 CA D0 05 04 00 00 24 7A 7A C0 0A CA D0 05 04 00 00 24 8F C0 00 75 CA D0 05 04 00 00 24 A6 8B C0 00 CA D0 05 04 00 00 24 C1 C0 0A DC CA D0 05 04 00 00 24 F3 DB C0 0A CA D0 05 04 00 00 25 9F C0 00 54 CA D0 05 04 00 00 25 D1 C0 0A FD CA D0 05 04 00 00 26 1A BE C0 00 CA D0 05 04 00 00 26 4F EE C0 0A CA D0 05 04 00 00 26 93 1F C0 00 CA D0 05 04 00 00 26 AF C0 00 37 CA D0 05 04 00 00 26 C6 4F C0 0A CA D0 05 04 00 00 26 E1 C0 0A 9E CA D0 05 04 00 00 00 A5 C0 0A 3A

#define STX			0xCA
#define ESC			0xC0
#define ESC_NULL	0x00
#define ESC_A		0x0A

#define MAX_PAYLOAD_SIZE    80

//MCC specific settings
#define MAX_NUM_POBOX	20

typedef enum {
	ROLE_UNCONFIGURED = 0x00,
	ROLE_GENERIC	  = 0x01,
	ROLE_MCC		  = 0x02,
	ROLE_GPS		  = 0x03
} roole_type_t;

typedef struct {
	uint8_t statusLed1Enabled;
	uint8_t statusLed2Enabled;
	uint8_t statusLed3Enabled;
	uint8_t statusLed4Enabled;
} config_led_t;

typedef struct {
	uint8_t bus;           /* Onewire Bus, 1, 2, or 3. 0 is invalid*/
	uint8_t enabled;       /* MCU has verified device currently on bus */
	uint8_t type;          /* Onewire device type, 0x00 temp*/
	uint8_t unique_id[8];  /* Onewire unique ID*/
} config_onewire_t;

typedef struct {
	uint8_t enabled; /* MCU has verified device on bus */
	uint8_t bus;     /* i2c bus 1 or 2, 0 is invalid */
	uint8_t type;    /* i2c device type (RTC, EEPROM, LCD*/
	uint8_t i2c_id;  /* i2c address*/
} config_i2c_t;

typedef struct {
	uint8_t i2c_rtc_enabled; /* true if there is an active I2C RTC */
	uint8_t i2c_rtc_number; /* offset of rtc in config.i2c structure */
} config_rtc_t;

typedef struct {
	uint8_t role;
	/* MCC specific Role definitions*/
	uint8_t mcc_num_mail_box;
	uint8_t mcc_mail_box_size;
	uint8_t* mcc_mail[MAX_NUM_POBOX]; //Maximum number of pointers to mailboxes
} role_t;

typedef struct {
	uint16_t version; /* Configuration version*/
	role_t role;      /* Meason role*/
	config_led_t status_leds; /* Status LED struct determines which are available */
	uint8_t nOW; /* Number of onewire devices configured */
	config_onewire_t onewire[OW_MAX_SENSORS]; /* onewire specific information */
	uint8_t nI2C; /* Number of i2c devices */
	config_rtc_t rtc;
	config_i2c_t i2c[I2C_MAX_SENSORS]; /* i2c specific information */
} config_t;

typedef enum {
	PKT_HEADER = 0x00,
	PKT_ID,
	PKT_SUB_ID,
	PKT_LENGTH,
	PKT_PAYLOAD,
	PKT_CRC1,
	PKT_CRC2,
	PKT_SUCCESS,
	PKT_FAILURE,
	PKT_NO_MORE_PACKETS
} packet_eeprom_t;

typedef struct {
	uint8_t stx;
	uint8_t id;
	uint8_t sub_id;
	uint8_t length;
	uint8_t payload[MAX_PAYLOAD_SIZE];
	uint8_t crc[2];
} packet_t;

typedef struct {
	unsigned short crc;
	bool byte_stuff;
	uint8_t received_length;
	packet_eeprom_t state;
	packet_t packet;
} parser_t;

typedef enum {
	PKT_EEP_ID_VERSION = 0x00,
	PKT_EEP_ID_HW_TEMP = 0x01,
	PKT_EEP_ID_HW_GPS  = 0x02,
	PKT_EEP_ID_HW_MOISTURE = 0x03,
} pkt_eep_id_t;

void packet_eeprom_save_configuration(uint32_t * Address, config_t config);
void packet_eeprom_init_config(config_t * config);
void packet_eeprom_print_configuration(USART_TypeDef * USARTx, config_t config);
void packet_eeprom_load_configuration(config_t * config);
void packet_eeprom_write_empty(const uint32_t *address);
void packet_eeprom_write(uint8_t * buffer, uint16_t size, uint32_t *address);
uint8_t packet_eeprom_read_packet(uint32_t *address, parser_t * details);
void packet_eeprom_prepare_packet(parser_t * details, uint8_t * sendBuffer, uint16_t * pos);
void packet_parser_init(parser_t * details);
uint8_t packet_eeprom_parser(uint8_t byte, parser_t * details);

#endif /* PACKET_EEPROM_H_ */
