/*
 * OneWire.c
 *
 *  Created on: Apr 25, 2015
 *      Author: Dustin
 */


/*
 * Currently I'm working on implementing the state logic to handle the different state transistions and
 * doing things like reading the bus state 550us after reset (it's 70 us after master stops pulling down bus).
 *
 * With a solid interrupt driven library, the processor can go into sleep and get woken up to perform the next action,
 * if one wire sampling is needed.
 *
 * The below code generated some unexpected timing, so I'm working in timer to see how I can improve it.  Currently I'm using
 * the default crystal, which is the MSI at 2.097 MHz. HSI clock of 16 MHz is available and may be the best since  16,000,000/1,000,000 = 16 periods (1000000 is 1/1u)
 *
 * Currently the basic reset seems to mostly work, but it has inconsistent timings for the 240us phase. Fixed
 *
 * Now it seems that I loose debugging control with the current timer setup. Problem is more pronounced with longer delays. __WFI was to blame.
 *
 */
#include "stm32l1xx.h"
#include "string.h"

#include "onewire.h"
#include "timer.h"

// method declarations for Maxim 1-wire

int  OWReset(onewire_port_t OWx);
void OWWriteByte(onewire_port_t OWx,unsigned char byte_value);
void OWWriteBit(onewire_port_t OWx,unsigned char bit_value);
unsigned char OWReadBit(onewire_port_t OWx);
int  OWSearch(onewire_port_t OWx);
unsigned char docrc8(unsigned char value);

// global search state
unsigned char ROM_NO[8];
int LastDiscrepancy;
int LastFamilyDiscrepancy;
int LastDeviceFlag;
unsigned char crc8;

//GPIO_InitTypeDef gpioOW1;
//GPIO_InitTypeDef gpioOW2;
//GPIO_InitTypeDef gpioOW3;
//onewire_OWn_t onewire_OW3;
OW_Device_t OW_devices[3];

//NVIC_InitTypeDef nvicOW4;


void onewire_Init(onewire_port_t OWx){
	if (OWx == onewire_OW1){
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
		OW_devices[OWx].port = GPIOC;
		OW_devices[OWx].pin = GPIO_Pin_6;

	}else if (OWx == onewire_OW2){
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
		OW_devices[OWx].port = GPIOB;
		OW_devices[OWx].pin = GPIO_Pin_2;
	}else if (OWx == onewire_OW3){
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
		OW_devices[OWx].port = GPIOC;
		OW_devices[OWx].pin = GPIO_Pin_7;
	}

	OW_devices[OWx].config.GPIO_Pin = OW_devices[OWx].pin;
	OW_devices[OWx].config.GPIO_Mode = GPIO_Mode_OUT;
	OW_devices[OWx].config.GPIO_OType = GPIO_OType_OD;
	OW_devices[OWx].config.GPIO_PuPd = GPIO_PuPd_NOPULL;
	OW_devices[OWx].config.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(OW_devices[OWx].port, &(OW_devices[OWx].config));

	OW_devices[OWx].port->BSRRL = OW_devices[OWx].pin;
}
void onewire_write(onewire_port_t OWx, onewire_level_t state){
	if (state == onewire_high){
		OW_devices[OWx].port->BSRRH = OW_devices[OWx].pin;
	}else{
		OW_devices[OWx].port->BSRRL = OW_devices[OWx].pin;
	}
}

uint8_t onewire_sendResetBasic(onewire_port_t OWx){
	uint8_t presence;

	//480us drive low
	onewire_write(OWx, onewire_high);
	delayus(480);

	//Release line
	onewire_write(OWx, onewire_low);
	delayus(60);

	//Do Read - 47 us to perform read and switch to output, 22.4us to just read
	OW_devices[OWx].config.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(OW_devices[OWx].port, &(OW_devices[OWx].config));
	//do read
	presence = GPIO_ReadInputDataBit(OW_devices[OWx].port, OW_devices[OWx].pin);

	//reset to output
	OW_devices[OWx].config.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(OW_devices[OWx].port, &(OW_devices[OWx].config));


	//wait 120us more
	delayus(180 - 61 + 240);  //extra 240 for total of 960 us

	//Done
	return !presence;  //return 1 if device is detected
}

__inline__ void onewire_WriteOneBasic(onewire_port_t OWx){
	startDelayus(52+10); //56, but with function overhead and the delay of setting up startDelayus, it can effect timing.
	onewire_write(OWx, onewire_high);

	waitSpecificCount(7);
	//delayus(6);
	onewire_write(OWx, onewire_low);
	waitStartedDelay();
}

__inline__ void onewire_WriteZeroBasic(onewire_port_t OWx){
	startDelayus(52); //56
	onewire_write(OWx, onewire_high);
	waitStartedDelay();
	onewire_write(OWx, onewire_low);
	delayus(10);

}

__inline__ uint8_t onewire_ReadBasic(onewire_port_t OWx){
	uint8_t bitRead;
	startDelayus(52+10); //56
	onewire_write(OWx, onewire_high);
	delayus(1);//normally this would be bad, but since it just returns for <2, we are good.
	onewire_write(OWx, onewire_low);

	OW_devices[OWx].config.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(OW_devices[OWx].port, &(OW_devices[OWx].config));

	waitSpecificCount(12);
	//do read
	bitRead = GPIO_ReadInputDataBit(OW_devices[OWx].port, OW_devices[OWx].pin);

	//reset to output
	OW_devices[OWx].config.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(OW_devices[OWx].port, &(OW_devices[OWx].config));

	waitStartedDelay();
	return bitRead;
}

__inline__ void  onewire_sendBit(onewire_port_t OWx, uint8_t bit){
	if (bit)
		onewire_WriteOneBasic(OWx);
	else
		onewire_WriteZeroBasic(OWx);
}

void onewire_sendByte(onewire_port_t OWx, uint8_t byte){
	uint8_t i = 0;
	while (i++ < 8){
		onewire_sendBit(OWx, byte & 0x01);
		byte = byte >> 1;
	}
}

uint8_t onewire_readByte(onewire_port_t OWx){
	uint8_t i = 0, byte = 0;
	while(i < 8){
		byte |= (uint8_t)((onewire_ReadBasic(OWx) << (i++)));
	}
	return byte;
}

void onewire_read_latest_ROM(uint8_t * rom){
	memcpy(rom, ROM_NO, sizeof(ROM_NO));
}

onewire_error_t onewire_read_temp(onewire_port_t OWx, uint8_t rom[8], uint16_t * temperature){
	uint8_t presence, i, crc;
	uint8_t buffer[9];
	uint16_t temp;
	presence = onewire_sendResetBasic(OWx);

	if (!presence)
		return ONEWIRE_NO_PRESENCE;
	onewire_sendByte(OWx, onewire_match_rom);

	for (i=0; i < 8; i++){
		onewire_sendByte(OWx, rom[i]);
	}

	onewire_sendByte(OWx, DS18B20_convert);
	//poll device to see when it is done

	delayms(750);
	presence = onewire_sendResetBasic(OWx);

	onewire_sendByte(OWx, onewire_match_rom);

	for (i=0; i < 8; i++){
		onewire_sendByte(OWx, rom[i]);
	}

	onewire_sendByte(OWx, DS18B20_read_scratchpad);
	crc8 = 0;
	for (i=0; i < 8; i++){
		buffer[i] = onewire_readByte(OWx);
		docrc8(buffer[i]);
	}
	crc = onewire_readByte(OWx);

	if (crc8 == crc){
		//good data
		temp = (uint16_t)((uint16_t)buffer[1] << 8) | buffer[0];
		printSerial(USART2, "Good data, temperature is %d %f C = %f F\r\n", temp, ((float)temp/16.0), (((float)temp/16.0)*9.0/5.0 + 32.0));
		*temperature = temp;
	}else{
		printSerial(USART2,"Bad data %d %d\r\n", crc, crc8);
		return ONEWIRE_NO_PRESENCE;
	}
	return ONEWIRE_NO_ERROR;
}

void onewire_trigger_temp(onewire_port_t OWx){
	uint8_t presence;

	presence = onewire_sendResetBasic(OWx);

	if (!presence)
		return;
	onewire_sendByte(OWx, onewire_skip_rom);

	onewire_sendByte(OWx, DS18B20_convert);
}

//can only be used when powered directly
onewire_error_t onewire_read_stored_temp(onewire_port_t OWx, uint8_t rom[8], uint16_t * temperature){
	uint8_t presence, i, crc, is_done = 0;
	uint8_t buffer[9], pbuf[60];
	uint16_t temp;
	uint16_t timeout = 0;

	//check that it's done before resetting
	timeout = 0;
	while(!is_done){
		is_done = onewire_ReadBasic(OWx);
		delayms(1);
		if (timeout++ > 10000){
			break;
		}
	}

	presence = onewire_sendResetBasic(OWx);
	if (!presence)
		return ONEWIRE_NO_PRESENCE;

	onewire_sendByte(OWx, onewire_match_rom);

	for (i=0; i < 8; i++){
		onewire_sendByte(OWx, rom[i]);
	}

	onewire_sendByte(OWx, DS18B20_read_scratchpad);
	crc8 = 0;
	for (i=0; i < 8; i++){
		buffer[i] = onewire_readByte(OWx);
		docrc8(buffer[i]);
	}
	crc = onewire_readByte(OWx);

	if (crc8 == crc){
		//good data
		temp = (uint16_t)((uint16_t)buffer[1] << 8) | buffer[0];
		*temperature = temp;
		printSerial(USART2,"Good trig data, temperature is %d %f C = %f F\r\n", temp, ((float)temp/16.0), (((float)temp/16.0)*9.0/5.0 + 32.0));
		return ONEWIRE_NO_ERROR;
	}else{
		printSerial(USART2,"Bad trig data %d %d\r\n", crc, crc8);
	}
	return ONEWIRE_NO_PRESENCE;
}

/*
 * The following code is from Maxim 1-Wire Search Algorithm, Application Note 187
 * http://www.maximintegrated.com/en/app-notes/index.mvp/id/187
 */

//--------------------------------------------------------------------------
// Find the 'first' devices on the 1-Wire bus
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : no device present
//
int OWFirst(onewire_port_t OWx)
{
	// reset the search state
	LastDiscrepancy = 0;
	LastDeviceFlag = OW_FALSE;
	LastFamilyDiscrepancy = 0;

	return OWSearch(OWx);
}

//--------------------------------------------------------------------------
// Find the 'next' devices on the 1-Wire bus
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
int OWNext(onewire_port_t OWx)
{
	// leave the search state alone
	return OWSearch(OWx);
}

//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
int OWSearch(onewire_port_t OWx)
{
	unsigned char id_bit_number;
	unsigned char last_zero, rom_byte_number, search_result;
	unsigned char id_bit, cmp_id_bit;
	unsigned char rom_byte_mask, search_direction;

	// initialize for search
	id_bit_number = 1;
	last_zero = 0;
	rom_byte_number = 0;
	rom_byte_mask = 1;
	search_result = 0;
	crc8 = 0;

	// if the last call was not the last one
	if (!LastDeviceFlag)
	{
		// 1-Wire reset
		if (!OWReset(OWx))
		{
			// reset the search
			LastDiscrepancy = 0;
			LastDeviceFlag = OW_FALSE;
			LastFamilyDiscrepancy = 0;
			return OW_FALSE;
		}

		// issue the search command
		OWWriteByte(OWx, 0xF0);

		// loop to do the search
		do
		{
			// read a bit and its complement
			id_bit = OWReadBit(OWx);
			cmp_id_bit = OWReadBit(OWx);

			// check for no devices on 1-wire
			if ((id_bit == 1) && (cmp_id_bit == 1))
				break;
			else
			{
				// all devices coupled have 0 or 1
				if (id_bit != cmp_id_bit)
					search_direction = id_bit;  // bit write value for search
				else
				{
					// if this discrepancy if before the Last Discrepancy
					// on a previous next then pick the same as last time
					if (id_bit_number < LastDiscrepancy)
						search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
					else
						// if equal to last pick 1, if not then pick 0
						search_direction = (id_bit_number == LastDiscrepancy);

					// if 0 was picked then record its position in LastZero
					if (search_direction == 0)
					{
						last_zero = id_bit_number;

						// check for Last discrepancy in family
						if (last_zero < 9)
							LastFamilyDiscrepancy = last_zero;
					}
				}

				// set or clear the bit in the ROM byte rom_byte_number
				// with mask rom_byte_mask
				if (search_direction == 1)
					ROM_NO[rom_byte_number] |= rom_byte_mask;
				else
					ROM_NO[rom_byte_number] &= (unsigned char)(~rom_byte_mask);

				// serial number search direction write bit
				OWWriteBit(OWx, search_direction);

				// increment the byte counter id_bit_number
				// and shift the mask rom_byte_mask
				id_bit_number++;
				rom_byte_mask = (unsigned char)(rom_byte_mask << 1);

				// if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
				if (rom_byte_mask == 0)
				{
					docrc8(ROM_NO[rom_byte_number]);  // accumulate the CRC
					rom_byte_number++;
					rom_byte_mask = 1;
				}
			}
		}
		while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

		// if the search was successful then
		if (!((id_bit_number < 65) || (crc8 != 0)))
		{
			// search successful so set LastDiscrepancy,LastDeviceFlag,search_result
			LastDiscrepancy = last_zero;

			// check for last device
			if (LastDiscrepancy == 0)
				LastDeviceFlag = OW_TRUE;

			search_result = OW_TRUE;
		}
	}

	// if no device found then reset counters so next 'search' will be like a first
	if (!search_result || !ROM_NO[0])
	{
		LastDiscrepancy = 0;
		LastDeviceFlag = OW_FALSE;
		LastFamilyDiscrepancy = 0;
		search_result = OW_FALSE;
	}

	return search_result;
}

//--------------------------------------------------------------------------
// Verify the device with the ROM number in ROM_NO buffer is present.
// Return TRUE  : device verified present
//        FALSE : device not present
//
int OWVerify(onewire_port_t OWx)
{
	unsigned char rom_backup[8];
	int i,rslt,ld_backup,ldf_backup,lfd_backup;

	// keep a backup copy of the current state
	for (i = 0; i < 8; i++)
		rom_backup[i] = ROM_NO[i];
	ld_backup = LastDiscrepancy;
	ldf_backup = LastDeviceFlag;
	lfd_backup = LastFamilyDiscrepancy;

	// set search to find the same device
	LastDiscrepancy = 64;
	LastDeviceFlag = OW_FALSE;

	if (OWSearch(OWx))
	{
		// check if same device found
		rslt = OW_TRUE;
		for (i = 0; i < 8; i++)
		{
			if (rom_backup[i] != ROM_NO[i])
			{
				rslt = OW_FALSE;
				break;
			}
		}
	}
	else
		rslt = OW_FALSE;

	// restore the search state
	for (i = 0; i < 8; i++)
		ROM_NO[i] = rom_backup[i];
	LastDiscrepancy = ld_backup;
	LastDeviceFlag = ldf_backup;
	LastFamilyDiscrepancy = lfd_backup;

	// return the result of the verify
	return rslt;
}

//--------------------------------------------------------------------------
// Setup the search to find the device type 'family_code' on the next call
// to OWNext() if it is present.
//
void OWTargetSetup(unsigned char family_code)
{
	int i;

	// set the search state to find SearchFamily type devices
	ROM_NO[0] = family_code;
	for (i = 1; i < 8; i++)
		ROM_NO[i] = 0;
	LastDiscrepancy = 64;
	LastFamilyDiscrepancy = 0;
	LastDeviceFlag = OW_FALSE;
}

//--------------------------------------------------------------------------
// Setup the search to skip the current device type on the next call
// to OWNext().
//
void OWFamilySkipSetup()
{
	// set the Last discrepancy to last family discrepancy
	LastDiscrepancy = LastFamilyDiscrepancy;
	LastFamilyDiscrepancy = 0;

	// check for end of list
	if (LastDiscrepancy == 0)
		LastDeviceFlag = OW_TRUE;
}

//--------------------------------------------------------------------------
// 1-Wire Functions to be implemented for a particular platform
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Reset the 1-Wire bus and return the presence of any device
// Return TRUE  : device present
//        FALSE : no device present
//
int OWReset(onewire_port_t OWx)
{
	// platform specific
	// TMEX API TEST BUILD
	//return (TMTouchReset(session_handle) == 1);
	return (onewire_sendResetBasic(OWx) == 1);
}

//--------------------------------------------------------------------------
// Send 8 bits of data to the 1-Wire bus
//
void OWWriteByte(onewire_port_t OWx, unsigned char byte_value)
{
	// platform specific

	// TMEX API TEST BUILD
	//TMTouchByte(session_handle,byte_value);
	onewire_sendByte(OWx, byte_value);
}

//--------------------------------------------------------------------------
// Send 1 bit of data to teh 1-Wire bus
//
void OWWriteBit(onewire_port_t OWx, unsigned char bit_value)
{
	// platform specific

	// TMEX API TEST BUILD
	//TMTouchBit(session_handle,(short)bit_value);
	onewire_sendBit(OWx, bit_value);
}

//--------------------------------------------------------------------------
// Read 1 bit of data from the 1-Wire bus
// Return 1 : bit read is 1
//        0 : bit read is 0
//
unsigned char OWReadBit(onewire_port_t OWx)
{
	// platform specific

	// TMEX API TEST BUILD

	//return (unsigned char)TMTouchBit(session_handle,0x01);
	return (unsigned char)onewire_ReadBasic(OWx);

}

static unsigned char dscrc_table[] = {
		0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
		157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
		35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
		190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
		70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
		219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
		101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
		248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
		140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
		17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
		175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
		50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
		202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
		87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
		233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
		116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};

//--------------------------------------------------------------------------
// Calculate the CRC8 of the byte value provided with the current
// global 'crc8' value.
// Returns current global crc8 value
//
unsigned char docrc8(unsigned char value)
{
	// See Application Note 27

	// TEST BUILD
	crc8 = dscrc_table[crc8 ^ value];
	return crc8;
}

/*
 * The preceding code is from Maxim 1-Wire Search Algorithm, Application Note 187
 * http://www.maximintegrated.com/en/app-notes/index.mvp/id/187
 */

