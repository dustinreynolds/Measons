/*
 * spi.h
 *
 *  Created on: May 9, 2015
 *      Author: Dustin
 */

#ifndef SPI_H_
#define SPI_H_


#define SPI_MAX_NUM_DEVICES 3
#define SPI_FLASH				0
#define SPI_RFM69_1				1
#define SPI_RFM69_2				2

#define SPI2_BUFFER_SIZE 260

typedef struct{
	SPI_TypeDef* SPIx;
	GPIO_TypeDef * cs_port;
	uint16_t cs_pin;
} SPI_Device_t;

void spi_SPI2_Configuration(void);
uint8_t spi_send(uint8_t data);
uint8_t __inline__ spi_send_buffer(uint32_t size);
uint8_t __inline__ spi_send_dummy_buffer(uint32_t size);
uint8_t spi_write_register_cs(uint8_t num, uint8_t reg_cmd, uint8_t reg_op);
uint8_t spi_read_register_cs(uint8_t num, uint8_t reg_cmd);

#endif /* SPI_H_ */
