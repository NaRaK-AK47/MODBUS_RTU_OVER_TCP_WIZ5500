/*
 * W5500_Spi.h
 *
 *  Created on: Apr 26, 2024
 *      Author: KARAN
 */

#ifndef W5500_W5500_SPI_H_
#define W5500_W5500_SPI_H_


#include "stm32f1xx_hal.h"

void W5500Init();

void  wizchip_select(void);
void  wizchip_deselect(void);

uint8_t wizchip_read();
void  wizchip_write(uint8_t wb);

void wizchip_readburst(uint8_t* pBuf, uint16_t len);
void  wizchip_writeburst(uint8_t* pBuf, uint16_t len);



#endif /* W5500_W5500_SPI_H_ */
