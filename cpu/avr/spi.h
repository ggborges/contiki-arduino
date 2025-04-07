#ifndef SPI_H
#define SPI_H

#include <stdint.h>

void spi_init(void);
uint8_t spi_read(void);
void spi_write(uint8_t data);
void spi_enable(void);   // Precisamos definir o controle do CSN
void spi_disable(void);  // idem

#endif