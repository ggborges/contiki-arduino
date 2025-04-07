/*
 * Copyright (c) 2007, Swedish Institute of Computer Science
 * All rights reserved. 
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met: 
 * 1. Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 * 3. Neither the name of the Institute nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND 
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
 * SUCH DAMAGE. 
 *
 */

#include <avr/io.h>

#include "contiki-conf.h"

#define MOSI PB3
#define MISO PB4
#define SCK  PB5
#define CSN  PB2
/*
 * On the Tmote sky access to I2C/SPI/UART0 must always be
 * exclusive. Set spi_busy so that interrupt handlers can check if
 * they are allowed to use the bus or not. Only the CC2420 radio needs
 * this in practice.
 * 
 */
unsigned char spi_busy = 0;

/*
 * Initialize SPI bus.
 */
void
spi_init(void)
{
  /* Initalize ports for communication with SPI units. */
  /* CSN=SS and must be output when master! */
  DDRB  |= BV(MOSI) | BV(SCK) | BV(CSN);
  PORTB |= BV(MOSI) | BV(SCK);

  /* Enables SPI, selects "master", clock rate FCK / 2, and SPI mode 0 */
  SPCR = BV(SPE) | BV(MSTR);
  SPSR = BV(SPI2X);
}

void spi_write(uint8_t data) {
  SPDR = data;
  while(!(SPSR & BV(SPIF)));
}

uint8_t spi_read(void) {
  SPDR = 0xFF; // Envia dummy byte
  while(!(SPSR & BV(SPIF)));
  return SPDR;
}

// Essas duas assumem que CSN está no pino definido como SS (por padrão PB2 = CSN)
void spi_enable(void) {
  PORTB &= ~BV(CSN); // CSN LOW
}

void spi_disable(void) {
  PORTB |= BV(CSN);  // CSN HIGH
}
