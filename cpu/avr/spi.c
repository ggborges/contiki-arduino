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
#include <stdio.h>
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
  DDRB &= ~BV(MISO); // MISO como entrada
  PORTB |= BV(MOSI) | BV(SCK);

  /* Enables SPI, selects "master", clock rate FCK / 2, and SPI mode 0
  SPCR = BV(SPE) | BV(MSTR);
  SPSR = BV(SPI2X);
  */

  /* Enables SPI, selects "master", clock rate FCK / 4, and SPI mode 0 */
  SPCR = BV(SPE) | BV(MSTR);       // SPI Enable + Master mode (SPR1=0, SPR0=0)
  SPSR &= ~BV(SPI2X);              // SPI2X = 0 → não dobra a velocidade
}

/*-------------------------------------------------------------------------------*/
// Comandos de controle do SPI
// R_REGISTER = 0x00 - Lê um registrador [000A AAAA - AAAAA é o endereço do registrador / 000 é o comando]
// W_REGISTER = 0x20 - Escreve em um registrador [001A AAAA - AAAAA é o endereço do registrador / 001 é o comando]
// R_RX_PAYLOAD = 0x61 - Lê o payload recebido. 1-32 bytes, usado após receber um pacote em modo RX  [0110 0001 - 0x61]
// W_TX_PAYLOAD = 0xA0 - Escreve o payload a ser transmitido. 1-32 bytes, usado para enviar dados em modo TX [1010 0000 - 0xA0]
// FLUSH_TX = 0xE1 - Limpa o FIFO de transmissão, usado para descartar dados pendentes no buffer de transmissão [1110 0001 - 0xE1]
// FLUSH_RX = 0xE2 - Limpa o FIFO de recepção, usado para descartar dados pendentes no buffer de recepção [1110 0010 - 0xE2]
// REUSE_TX_PL = 0xE3 - Reutiliza o payload de transmissão
// ACTIVATE = 0x50 - Ativa o comando de ativação (usado para comandos especiais)
// R_RX_PL_WID = 0x60 - Lê o tamanho do payload recebido
// W_ACK_PAYLOAD = 0xA8 - Escreve o payload de ACK
// W_TX_PAYLOAD_NO_ACK = 0xB0 - Envia payload sem esperar
// NOP = 0xFF - No Operation (usado para ler o status do
/*-------------------------------------------------------------------------------*/

uint8_t spi_transfer(uint8_t data) {
  SPDR = data;
  while (!(SPSR & BV(SPIF)));
  return SPDR;
}

/*

FUNÇÕES ANTIGAS

void spi_write(uint8_t data) {
  spi_enable(); // Ativa CSN (Chip Select Not)
  SPDR = data;
  while(!(SPSR & BV(SPIF)));
  uint8_t resp = SPDR;
  printf("SPI Write: 0x%02X, Read: 0x%02X\n", data, resp);
  spi_disable(); // Desativa CSN
}

uint8_t spi_read(void) {
  //spi_enable(); // Ativa CSN (Chip Select Not)
  SPDR = 0xFF; // Envia dummy byte
  while(!(SPSR & BV(SPIF)));
  uint8_t val = SPDR;
  printf("SPI Read: 0x%02X\n", val);
  return val;
  //spi_disable(); // Desativa CSN
}

// Essas duas assumem que CSN está no pino definido como SS (por padrão PB2 = CSN)
void spi_enable(void) {
  PORTB &= ~BV(CSN); // CSN LOW
}

void spi_disable(void) {
  PORTB |= BV(CSN);  // CSN HIGH
}

*/