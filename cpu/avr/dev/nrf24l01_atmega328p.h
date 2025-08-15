/*
 * Copyright (c) 2025, Gustavo Gonçalves Borges, Federal University of Pernambuco.
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
 * This file is part of the Contiki operating system.
 *
 * @(#)$$
 */

/**
 * \file
 *         AVR specific definitions for the rs232 port.
 *
 * \author
 *         Simon Barner <barner@in.tum.de
 */

#ifndef NRF24L01_ATMEGA328P_H_
#define NRF24L01_ATMEGA328P_H_

#include <stdint.h>
#include <stdbool.h>

// Pinos SPI e controle
// MISO - D12 (PB4)
// MOSI - D11 (PB3)
// SCK  - D13 (PB5)
// CSN  - D10 (PB2)
// CE   - D9  (PB1)
// VCC - 3.3V
// GND - GND

/* Define os pinos do SPI e CE, CSN */
#define NRF_CE_PORT  PORTB
#define NRF_CE_DDR   DDRB
#define NRF_CE_PIN   PB1

#define NRF_CSN_PORT PORTB
#define NRF_CSN_DDR  DDRB
#define NRF_CSN_PIN  PB2

/* Registradores nRF24L01 */
#define NRF24_REG_CONFIG      0x00
#define NRF24_REG_EN_AA       0x01
#define NRF24_REG_EN_RXADDR   0x02
#define NRF24_REG_SETUP_AW    0x03
#define NRF24_REG_SETUP_RETR  0x04
#define NRF24_REG_RF_CH       0x05
#define NRF24_REG_RF_SETUP    0x06
#define NRF24_REG_STATUS      0x07
#define NRF24_REG_OBSERVE_TX  0x08
#define NRF24_REG_CD          0x09
#define NRF24_REG_RX_ADDR_P0  0x0A
#define NRF24_REG_RX_ADDR_P1  0x0B
#define NRF24_REG_TX_ADDR     0x10
#define NRF24_REG_RX_PW_P0    0x11
#define NRF24_REG_RX_PW_P1    0x12
#define NRF24_REG_FIFO_STATUS 0x17

void nrf24_init(void);
void nrf24_config(uint16_t kbps, uint8_t channel, uint8_t payload_size);
void nrf24_reset(void);
void nrf24_power_down(void);
void nrf24_power_up(void);
void nrf24_set_tx_address(const uint8_t *addr, uint8_t len);
void nrf24_set_rx_address(uint8_t pipe, const uint8_t *addr, uint8_t len);
void nrf24_set_tx_mode(void);
void nrf24_set_rx_mode(void);
bool nrf24_send(const void *data, uint8_t len);
bool nrf24_send_std(const void *data, uint8_t len);
int nrf24_receive(void *buf, uint8_t bufsize);
uint8_t nrf_get_status(void);
bool nrf_data_available(void);
uint8_t nrf_read_register(uint8_t reg);
void nrf_read_register_bytes(uint8_t reg, uint8_t *data, uint8_t len);
void nrf_write_register(uint8_t reg, const uint8_t *data, uint8_t len);

// Auxiliares
void nrf_write_payload(const uint8_t *data, uint8_t len);
void nrf_read_payload(uint8_t *data, uint8_t len);

#endif /* NRF24L01_ATMEGA328P_H_ */