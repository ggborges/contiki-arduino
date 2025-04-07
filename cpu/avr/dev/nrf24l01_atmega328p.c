#include "nrf24l01_atmega328p.h"
#include "contiki.h"
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include "spi.h"

/* Macros de controle */
#define CE_HIGH()  (NRF_CE_PORT |= (1 << NRF_CE_PIN))
#define CE_LOW()   (NRF_CE_PORT &= ~(1 << NRF_CE_PIN))
#define CSN_HIGH() (NRF_CSN_PORT |= (1 << NRF_CSN_PIN))
#define CSN_LOW()  (NRF_CSN_PORT &= ~(1 << NRF_CSN_PIN))

void nrf24_init(void)
{
  NRF_CE_DDR  |= (1 << NRF_CE_PIN);
  NRF_CSN_DDR |= (1 << NRF_CSN_PIN);
  CE_LOW();
  CSN_HIGH();

  spi_init();

  _delay_ms(100); // Tempo de boot

  // CONFIG: PWR_UP | PRIM_RX | CRC habilitado
  uint8_t config = 0b00001111; // 0x0F
  nrf_write_register(0x00, &config, 1);

  // EN_RXADDR: Habilita pipe 0
  uint8_t en_rxaddr = 0x01;
  nrf_write_register(0x02, &en_rxaddr, 1);

  // RF_CH: Canal 2
  uint8_t channel = 2;
  nrf_write_register(0x05, &channel, 1);

  // RX_PW_P0: Tamanho do payload fixo
  uint8_t payload_size = 8;
  nrf_write_register(0x11, &payload_size, 1);

  // STATUS: Limpa bits de IRQ (escrevendo 1)
  uint8_t clear_status = 0x70;
  nrf_write_register(0x07, &clear_status, 1);

  // Ativa recepção
  CE_HIGH();
  _delay_ms(2);
}

bool nrf24_send(const void *data, uint8_t len)
{
  /* Placeholder - envio não implementado */
  (void)data;
  (void)len;
  return false;
}

int nrf24_receive(void *buf, uint8_t bufsize)
{
  /* Placeholder - recepção não implementada */
  (void)buf;
  (void)bufsize;
  return 0;
}

uint8_t nrf24_status(void)
{
  /* Placeholder para ler o status do rádio */
  return 0;
}

/* Função axuliar de escrita em registrador */
void nrf_write_register(uint8_t reg, const uint8_t *data, uint8_t len)
{
  CSN_LOW();
  spi_write(0x20 | reg); // Comando de escrita
  for(uint8_t i = 0; i < len; i++) {
    spi_write(data[i]);
  }
  CSN_HIGH();
}

/* Função auxiliar de leitura de registrador */
void nrf_read_register_bytes(uint8_t reg, uint8_t *data, uint8_t len)
{
  CSN_LOW();
  spi_write(reg); // Comando de leitura
  for(uint8_t i = 0; i < len; i++) {
    data[i] = spi_read();
  }
  CSN_HIGH();
}

/* Função de leitura do registrador STATUS */
uint8_t nrf_get_status(void)
{
  CSN_LOW();
  uint8_t status = spi_read(); // NOP retorna STATUS
  CSN_HIGH();
  return status;
}

/* Função de leitura de um registrador específico */
uint8_t nrf_read_register(uint8_t reg) {
    uint8_t value;

    spi_enable();
    spi_write(reg & 0x1F);
    value = spi_read();
    spi_disable();

    return value;
}