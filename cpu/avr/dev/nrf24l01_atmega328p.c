
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

  _delay_ms(100); // Tempo de boot do modulo

  // CONFIG: PWR_UP | PRIM_RX | CRC habilitado (2 bytes)
  uint8_t config = 0b00001111; // 0x0F
  nrf_write_register(NRF24_REG_CONFIG, &config, 1);

  // EN_RXADDR: Habilita pipe 0
  uint8_t en_rxaddr = 0x01;
  nrf_write_register(NRF24_REG_EN_RXADDR, &en_rxaddr, 1);

  // RF_CH: Canal 2
  uint8_t channel = 2;
  nrf_write_register(NRF24_REG_RF_CH, &channel, 1);

  // RX_PW_P0: Tamanho do payload fixo de 8 bytes no pipe 0
  uint8_t payload_size = 8;
  nrf_write_register(NRF24_REG_RX_PW_P0, &payload_size, 1);

  // STATUS: Limpa bits de IRQ (escrevendo 1)
  uint8_t clear_status = 0x70;
  nrf_write_register(NRF24_REG_STATUS, &clear_status, 1);

  // Ativa recepção
  CE_HIGH();
  _delay_ms(2);
}

void nrf24_reset(void)
{
  // Reseta o rádio para o estado inicial
  nrf24_init();
  
  // Configura o modo RX
  nrf24_set_rx_mode();
}

void nrf24_power_down(void)
{
  CE_LOW(); // Desliga CE para evitar transmissão acidental

  // Coloca o rádio em modo power down
  uint8_t config = nrf_read_register(NRF24_REG_CONFIG);
  config &= ~(1 << 1); // PWR_UP = 0
  nrf_write_register(NRF24_REG_CONFIG, &config, 1);
  
  // Espera o rádio estabilizar
  _delay_ms(2);
}

void nrf24_power_up(void)
{
  CE_LOW(); // Desliga CE para evitar transmissão acidental

  // Coloca o rádio em modo power up
  uint8_t config = nrf_read_register(NRF24_REG_CONFIG);
  config |= (1 << 1); // PWR_UP = 1
  nrf_write_register(NRF24_REG_CONFIG, &config, 1);
  
  // Espera o rádio estabilizar
  _delay_ms(2);
}

void nrf24_set_tx_address(const uint8_t *addr, uint8_t len)
{
  if (len > 5) len = 5; // Máximo de 5 bytes

  // Define o endereço de transmissão
  nrf_write_register(NRF24_REG_TX_ADDR, addr, len);

  // Define o endereço de recepção para ACK
  nrf_write_register(NRF24_REG_RX_ADDR_P0, addr, len);
}

void nrf24_set_rx_address(uint8_t pipe, const uint8_t *addr, uint8_t len)
{
  if (pipe > 1 || len > 5) return; // Apenas pipes 0 e 1, máximo de 5 bytes

  // Define o endereço de recepção para o pipe especificado
  if (pipe == 0) {
    nrf_write_register(NRF24_REG_RX_ADDR_P0, addr, len);
  } else if (pipe == 1) {
    nrf_write_register(NRF24_REG_RX_ADDR_P1, addr, len);
  }
}

void nrf24_set_tx_mode(void)
{
  CE_LOW(); // Desliga CE para evitar transmissão acidental

  // Coloca o rádio em modo TX
  uint8_t config = nrf_read_register(NRF24_REG_CONFIG);
  config |= (1 << 1); // PWR_UP
  config &= ~(1 << 0); // PRIM_RX = 0
  nrf_write_register(NRF24_REG_CONFIG, &config, 1);
  
  // Espera o rádio estabilizar
  _delay_ms(2);
}

void nrf24_set_rx_mode(void)
{
  // Coloca o rádio em modo RX
  uint8_t config = nrf_read_register(NRF24_REG_CONFIG);
  config |= (1 << 0); // PRIM_RX = 1
  config |= (1 << 1); // PWR_UP
  nrf_write_register(NRF24_REG_CONFIG, &config, 1);

  CE_HIGH(); // Ativa CE para iniciar recepção
}

bool nrf24_send(const void *data, uint8_t len)
{
  // Carrega no FIFO
  CSN_LOW();
  spi_write(0xA0); // Comando W_TX_PAYLOAD
  for (uint8_t i = 0; i < len; i++) {
    spi_write(((const uint8_t*)data)[i]);
  }
  CSN_HIGH();

  // Pulso em CE para iniciar a transmissão
  CE_HIGH();
  _delay_us(15); // >=10us
  CE_LOW();

  // Aguarda resultado
  uint8_t status;
  uint8_t timeout = 100;
  do
  {
    status = nrf_read_register(NRF24_REG_STATUS);
    _delay_ms(1); // Espera 1ms entre leituras
  } while (!(status & (1 << 5)) && !(status & (1 << 4)) && --timeout);
  
  // Limpa Flags
  uint8_t clear = 0x70; // Limpa TX_DS e MAX_RT
  nrf_write_register(NRF24_REG_STATUS, &clear, 1);

  return (status & (1 << 5)); // Retorna true se TX_DS (transmissão bem-sucedida)

}

bool nrf24_send_std(const void *data, uint8_t len)
{
  // Coloca o rádio em modo TX
  uint8_t config = 0b00001010; // PWR_UP + PRIM_RX=0
  nrf_write_register(NRF24_REG_CONFIG, &config, 1);
  _delay_ms(2);

  // Define endereço de destino
  uint8_t addr[5] = "1Node";
  nrf_write_register(0x10, addr, 5); // TX_ADDR
  nrf_write_register(0x0A, addr, 5); // RX_ADDR_P0 (ACK)

  // Envia os dados
  nrf_write_payload(data, len);

  // Pulso em CE para iniciar a transmissão
  CE_HIGH();
  _delay_us(15); // >=10us
  CE_LOW();

  // Aguarda transmissão (poderia verificar STATUS TX_DS, aqui simplificamos)
  _delay_ms(5);

  // Limpa STATUS
  uint8_t clear = 0x70;
  nrf_write_register(NRF24_REG_STATUS, &clear, 1);

  return true;
}

void nrf_write_payload(const uint8_t *data, uint8_t len) {
  CSN_LOW();
  spi_write(0xA0); // Comando W_TX_PAYLOAD
  for (uint8_t i = 0; i < len; i++) {
    spi_write(data[i]);
  }
  CSN_HIGH();
}

int nrf24_receive(void *buf, uint8_t bufsize)
{
  uint8_t status = nrf_get_status();

  // Verifica se há dados disponíveis
  if (!(status & (1 << 6))) { // RX_DR (bit 6)
    return 0;
  }

  // Lê os dados - nrf_read_payload
  CSN_LOW();
  spi_write(0x61); // R_RX_PAYLOAD
  for (uint8_t i = 0; i < bufsize; i++) {
    ((uint8_t*)buf)[i] = spi_read();
  }
  CSN_HIGH();

  // Limpa bit RX_DR
  uint8_t clear = 0x40;
  nrf_write_register(NRF24_REG_STATUS, &clear, 1);

  return bufsize;
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