
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

/* Comandos SPI */
#define CMD_R_REGISTER       0x00
#define CMD_W_REGISTER       0x20
#define CMD_R_RX_PAYLOAD     0x61
#define CMD_W_TX_PAYLOAD     0xA0
#define CMD_FLUSH_TX         0xE1
#define CMD_FLUSH_RX         0xE2
#define CMD_NOP              0xFF
#define CMD_REUSE_TX_PL      0xE3
#define CMD_ACTIVATE         0x50
#define CMD_R_RX_PL_WID      0x60
#define CMD_W_ACK_PAYLOAD    0xA8
#define CMD_W_TX_PAYLOAD_NO_ACK 0xB0

/* Funções auxiliares internas */
static void nrf_csn_low(void) { CSN_LOW(); }
static void nrf_csn_high(void) { CSN_HIGH(); }

static void nrf_ce_pulse(void) {
  CE_HIGH();
  _delay_us(15);
  CE_LOW();
}

void nrf24_init(void)
{
  NRF_CE_DDR  |= (1 << NRF_CE_PIN);
  NRF_CSN_DDR |= (1 << NRF_CSN_PIN);
  CE_LOW();
  CSN_HIGH();

  spi_init();

  _delay_ms(100); // Tempo de boot do modulo

  // CONFIG: PWR_UP | PRIM_RX | CRC habilitado (2 bytes)
  uint8_t config = 0x0F; // 0b00001111
  nrf_write_register(NRF24_REG_CONFIG, &config, 1);

  _delay_ms(5);
  uint8_t confirm_config = nrf_read_register(NRF24_REG_CONFIG); // APAGAR
  printf("NRF CONFIG REGISTER = 0x%02X\n", confirm_config);

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

/* Função axuliar de escrita em registrador */
void nrf_write_register(uint8_t reg, const uint8_t *data, uint8_t len)
{
  printf("WRITE: reg=0x%02X len=%d\n", reg, len);
  for (uint8_t i = 0; i < len; i++)
    printf("  val[%d] = 0x%02X\n", i, data[i]);
  
  CSN_LOW(); // Ativa CSN

  spi_transfer(CMD_W_REGISTER | (reg & 0x1F)); // Comando de escrita
  for(uint8_t i = 0; i < len; i++) spi_transfer(data[i]);

  printf("Wrote %d bytes to register 0x%02X\n", len, reg);

  CSN_HIGH(); // Desativa CSN
}

/* Função de leitura de um registrador específico */
uint8_t nrf_read_register(uint8_t reg) {
    uint8_t value;

    CSN_LOW(); // Ativa CSN

    spi_transfer(CMD_R_REGISTER | (reg & 0x1F)); // Comando de leitura
    
    value = spi_transfer(CMD_NOP); // Lê o valor do registrador
    // printf("READ: reg=0x%02X value=0x%02X\n", reg, value);

    CSN_HIGH(); // Desativa CSN
    
    return value;
}

/* Função auxiliar de leitura de registrador */
void nrf_read_register_bytes(uint8_t reg, uint8_t *data, uint8_t len)
{
  CSN_LOW();
  
  spi_transfer(CMD_R_REGISTER | (reg & 0x1F)); // Comando de leitura
  for(uint8_t i = 0; i < len; i++) data[i] = spi_transfer(CMD_NOP); // Lê os dados
  
  CSN_HIGH();
}

/* Função de leitura do registrador STATUS */
uint8_t nrf_get_status(void)
{
  CSN_LOW();

  uint8_t status = spi_transfer(CMD_NOP); // Lê o status
  printf("NRF STATUS = 0x%02X\n", status);

  CSN_HIGH();
  return status;
}

/* --- PAYLOAD --- */
void nrf_write_payload(const uint8_t *data, uint8_t len) {
  CSN_LOW();

  spi_transfer(CMD_W_TX_PAYLOAD); // Comando W_TX_PAYLOAD
  for (uint8_t i = 0; i < len; i++) spi_transfer(data[i]);
  printf("Wrote %d bytes to payload\n", len);

  CSN_HIGH();
}

void nrf24_read_payload(uint8_t *data, uint8_t len) {
  CSN_LOW();

  spi_transfer(CMD_R_RX_PAYLOAD);
  for(uint8_t i = 0; i < len; i++) data[i] = spi_transfer(CMD_NOP);
  
  CSN_HIGH();
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
  uint8_t reg = (pipe == 0) ? NRF24_REG_RX_ADDR_P0 : NRF24_REG_RX_ADDR_P1;
  nrf_write_register(reg, addr, len);
}

void nrf24_set_tx_mode(void)
{
  CE_LOW(); // Desliga CE para evitar transmissão acidental

  // Coloca o rádio em modo TX
  uint8_t config = nrf_read_register(NRF24_REG_CONFIG);
  printf("tx_mode_reading: NRF CONFIG REGISTER = 0x%02X\n", config);
  config |= (1 << 1); // PWR_UP
  config &= ~(1 << 0); // PRIM_RX = 0
  printf("tx_mode_writing: NRF CONFIG REGISTER  = 0x%02X\n", config);
  nrf_write_register(NRF24_REG_CONFIG, &config, 1);
  
  // Espera o rádio estabilizar
  _delay_ms(2);
}

void nrf24_set_rx_mode(void)
{
  CE_LOW(); // Desliga CE para evitar transmissão acidental
  // Coloca o rádio em modo RX
  uint8_t config = nrf_read_register(NRF24_REG_CONFIG);
  printf("rx_mode_reading: NRF CONFIG REGISTER = 0x%02X\n", config);
  config |= (1 << 1); // PWR_UP
  config |= (1 << 0); // PRIM_RX = 1
  printf("rx_mode_writing: NRF CONFIG REGISTER  = 0x%02X\n", config);
  nrf_write_register(NRF24_REG_CONFIG, &config, 1);

  CE_HIGH(); // Ativa CE para iniciar recepção
}

bool nrf24_send(const void *data, uint8_t len)
{
  nrf24_set_tx_mode(); // Coloca o rádio em modo TX (CONSIDERAR CONTROLAR NO APP)
  nrf_write_payload(data, len); // Envia os dados
  nrf_ce_pulse(); // Pulso em CE para iniciar a transmissão

  // Aguarda resultado
  uint8_t status;
  uint8_t timeout = 100;
  do
  {
    status = nrf_read_register(NRF24_REG_STATUS);
    _delay_ms(1); // Espera 1ms entre leituras
  } while (!(status & (1 << 5)) && !(status & (1 << 4)) && --timeout); // TX_DS ou MAX_RT ou timeout
  
  // Limpa Flags
  uint8_t clear = 0x70; // Limpa TX_DS e MAX_RT
  nrf_write_register(NRF24_REG_STATUS, &clear, 1);
  printf("Cleared STATUS: 0x%02X\n", clear);

  return (status & (1 << 5)); // Retorna true se TX_DS (transmissão bem-sucedida)
}

int nrf24_receive(void *buf, uint8_t bufsize)
{
  uint8_t status = nrf_get_status();

  // Verifica se há dados disponíveis
  if (!(status & (1 << 6))) { // RX_DR (bit 6)
    return 0;
  }

  // Lê os dados - nrf_read_payload
  nrf24_read_payload((uint8_t *) buf, bufsize); // Lê o payload recebido

  // Limpa bit RX_DR
  uint8_t clear = 0x40;
  nrf_write_register(NRF24_REG_STATUS, &clear, 1);

  return bufsize;
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
