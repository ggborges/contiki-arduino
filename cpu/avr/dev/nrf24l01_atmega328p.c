#include "nrf24l01_atmega328p_aux.h"
#include "contiki.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "spi.h"

/* Macros de controle */
#define CE_HIGH()  (NRF_CE_PORT |= (1 << NRF_CE_PIN))
#define CE_LOW()   (NRF_CE_PORT &= ~(1 << NRF_CE_PIN))
#define CSN_HIGH() (NRF_CSN_PORT |= (1 << NRF_CSN_PIN))
#define CSN_LOW()  (NRF_CSN_PORT &= ~(1 << NRF_CSN_PIN))

/* Comandos SPI */
#define CMD_R_REGISTER       0x00 // 000A AAAA
#define CMD_W_REGISTER       0x20 // 001A AAAA
#define CMD_R_RX_PAYLOAD     0x61 // 0110 0001
#define CMD_W_TX_PAYLOAD     0xA0 // 1010 0000
#define CMD_FLUSH_TX         0xE1 // 1110 0001
#define CMD_FLUSH_RX         0xE2 // 1110 0010
#define CMD_NOP              0xFF // 1111 1111
#define CMD_REUSE_TX_PL      0xE3 // 1110 0011
#define CMD_ACTIVATE         0x50 // 0101 0000
#define CMD_R_RX_PL_WID      0x60 // 0110 0000
#define CMD_W_ACK_PAYLOAD    0xA8 // 1010 1000
#define CMD_W_TX_PAYLOAD_NO_ACK 0xB0 // 1011 0000

// Bits do registrador STATUS
#define RX_DR   6
#define TX_DS   5
#define MAX_RT  4

#define RXMODE 0x01 // Modo RX
#define TXMODE 0x00 // Modo TX
#define NRF24_CHANNEL 2 // Canal de operação
#define NRF24_PAYLOAD_SIZE 8 // Tamanho do payload

#define NRF24L01_MAX_PAYLOAD_SIZE 32 // Tamanho máximo do payload

volatile uint8_t PMODE;


/* Funções auxiliares internas */
static void nrf_csn_low(void) { CSN_LOW(); }
static void nrf_csn_high(void) { CSN_HIGH(); }
static void nrf_ce_pulse(void) {
  CE_HIGH();
  _delay_us(15);
  CE_LOW();
}
static void nrf_flush_tx(void) {
  CSN_LOW(); // Ativa CSN
  spi_transfer(CMD_FLUSH_TX); // Comando para limpar FIFO TX
  CSN_HIGH(); // Desativa CSN
}
static void nrf_flush_rx(void) {
  CSN_LOW(); // Ativa CSN
  spi_transfer(CMD_FLUSH_RX); // Comando para limpar FIFO RX
  CSN_HIGH(); // Desativa CSN
}
// Criar macros para limpar interrupções
static void nrf_clear_interrupts(void) {
  nrf_write_register(NRF24_REG_STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));
}

void nrf_write_register(uint8_t reg, const uint8_t value) {
  CSN_LOW(); // Ativa CSN
  spi_transfer(CMD_W_REGISTER | reg); // Comando W_REGISTER
  spi_transfer(value); // Envia o valor do registrador
  CSN_HIGH(); // Desativa CSN
}

void nrf_write_register_bytes(uint8_t reg, const uint8_t *data, uint8_t len) {
  CSN_LOW(); // Ativa CSN
  spi_transfer(CMD_W_REGISTER | reg); // Comando W_REGISTER
  for (uint8_t i = 0; i < len; i++) {
    spi_transfer(data[i]);
  }
  CSN_HIGH(); // Desativa CSN
}

uint8_t nrf_read_register(uint8_t reg) {
    uint8_t value;
    // "Every new command must be started by a high to low transition CSN."  (NRF24L01+ Datasheet, pg 50)
    CSN_LOW(); // Ativa CSN
    _delay_us(10); // Espera um pouco para garantir que CSN esteja baixa
    
    spi_transfer(CMD_R_REGISTER | reg); // Comando de leitura
    value = spi_transfer(CMD_NOP); // Lê o valor do registrador
    printf("READ: reg=0x%02X value=0x%02X\n", reg, value);

    CSN_HIGH(); // Desativa CSN
    return value;
}

void nrf_write_payload(const uint8_t *data, uint8_t len) {
  if (len > NRF24_PAYLOAD_SIZE) len = NRF24_PAYLOAD_SIZE; // Limita o tamanho do payload

  CSN_LOW(); // Ativa CSN
  _delay_us(10); // Espera um pouco para garantir que CSN esteja baixa

  spi_transfer(CMD_W_TX_PAYLOAD); // Comando W_TX_PAYLOAD
  for (uint8_t i = 0; i < len; i++) {
    spi_transfer(data[i]);
  }
  printf("Wrote %d bytes to payload\n", len);
  CSN_HIGH(); // Desativa CSN
}

uint8_t nrf_read_payload(uint8_t *data, uint8_t len) {
  if (len > NRF24_PAYLOAD_SIZE) len = NRF24_PAYLOAD_SIZE; // Limita o tamanho do payload

  CSN_LOW(); // Ativa CSN
  _delay_us(10); // Espera um pouco para garantir que CSN esteja baixa

  spi_transfer(CMD_R_RX_PAYLOAD); // Comando R_RX_PAYLOAD
  for (uint8_t i = 0; i < len; i++) {
    data[i] = spi_transfer(CMD_NOP);
  }
  printf("Read %d bytes from payload\n", len);
  CSN_HIGH(); // Desativa CSN
  nrf_clear_interrupts(); // Limpa interrupções
  return len;
}

/* Inicializa o NRF24L01+ */
void nrf24_init(void)
{
  NRF_CE_DDR  |= (1 << NRF_CE_PIN);
  NRF_CSN_DDR |= (1 << NRF_CSN_PIN);
  CE_LOW();
  CSN_HIGH();
  spi_init();

  _delay_ms(100); // Tempo de boot do modulo

  // Limpa IRQs e status
  uint8_t clear_status = 0x70; // Limpa TX_DS, RX
  nrf_write_register(NRF24_REG_STATUS, clear_status);
  // Limpar FIFO
  nrf_flush_tx(); // Limpa FIFO TX
}

// === Configuração PRX ===
void nrf24_config_prx(uint16_t kbps)
{
  // Desliga CE para evitar transmissão acidental
  CE_LOW();

  // RX_ADDR_P0: Define o endereço de recepção para o pipe 0
  uint8_t rx_addr[5] = { 'R', 'x', 'A', 'A', 'A' };
  nrf_write_register_bytes(NRF24_REG_RX_ADDR_P0, rx_addr, sizeof(rx_addr));
  // Configura o canal
  nrf_write_register(NRF24_REG_RF_CH, NRF24_CHANNEL);
  // Configura o payload size
  nrf_write_register(NRF24_REG_RX_PW_P0, NRF24_PAYLOAD_SIZE);
  // EN_RXADDR: habilita pipe 0
  uint8_t en_rxaddr = 0x01; // Habilita apenas pipe 0
  nrf_write_register(NRF24_REG_EN_RXADDR, en_rxaddr);
  // EN_AA: habilita ACK automático para pipe 0
  uint8_t en_aa = 0x01; // Habilita ACK automático
  nrf_write_register(NRF24_REG_EN_AA, en_aa);
  // Setup retransmissão: 1500us delay, 15 retransmissões
  uint8_t setup_retr = (5 << 4) | 15;
  nrf_write_register(NRF24_REG_SETUP_RETR, setup_retr);
  // RF_SETUP: configura potência e taxa de dados
  //uint8_t rf_setup = 0x26
  uint8_t rf_setup = (1 << 3) | (kbps == 250 ? (1 << 5) : (kbps == 1000 ? (2 << 5) : (3 << 5))); // Potência máxima e taxa de dados
  nrf_write_register(NRF24_REG_RF_SETUP, rf_setup);
  // CONFIG: PWR_UP=1, PRIM_RX=1 (modo RX), CRC habilitado (2 bytes)
  uint8_t config = 0x0F; // 0b00001111
  //config |= (1 << 1); // PWR_UP
  //config |= (1 << 0); // PRIM_RX = 1
  nrf_write_register(NRF24_REG_CONFIG, config);

  // Espera o rádio estabilizar
  _delay_ms(2);
  uint8_t verify = nrf_read_register(NRF24_REG_CONFIG);
  printf("rx_mode_verification: NRF CONFIG REGISTER = 0x%02X\n", verify);
  _delay_ms(2);

  PMODE = RXMODE; // Modo RX

  CE_HIGH(); // Ativa CE para iniciar recepção
}

// == Configuração PTX ==
void nrf24_config_ptx(uint16_t kbps)
{
  CE_LOW(); // Desliga CE para evitar transmissão acidental

  uint8_t tx_addr[5] = { 'R', 'x', 'A', 'A', 'A' };
  // Define o endereço de transmissão
  nrf_write_register_bytes(NRF24_REG_TX_ADDR, tx_addr, sizeof(tx_addr));
  nrf_write_register_bytes(NRF24_REG_RX_ADDR_P0, tx_addr, sizeof(tx_addr)); // Endereço do pipe 1 (ACK)
  
  // Configura o canal
  nrf_write_register(NRF24_REG_RF_CH, NRF24_CHANNEL);
  // Configura o payload size
  nrf_write_register(NRF24_REG_RX_PW_P0, NRF24_PAYLOAD_SIZE); // Tamanho do payload para pipe 0
  // Habilita ACK automático para pipe 0
  uint8_t en_aa = 0x01; // ERX_P0 = (1 << 0)
  nrf_write_register(NRF24_REG_EN_AA, en_aa);
  // Configura o pipe 0 para receber ACK
  uint8_t en_rxaddr = 0x01; // Habilita apenas pipe 0
  nrf_write_register(NRF24_REG_EN_RXADDR, en_rxaddr);
  // Setup retransmissão: 1500us delay, 15 retransmissões
  uint8_t setup_retr = (5 << 4) | 15;
  nrf_write_register(NRF24_REG_SETUP_RETR, setup_retr);
  // RF_SETUP: configura potência e taxa de dados
  // RF_SETUP: 0x26
  uint8_t rf_setup = (3 << 1) | (kbps == 250 ? (1 << 5) : (kbps == 2000 ? (1 << 3) : (0 << 3))); // Potência máxima e taxa de dados
  nrf_write_register(NRF24_REG_RF_SETUP, rf_setup);

  // CONFIG: PWR_UP=1, PRIM_RX=0 (modo TX), CRC habilitado (2 bytes)
  uint8_t config = 0x0E; // 0b00001110
  //config |= (1 << 1); // PWR_UP
  //config &= ~(1 << 0); // PRIM_RX = 0
  nrf_write_register(NRF24_REG_CONFIG, config);
  // Espera o rádio estabilizar
  _delay_ms(2);
  uint8_t verify = nrf_read_register(NRF24_REG_CONFIG);
  printf("tx_mode_verification: NRF CONFIG REGISTER = 0x%02X\n", verify);
  _delay_ms(2);

  PMODE = TXMODE; // Modo TX
}

void nrf24_reset(void)
{
  // Limpa IRQs e status
  uint8_t clear_status = 0x70; // Limpa TX_DS, RX
  nrf_write_register(NRF24_REG_STATUS, clear_status);

  // Limpa FIFO TX e RX
  uint8_t flush = CMD_FLUSH_TX;
  CSN_LOW(); // Ativa CSN
  spi_transfer(flush); // Comando para limpar FIFO TX
  CSN_HIGH(); // Desativa CSN

  flush = CMD_FLUSH_RX;
  CSN_LOW(); // Ativa CSN
  spi_transfer(flush); // Comando para limpar FIFO RX
  CSN_HIGH(); // Desativa CSN
}

// === Envio de dados (PTX) ===
bool nrf24_send(const uint8_t *data, uint8_t len)
{
  // clear_interrupts();
  nrf_write_payload(data, len); // Envia os dados
  nrf_ce_pulse(); // Pulso em CE para iniciar a transmissão

  while (1) {
    uint8_t status = nrf_read_register(NRF24_REG_STATUS);
    if (status & (1 << 5)) { // TX_DS (1 << TX_DS)
      // Transmissão bem-sucedida
      // clear_interrupts(); // Limpa TX_DS
      uint8_t clear_status = 0x70; // Limpa TX_DS e MAX_RT
      nrf_write_register(NRF24_REG_STATUS, clear_status);
      printf("Data sent successfully.\n");
      return true;
    } else if (status & (1 << 4)) { // MAX_RT (1 << MAX_RT)
      // Máximo de retransmissões atingido
      uint8_t clear_status = 0x70; // Limpa TX_DS e MAX_RT
      nrf_write_register(NRF24_REG_STATUS, clear_status); // Limpa MAX_RT
      //nrf_write_register(CMD_FLUSH_TX, NULL, 0); // Limpa FIFO TX - precisa de uma função para isso
      printf("Transmission failed: MAX_RT reached.\n");
      return false;
    }
    _delay_ms(2); // Espera 2ms entre leituras
  }
}

// === Recebimento de dados (PRX) ===
bool nrf24_receive(uint8_t *data, uint8_t len)
{
  uint8_t status = nrf_read_register(NRF24_REG_STATUS);
  if (status & (1 << 6)) { // RX_DR (1 << RX_DR)
    nrf_read_payload(data, len); // Lê o payload recebido
    // clear_interrupts(); // Limpa RX_DR
    printf("Data received: %s\n", data);
    // Limpa RX_DR
    uint8_t clear_status = (1 << 6); // Limpa RX_DR
    nrf_write_register(NRF24_REG_STATUS, clear_status);
    return true; // Dados recebidos com sucesso
  }
  return false; // Nenhum dado disponível
}

// === Configura interrupção externa INT0 para IRQ ===
void nrf24_setup_irq(void) {
  // PD2 (INT0) como entrada (já é por padrão)
  DDRD &= ~(1 << PD2);

  // Habilita pull-up (opcional, depende do módulo)
  PORTD |= (1 << PD2);

  // Configura INT0 para borda de descida (falling edge)
  EICRA |= (1 << ISC01);
  EICRA &= ~(1 << ISC00);

  // Habilita a interrupção INT0
  EIMSK |= (1 << INT0);

  // Ativa interrupções globais
  sei();
}

// === Tratador da interrupção gerada pelo pino IRQ ===
void nrf24_irq_handler_rx(void) {
  printf("IRQ: nRF24L01+ RX interrupt triggered.\n");
  uint8_t status = nrf_read_register(NRF24_REG_STATUS);

  if (status & (1 << RX_DR)) {
    uint8_t payload[NRF24_PAYLOAD_SIZE];
    nrf_read_payload(payload, NRF24_PAYLOAD_SIZE);

    printf("IRQ: Data received via IRQ: %s\n", payload);

    // RX_DR é limpo por nrf_read_payload(), pois ele chama nrf_clear_interrupts()
    // Para mais controle, limpar aqui com:
    // nrf_write_register(NRF24_REG_STATUS, (1 << RX_DR));
  }

  // Para lidar com TX_DS ou MAX_RT em outros modos, tratar aqui também
}

// === Define o vetor da interrupção INT0 ===
/*ISR(INT0_vect) {
  nrf24_irq_handler();
}*/
