#include "contiki.h"
#include <stdio.h>
#include <string.h>
#include "nrf24l01_atmega328p.h"
#include "sys/etimer.h"

PROCESS(nrf_sender_process, "NRF24 Sender");
AUTOSTART_PROCESSES(&nrf_sender_process);

PROCESS_THREAD(nrf_sender_process, ev, data)
{
  static struct etimer timer;
  static uint8_t message[8] = "HELLO";
  uint8_t addr[5] = "NODEA";

  PROCESS_BEGIN();

  nrf24_init();

  uint8_t config = nrf_read_register(NRF24_REG_CONFIG);
  printf("CONFIG: 0x%02X\n", config);

  uint8_t test_status = nrf_get_status();
  printf("STATUS inicial após init: 0x%02X\n", test_status);

  nrf24_set_tx_address(addr, 5); // Define o endereço de transmissão
  nrf24_set_rx_address(0, addr, 5); // Define o endereço de recepção para ACK
  nrf24_set_tx_mode(); // Coloca o rádio em modo TX

  printf("Enviando mensagens...\n");

  while (1) {
    etimer_set(&timer, CLOCK_SECOND * 5);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));

    uint8_t status = nrf_get_status();
    printf("Status do NRF antes do envio: 0x%02X\n", status);

    if (nrf24_send(message, sizeof(message))) {
      printf("Mensagem enviada: %s\n", message);
    } else {
      printf("Falha ao enviar mensagem.\n");
    }

    status = nrf_get_status();
    printf("Status do NRF após o envio: 0x%02X\n", status);
  }

  PROCESS_END();
}