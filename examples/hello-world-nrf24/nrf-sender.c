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

  PROCESS_BEGIN();

  nrf24_init();
  nrf24_set_tx_address((const uint8_t *)"NODEB", 5); // Define o endereço de transmissão
  nrf24_set_rx_address(0, (const uint8_t *)"NODEA", 5); // Define o endereço de recepção para ACK
  nrf24_set_tx_mode(); // Coloca o rádio em modo TX

  printf("Enviando mensagens...\n");

  while (1) {
    etimer_set(&timer, CLOCK_SECOND * 5);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));

    if (nrf24_send(message, sizeof(message))) {
      printf("Mensagem enviada: %s\n", message);
    } else {
      printf("Falha ao enviar mensagem.\n");
    }
  }

  PROCESS_END();
}