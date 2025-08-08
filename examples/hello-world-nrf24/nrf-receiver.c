#include "contiki.h"
#include "sys/etimer.h"
#include <stdio.h>
#include "nrf24l01_atmega328p.h"

PROCESS(nrf_receiver_process, "NRF24 Receiver");
AUTOSTART_PROCESSES(&nrf_receiver_process);

PROCESS_THREAD(nrf_receiver_process, ev, data)
{
  static struct etimer timer;
  static uint8_t buffer[8];
  static int len;
  uint8_t addr[5] = "NODEA"; // Endereço de recepção

  PROCESS_BEGIN();

  nrf24_init(); // já configura PRIM_RX e CE_HIGH()
  
  uint8_t test_status = nrf_get_status();
  printf("STATUS inicial após init: 0x%02X\n", test_status);
  
  nrf24_set_rx_address(0, addr, 5); // Define o endereço de recepção
  nrf24_set_rx_mode(); // Coloca o rádio em modo RX

  printf("Esperando mensagens...\n");

  while (1) {
    etimer_set(&timer, CLOCK_SECOND * 5);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));

    uint8_t status = nrf_get_status();
    printf("Status do NRF antes de receber: 0x%02X\n", status);

    int len = nrf24_receive(buffer, sizeof(buffer));
    if (len > 0) {
      printf("Recebido: ");
      for (int i = 0; i < len; i++) {
        printf("%c", buffer[i]);
      }
      printf("\n");
    } else {
      printf("Nenhuma mensagem recebida.\n");
    }
  }

  /*while (1) {
    len = nrf24_receive(buf, sizeof(buf) - 1);
    if (len > 0) {
      buf[len] = '\0';
      printf("Recebido: %s\n", buf);
    }

    PROCESS_PAUSE();
  }*/

  PROCESS_END();
}
