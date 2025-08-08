#include "contiki.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <util/delay.h>
#include "sys/autostart.h"
#include "sys/etimer.h"
#include "dev/spi.h"
#include "dev/nrf24l01_atmega328p_aux.h"

// Define o processo
PROCESS(ptx_process, "NRF24 PTX Process");
AUTOSTART_PROCESSES(&ptx_process);

// Implementação do processo
PROCESS_THREAD(ptx_process, ev, data)
{
  static struct etimer timer;
  static uint8_t counter = 0;
  static uint8_t payload[8];

  PROCESS_BEGIN();

  nrf24_init();
  _delay_ms(100); // Espera o módulo estabilizar
  nrf24_config_ptx(250);
  
  printf("PTX iniciado. Enviando pacotes...\n");

  etimer_set(&timer, CLOCK_SECOND); // 1 segundo

  while (1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));

    // Monta payload de 8 bytes
    for (uint8_t i = 0; i < 8; i++) {
      payload[i] = counter + i;
    }

    bool success = nrf24_send(payload, 8);
    if (success) {
      printf("Pacote enviado com sucesso: %u\n", counter);
    } else {
      printf("Falha no envio (MAX_RT): %u\n", counter);
    }

    counter++;
    etimer_reset(&timer);
  }

  PROCESS_END();
}
