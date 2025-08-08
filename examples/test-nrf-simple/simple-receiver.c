#include "contiki.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <util/delay.h>
#include "sys/autostart.h"
#include "sys/etimer.h"
#include "dev/spi.h"
#include "dev/nrf24l01_atmega328p_aux.h"

// Variável de sinalização de interrupção
volatile bool pacote_disponivel = false;

// ISR de INT0 → ativada por IRQ do nRF24
ISR(INT0_vect) {
  pacote_disponivel = true;
}

// Define o processo
PROCESS(prx_process, "NRF24 PRX Process");
AUTOSTART_PROCESSES(&prx_process);

PROCESS_THREAD(prx_process, ev, data)
{
  //static struct etimer timer;
  static uint8_t payload[8];

  PROCESS_BEGIN();

  nrf24_init(); // Inicializa o módulo nRF24L01
  nrf24_config_prx(250);
  nrf24_setup_irq(); // Configura interrupção para RX

  printf("PRX iniciado. Aguardando pacotes...\n");

  //etimer_set(&timer, CLOCK_SECOND / 10); // Polling a cada 100ms

  while (1) {
    //PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));

    PROCESS_YIELD(); // Espera por eventos, incluindo interrupções
    if (pacote_disponivel) {
      pacote_disponivel = false; // Reseta a flag

      // Lê o payload recebido
      if (nrf24_receive(payload, 8)) {
        printf("Pacote recebido: ");
        for (uint8_t i = 0; i < 8; i++) {
          printf("%u ", payload[i]);
        }
        printf("\n");
      }
    }
    
    //etimer_reset(&timer); // Reseta o timer para o próximo ciclo
  }

  PROCESS_END();
}
