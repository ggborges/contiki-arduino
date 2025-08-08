#include "contiki.h"
#include <stdio.h>
#include "dev/spi.h"
#include "dev/nrf24l01_atmega328p_aux.h"

PROCESS(writing_config_reg_process, "Writing CONFIG Register Test");
AUTOSTART_PROCESSES(&writing_config_reg_process);

PROCESS_THREAD(writing_config_reg_process, ev, data)
{
  PROCESS_BEGIN();

  printf("\n[Contiki] Iniciando teste do registrador CONFIG\n");

  nrf24_init(); // Inicializa o NRF24L01+
  nrf24_config_prx(250);

  clock_wait(CLOCK_SECOND / 10); // Espera 100ms para estabilizar

  uint8_t valores[] = {0x00, 0x7F, 0x2A, 0x55, 0x0F, 0x0A};
  uint8_t lido;

  for (uint8_t i = 0; i < sizeof(valores); i++) {
    uint8_t val = valores[i];
    printf("Escrevendo 0x%02X no CONFIG...\n", val);
    nrf_write_register(NRF24_REG_CONFIG, val);
    clock_delay_usec(5000); // 5ms

    lido = nrf_read_register(NRF24_REG_CONFIG);
    printf("CONFIG lido: 0x%02X %s\n", lido, (lido == val) ? "[OK]" : "[FALHA]");
  }

  PROCESS_END();
}
