#include "contiki.h"
#include "sys/autostart.h"
#include "sys/etimer.h"
#include "nrf24l01_atmega328p.h"

#include <stdio.h>

/*--------------------------------------------------------------------------*/
PROCESS(nrf_test_process, "NRF24 Test Process");
AUTOSTART_PROCESSES(&nrf_test_process);
/*--------------------------------------------------------------------------*/

PROCESS_THREAD(nrf_test_process, ev, data)
{
    static struct etimer timer;

    PROCESS_BEGIN();
    nrf24_init(); // Inicializa o NRF24
    printf("NRF24 Test Process started\n");

    while(1) {
        etimer_set(&timer, CLOCK_SECOND * 5);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));

        printf("NRF24 Test Process running\n");
        uint8_t raw_status = nrf_read_register(NRF24_REG_STATUS);
        printf("NRF RAW STATUS REGISTER = 0x%02X\n", raw_status);
    }

    PROCESS_END();
}