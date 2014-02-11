#include "contiki.h"
#include "random.h"
#include "dev/leds.h"

#include <stdio.h>

void print_leds();

/*---------------------------------------------------------------------------*/
PROCESS(blink, "Blink test");
AUTOSTART_PROCESSES(&blink);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(blink, ev, data)
{
  static struct etimer et;
  PROCESS_BEGIN();

  while(1) {
    /* Delay 2-4 seconds */
    etimer_set(&et, CLOCK_SECOND/2);

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    leds_invert(LEDS_RED);
    
    print_leds();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

void print_leds(){
  unsigned char ledst = leds_get();
  int green, yellow, red;

  green = ledst&LEDS_GREEN;
  yellow = (ledst&LEDS_YELLOW)>>1;
  red = (ledst&LEDS_RED)>>2;
  
  printf("ledfd:%u%u%u\n", yellow, green, red);
}
