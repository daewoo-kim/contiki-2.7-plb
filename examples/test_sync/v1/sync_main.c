#include "contiki.h"
#include "dev/leds.h"
#include "sclock.c"
#include "net/rime.h"

#include <stdio.h>
#include <sys/clock.h>

#define HOST 0

void print_leds();
/*---------------------------------------------------------------------------*/
PROCESS(sync_main, "Sync main");
AUTOSTART_PROCESSES(&sync_main);
/*---------------------------------------------------------------------------*/
static struct sclock sclk;
static struct stimer st;

PROCESS_THREAD(sync_main, ev, data)
{
  /* static struct timer tm; */
  char* packet;
  int packet_len = 0;

  PROCESS_EXITHANDLER(abc_close(&abc);)
  PROCESS_BEGIN();

  sclock_init(&sclk);
  st.clock = &sclk;
  abc.ptr = (void*) &sclk;
  abc_open(&abc, 128, &abc_call);

  struct stimeval tv;
  while(1) {
    tv.sec = 15;
    stimer_set_blink(&st, &tv);
    PROCESS_WAIT_EVENT_UNTIL(stimer_expired(&st));
    
    leds_invert(LEDS_RED);

#if HOST
    packet = ntp_make_request(&sclk, &packet_len); 
    ntp_send_packet(packet, packet_len);
    free(packet);
#endif
  }
  
  /*  timer_set(&tm, CLOCK_SECOND);
  while(1) {
    if(timer_expired(&tm)){
      leds_invert(LEDS_RED);
      
      packet = ntp_make_request(&sclk, &packet_len);
      send_packet(packet, packet_len);
      free(packet);
      timer_set(&tm, CLOCK_SECOND);
    }
    }*/
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

void
print_leds(){
  unsigned char ledst = leds_get();
  int green, yellow, red;

  green = ledst&LEDS_GREEN;
  yellow = (ledst&LEDS_YELLOW)>>1;
  red = (ledst&LEDS_RED)>>2;
  
  printf("ledfd:%u%u%u\n", yellow, green, red);
}

