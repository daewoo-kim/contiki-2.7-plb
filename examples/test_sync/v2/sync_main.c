#include "contiki.h"
#include "dev/leds.h"
//#include "ntp.c"
//#include "stimer.c"
#include "sclock.c"
#include "net/rime.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/clock.h>

#define REQUEST_ON 1

void print_leds();
/*---------------------------------------------------------------------------*/
PROCESS(sync_main, "Sync main");
AUTOSTART_PROCESSES(&sync_main);
/*---------------------------------------------------------------------------*/
static struct sclock sclk;
//static struct ntp ntp;
//static struct stimer st;

PROCESS_THREAD(sync_main, ev, data)
{
  /* static struct timer tm; */
  static struct etimer et;
  char* packet;
  int packet_len = 0;

  //  PROCESS_EXITHANDLER(abc_close(&abc);)
  PROCESS_BEGIN();

  sclock_init(&sclk);
  //st.clock = &sclk;
  //ntp_init(&ntp, &sclk);

  //abc.ptr = (void*) &ntp;
  //abc_open(&abc, 128, &abc_call);

  struct stimeval *stv;

  while(1) {
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    
    leds_invert(LEDS_RED);

    sclock_update(&sclk);
    stv = &(sclk.now);
    printf("--result--\n");
    printf("min: %d\nsec: %d\nmsec: %d\n", stv->min, stv->sec, stv->msec);
  }

  /*  while(1) {
    tv.sec = 15;
    stimer_set_blink(&st, &tv);
    PROCESS_WAIT_EVENT_UNTIL(stimer_expired(&st));
    
    leds_invert(LEDS_RED);

#if REQUEST_ON
    packet = ntp_make_request(&ntp, &packet_len); 
    ntp_send_packet(packet, packet_len);
    free(packet);
#endif
}*/
  
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

