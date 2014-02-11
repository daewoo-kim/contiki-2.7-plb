#include "contiki.h"
#include "timestamp.h"
#include "string.h"

/*---------------------------------------------------------------------------*/
static void print_packet(uint8_t *packet, int len)
{
  int i,j;
  uint8_t num, num2;
  uint8_t jisu;

  for(i=0; i<len; i++){
    num = packet[i];
    for(j=7; j>-1; j--){
      jisu = 1 << j;
      num2 = num&jisu;
      num2 = num2 >> j;
      if(num&jisu){
	printf("1");
      }
      else{
	printf("0");
      }
    } 
    printf(" ");
  }
  printf("\n");
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS(ts_test, "Timestamp test");
AUTOSTART_PROCESSES(&ts_test);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ts_test, ev, data)
{
  PROCESS_BEGIN();

  struct timestamp ts;
  struct timestamp ts2;
  timestamp_init(&ts);
  ts.year = 12;
  ts.month = 12;
  ts.day = 31;
  ts.hour = 23;
  ts.min = 59;
  ts.sec = 59;
  ts.msec = 999;
  
  uint8_t packet[10];
  memset(packet,0,10);
  
  timestamp_to_packet(&ts,packet,0);
  timestamp_from_packet(&ts2,packet,0);
  print_timestamp(&ts2);

  /*  struct timestamp ts;
  struct timestamp ts2;
  timestamp_init(&ts);


  while(ts.year <= 1){
    timestamp_init(&ts2);
    ts2.min = 40;
    timestamp_plus(&ts,&ts2);
    print_timestamp(&ts);
  }
   
  int i=0;
  while(i<10000){
    i++;
    timestamp_init(&ts2);
    ts2.msec = 40;
    
    if( timestamp_minus(&ts,&ts2) < 0)
      break;
    print_timestamp(&ts);
    }*/



  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
