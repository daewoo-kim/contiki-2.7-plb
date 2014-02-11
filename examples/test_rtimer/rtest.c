#include "contiki.h"

#include <stdio.h>
#include "sys/pt.h"
#include "sys/rtimer.h"

static struct pt pt;

/*static void call(struct rtimer *t, void *ptr){
  int *p = (int*)ptr;
  (*p)++;
  printf("%d\n",(*p));
  printf("rtimer time:%d\n",RTIMER_TIME(t));
  printf("rtimer now:%lu\n",RTIMER_NOW());
  }*/

static void call3(struct rtimer *t, void *ptr){
  printf("hello\n");
}

char call2(struct rtimer *t, void *ptr){
  PT_BEGIN(&pt);

  printf("new\n");
  static int i=0;
  while(1){
    i++;
    printf("i=%d\n",i);
    printf("1\n");
    rtimer_set(t, RTIMER_NOW()+RTIMER_ARCH_SECOND,1, 
	       (void (*)(struct rtimer *, void *))call2,ptr);
    printf("2\n");
    PT_YIELD(&pt);
    printf("3\n");
  }

  PT_END(&pt);
}
/*---------------------------------------------------------------------------*/
PROCESS(rtest, "Rtimer test");
AUTOSTART_PROCESSES(&rtest);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(rtest, ev, data)
{
  PROCESS_BEGIN();
  rtimer_init();
  PT_INIT(&pt);
  static struct rtimer rt;

  int val = 0;
  
  call2(&rt, NULL);


  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
