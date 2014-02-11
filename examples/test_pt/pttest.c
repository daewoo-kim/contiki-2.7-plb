#include "contiki.h"
#include <stdio.h>
#include "sys/pt.h"
#include "sys/rtimer.h"
#include <signal.h>
#include <unistd.h>
#include "sys/etimer.h"

static struct pt pt;
static struct rtimer rt;
static int done;
char call1(struct rtimer *t, void *ptr){
  PT_BEGIN(&pt);

  printf("new call1\n");
  static int i=0;
  while(1){
    i++;
    printf("call1 i=%d\n",i);
    rtimer_set(t, RTIMER_NOW()+RTIMER_ARCH_SECOND,1, 
	       (void (*)(struct rtimer *, void *))call1,ptr);
    PT_YIELD(&pt);
    printf("back to call1\n");
  }

  PT_END(&pt);
}

char call2(struct rtimer *t, void *ptr){
  PT_BEGIN(&pt);

  printf("new call2\n");
  static int i=0;
  while(i<2){
    i++;
    printf("call2 i=%d\n",i);
    rtimer_set(t, RTIMER_NOW()+RTIMER_ARCH_SECOND,1, 
	       (void (*)(struct rtimer *, void *))call2,ptr);
    PT_YIELD(&pt);
    printf("back to call2\n");
  }
  PT_END(&pt);
}

PT_THREAD(call3(struct rtimer *t, void *ptr)){
  PT_BEGIN(&pt);

  printf("new call3\n");
  static int i=0;
  while(1){
    i++;
    printf("call3 i=%d\n",i);
    rtimer_set(t, RTIMER_NOW()+RTIMER_ARCH_SECOND,1, 
	       (void (*)(struct rtimer *, void *))call3,ptr);
    PT_YIELD(&pt);
    printf("back to call3\n");
  }
  done = 1;
  PT_END(&pt);
}

void call4()
{
  rtimer_clock_t rct;
  int i=0;
  while(i<3){
    i++;
    printf("========call4========\n");
    rct = RTIMER_NOW();
    while(RTIMER_CLOCK_LT(RTIMER_NOW(), rct+RTIMER_ARCH_SECOND*3)){
    }
  }
}

static char sig_handler(int signo)
{
  printf("signal!\n");
  call4();
  return 0;
}

static void wrap()
{
  call4();
  printf("aaaaa\n");
  call3(&rt, NULL);
}

/*---------------------------------------------------------------------------*/
PROCESS(pttest, "Protothread test");
AUTOSTART_PROCESSES(&pttest);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(pttest, ev, data)
{
  PROCESS_BEGIN();

  signal(SIGTSTP, (void *)sig_handler);
  
  rtimer_init();
  PT_INIT(&pt);
  done = 0;
  call3(&rt,NULL);
  printf("process end\n");

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
