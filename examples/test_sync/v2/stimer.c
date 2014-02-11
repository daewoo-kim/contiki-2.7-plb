#include "stimer.h"
#include <stdio.h>

void stimer_set(struct stimer *st, struct stimeval *interval){
  clock_time_t tick = stimeval_to_tick(interval, st->clock);
  etimer_set(&(st->et), tick);
}
void stimer_set_blink(struct stimer *st, struct stimeval *interval){
  struct stimeval rest;
  stime_t now = sclock_get(st->clock);
  rest.sec = (interval->sec) - (now % (interval->sec));
  printf("now:%d, rest:%d\n", now, rest.sec);

  clock_time_t tick = stimeval_to_tick(&rest, st->clock);
  etimer_set(&(st->et), tick);
}

int stimer_expired(struct stimer *st){
  return etimer_expired(&(st->et));
}
