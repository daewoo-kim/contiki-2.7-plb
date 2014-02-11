#pragma once

#include "contiki.h"
#include "sclock.h"

#include <stdio.h>

void
sclock_init(struct sclock *sc)
{
  sc->now.min = 0;
  sc->now.sec = 0;
  sc->now.msec = 0;

  sc->slope = SLOPE_INIT;
  sc->last_tick = clock_time();
}

void
sclock_update(struct sclock *sc)
{
  clock_time_t now_tick = clock_time();
  stime_t calc = (now_tick-(sc->last_tick))/(sc->slope);
  clock_time_t remain = (now_tick-(sc->last_tick))%(sc->slope);

  sc->last_tick = now_tick-remain;

  struct stimeval st_calc;
  stimeval_copy_stime(&st_calc, calc);
  stimeval_cat(&(sc->now), &st_calc);
}

clock_time_t
stimeval_to_tick(struct stimeval *stval, struct sclock *sc)
{
  clock_time_t tick = (stval->sec)*(sc->slope);
  return tick;
}

void
stimeval_copy_stime(struct stimeval *dst, stime_t src){
  dst->min = src / (SEC_LIMIT*MSEC_LIMIT);
  src = src % (SEC_LIMIT*MSEC_LIMIT);

  dst->sec = src / MSEC_LIMIT;
  src = src % MSEC_LIMIT;

  dst->msec = src;
}

void
stimeval_cat(struct stimeval* dst, struct stimeval *src)
{
  dst->msec += src->msec;
  dst->sec += src->sec;
  dst->min += src->min;

  if(dst->msec > MSEC_LIMIT){
    dst->sec += (dst->msec / MSEC_LIMIT);
    dst->msec = dst->msec % MSEC_LIMIT;
  }
  if(dst->sec > SEC_LIMIT){
    dst->min += (dst->sec / SEC_LIMIT);
    dst->sec = dst->sec % SEC_LIMIT;
  }
}
