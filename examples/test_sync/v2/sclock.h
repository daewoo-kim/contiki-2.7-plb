#pragma once

#include "contiki.h"
#include <sys/clock.h>

/* #define SLOPE_INIT CLOCK_SECOND */
#define SLOPE_INIT 10

typedef int stime_t;

struct stimeval{
#define MIN_LIMIT 60
#define SEC_LIMIT 60
#define MSEC_LIMIT 100
  stime_t min;
  stime_t sec;
  stime_t msec;
};

struct sclock{
  struct stimeval now;
  clock_time_t slope; /* ticks per unit time */
  clock_time_t last_tick;
};

void sclock_update(struct sclock *sc);
void sclock_init(struct sclock *sc);

clock_time_t stimeval_to_tick(struct stimeval *stval, struct sclock *sc);
void stimeval_copy_stime(struct stimeval *dst, stime_t src);
void stimeval_cat(struct stimeval* src, struct stimeval *dst);
