#include "sclock.c"
#include <sys/clock.h>

/* stimer */
struct stimer{
  struct etimer et;
  struct sclock *clock;
};
void stimer_set(struct stimer *st, struct stimeval *interval);
void stimer_set_blink(struct stimer *st, struct stimeval *interval);
int stimer_expired(struct stimer *st);
