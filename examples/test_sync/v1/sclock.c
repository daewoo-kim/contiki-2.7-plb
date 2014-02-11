#include "contiki.h"
#include "sclock.h"
#include "net/rime.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void
sclock_init(struct sclock *sc)
{
  sc->now.sec = 0;
  sc->slope = SLOPE_INIT;
  sc->last_tick = clock_time();
}

clock_time_t
stimeval_to_tick(struct stimeval *stval, struct sclock *sc)
{
  clock_time_t tick = (stval->sec)*(sc->slope);
  return tick;
}

stime_t
sclock_get(struct sclock *sc)
{
  clock_time_t now_tick = clock_time();
  clock_time_t calc = (now_tick-(sc->last_tick))/(sc->slope);
  clock_time_t remain = (now_tick-(sc->last_tick))%(sc->slope);
  /* printf("now_tick:%d\nlast_tick:%d\nslope:%d\n",now_tick, sc->last_tick, sc->slope); */
  /* printf("calc:%d, remain:%d\n", calc, remain); */
  sc->last_tick = now_tick-remain;
  /* printf("clast_tick:%d\n", sc->last_tick); */
  sc->now.sec += calc;
  return sc->now.sec;
}

void
ntp_handle(char* handle_str, struct sclock *sc)
{
  /* parsing */
  char* tok = strtok(handle_str, "\n:");
  if(strcmp(tok, "Sync_Request") == 0){
    ntp_handle_request(sc);
  }
  else if(strcmp(tok, "Sync_Reply") == 0){
    tok = strtok(NULL, "\n:");
    if(strcmp(tok, "t2") == 0){
      tok = strtok(NULL, "\n:");
      sc->ntp.t2 = atoi(tok);

      tok = strtok(NULL, "\n:");
      if(strcmp(tok, "t3") == 0){
	tok = strtok(NULL, "\n:");
	sc->ntp.t3 = atoi(tok);

	sc->ntp.t4 = sclock_get(sc);
	ntp_handle_reply(sc);
      }     
    }
    

  }
}

void ntp_handle_request(struct sclock *sc)
{
  sc->ntp.t2 = sclock_get(sc);
  sc->ntp.t3 = sclock_get(sc);

  char* packet;
  int packet_len = 0;
  packet = ntp_make_reply(sc, &packet_len);
  ntp_send_packet(packet, packet_len);
  free(packet);
}
void ntp_handle_reply(struct sclock *sc)
{
  stime_t t1 = sc->ntp.t1;
  stime_t t2 = sc->ntp.t2;
  stime_t t3 = sc->ntp.t3;
  stime_t t4 = sc->ntp.t4;

  if( t1 > t4)
    return;
  if( t2 > t3)
    return;

  int offset = ((t2-t1)+(t3-t4))/2;
  int rtt = (t2-t1)-(t3-t4);
  printf("off:%d\nrtt:%d\n",offset,rtt);
  sc->now.sec = sc->now.sec+offset;
  if( offset > 0){
    (sc->slope)--;
  }
  else if( offset < 0){
    (sc->slope)++;
  }
  print_ntp(&(sc->ntp));
  printf("now:%d\n",sc->now.sec);
  printf("slope:%d\n",sc->slope);
}

char*
ntp_make_request(struct sclock *sc, int* len_p)
{
  sc->ntp.t1 = sclock_get(sc);

  int len = REQUEST_LEN;
  char *clk_char = (char*) malloc(len);
  memset(clk_char, 0, len);
  sprintf(clk_char, "Sync_Request\n");
  (*len_p) = len;

  return clk_char;
}
char*
ntp_make_reply(struct sclock *sc, int *len_p)
{
  int len = REPLY_LEN;
  char* clk_char = (char*) malloc(len);
  memset(clk_char, 0, len);
  sprintf(clk_char, "Sync_Reply\nt2:%d\nt3:%d\n", sc->ntp.t2, sc->ntp.t3);
  (*len_p) = len;

  return clk_char;
}

void
print_ntp(struct ntp* n){
  printf("t1:t2:t3:t4 %d:%d:%d:%d\n",n->t1,n->t2,n->t3,n->t4);
}

/*---------------------------------------------------------------------------*/
static void ntp_recv_packet(struct abc_conn *c)
{
  struct sclock* sc = (struct sclock*) c->ptr;
  char* recv_str = (char *)packetbuf_dataptr();
  printf("%s\n",recv_str);
  ntp_handle(recv_str, sc);
}
static struct abc_conn abc;
static const struct abc_callbacks abc_call = {ntp_recv_packet};
/*---------------------------------------------------------------------------*/
void ntp_send_packet(char* packet, int len){ 
  packetbuf_copyfrom(packet, len);
  abc_send(&abc);
}


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
