#include "ntp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void
ntp_init(struct ntp *ntp, struct sclock *clock)
{
  ntp->sc = clock;
  ntp->t1 = 0;
  ntp->t2 = 0;
  ntp->t3 = 0;
  ntp->t4 = 0;
}

void
ntp_handle(char* handle_str, struct ntp *ntp)
{
  /* parsing */
  char* tok = strtok(handle_str, "\n:");
  if(strcmp(tok, "Sync_Request") == 0){
    ntp_handle_request(ntp);
  }
  else if(strcmp(tok, "Sync_Reply") == 0){
    tok = strtok(NULL, "\n:");
    if(strcmp(tok, "t2") == 0){
      tok = strtok(NULL, "\n:");
      ntp->t2 = atoi(tok);

      tok = strtok(NULL, "\n:");
      if(strcmp(tok, "t3") == 0){
	tok = strtok(NULL, "\n:");
	ntp->t3 = atoi(tok);

	ntp->t4 = sclock_get(ntp->sc);
	ntp_handle_reply(ntp);
      }     
    }
    

  }
}

void ntp_handle_request(struct ntp *ntp)
{
  ntp->t2 = sclock_get(ntp->sc);
  ntp->t3 = sclock_get(ntp->sc);

  char* packet;
  int packet_len = 0;
  packet = ntp_make_reply(ntp, &packet_len);
  ntp_send_packet(packet, packet_len);
  free(packet);
}
void ntp_handle_reply(struct ntp *ntp)
{
  stime_t t1 = ntp->t1;
  stime_t t2 = ntp->t2;
  stime_t t3 = ntp->t3;
  stime_t t4 = ntp->t4;
  struct sclock *sc = ntp->sc;

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
  print_ntp(ntp);
  printf("now:%d\n",sc->now.sec);
  printf("slope:%d\n",sc->slope);
}

char*
ntp_make_request(struct ntp *ntp, int* len_p)
{
  ntp->t1 = sclock_get(ntp->sc);

  int len = REQUEST_LEN;
  char *clk_char = (char*) malloc(len);
  memset(clk_char, 0, len);
  sprintf(clk_char, "Sync_Request\n");
  (*len_p) = len;

  return clk_char;
}
char*
ntp_make_reply(struct ntp *ntp, int *len_p)
{
  int len = REPLY_LEN;
  char* clk_char = (char*) malloc(len);
  memset(clk_char, 0, len);
  sprintf(clk_char, "Sync_Reply\nt2:%d\nt3:%d\n", ntp->t2, ntp->t3);
  (*len_p) = len;

  return clk_char;
}

void
print_ntp(struct ntp* n){
  printf("t1:t2:t3:t4 %d:%d:%d:%d\n",n->t1,n->t2,n->t3,n->t4);
}

void ntp_send_packet(char* packet, int len){ 
  packetbuf_copyfrom(packet, len);
  abc_send(&abc);
}
