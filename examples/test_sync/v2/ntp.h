#include "sclock.c"
#include "net/rime.h"

/* Protocol */
/* Request/Reply
 * header : value
 * 
 * ex)
 * Sync_Request\n
 *
 * ex)
 * Sync_Reply\n
 * t2:1233\n
 * t3:1256\n
 */
#define REQUEST_LEN 50
#define REPLY_LEN 100

struct ntp{
  struct sclock *sc;
  stime_t t1, t2, t3, t4;
};

/* ntp functions */
void ntp_init(struct ntp *ntp, struct sclock *clock);
char* ntp_make_request(struct ntp *ntp, int *len_p);
char* ntp_make_reply(struct ntp *ntp, int* len_p);
void ntp_handle(char* handle_str, struct ntp *ntp);
void ntp_handle_request(struct ntp *ntp);
void ntp_handle_reply(struct ntp *ntp);
void ntp_send_packet(char* packet, int len);
void print_ntp(struct ntp* n);

/*---------------------------------------------------------------------------*/
static void ntp_recv_packet(struct abc_conn *c)
{
  struct ntp *ntp = (struct ntp*) c->ptr;
  char* recv_str = (char *)packetbuf_dataptr();
  ntp_handle(recv_str, ntp);
}
static struct abc_conn abc;
static const struct abc_callbacks abc_call = {ntp_recv_packet};
/*---------------------------------------------------------------------------*/
