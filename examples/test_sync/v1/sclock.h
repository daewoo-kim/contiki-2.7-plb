#include "contiki.h"
#include <sys/clock.h>

/* #define SLOPE_INIT CLOCK_SECOND */
#define SLOPE_INIT 10

typedef int stime_t;

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
  stime_t t1, t2, t3, t4;
};

/* clock */
struct stimeval{
  stime_t sec;
};

struct sclock{
  struct stimeval now;
  clock_time_t slope; /* a second */
  clock_time_t last_tick;
  struct ntp ntp;
};

stime_t sclock_get(struct sclock *sc);
clock_time_t stimeval_to_tick(struct stimeval *stval, struct sclock *sc);
void sclock_init(struct sclock *sc);
void sclock_sync(struct sclock *sc);

/* ntp functions */
char* ntp_make_request(struct sclock *sc, int *len_p);
char* ntp_make_reply(struct sclock *sc, int* len_p);
void ntp_handle(char* handle_str, struct sclock *sc);
void ntp_handle_request(struct sclock *sc);
void ntp_handle_reply(struct sclock *sc);
void ntp_send_packet(char* packet, int len);
void print_ntp(struct ntp* n);

/* stimer */
struct stimer{
  struct etimer et;
  struct sclock *clock;
};
void stimer_set(struct stimer *st, struct stimeval *interval);
int stimer_expired(struct stimer *st);
