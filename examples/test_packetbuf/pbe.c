#include "contiki.h"

#include <stdio.h>
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include <string.h>
void print_packetbuf();
void packetbuf_copyfrom_hdr(const void* from, uint16_t len);
/*---------------------------------------------------------------------------*/
PROCESS(pbe, "Pbe test");
AUTOSTART_PROCESSES(&pbe);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(pbe, ev, data)
{
  PROCESS_BEGIN();
  struct queuebuf *packet;

  char aaa[10] = "abcde";
  char bbb[10] = "12345";
  char ccc[10] = "qwerty";

  /* test1: copyfrom */
  printf("-test1-\n");
  packetbuf_clear();
  packetbuf_copyfrom(aaa, 4);
  packetbuf_copyfrom_hdr(ccc, 5);
  print_packetbuf();


  /* test2: reference */
  packetbuf_reference(bbb, 4);
  printf("-test2-1-\n");
  if(packetbuf_is_reference()){
    print_packetbuf();
  }
  printf("-test2-2-\n");
  strcpy(bbb, "2n12h2h3");
  print_packetbuf();


  /* test3: copyfrom_hdr */
  printf("-test3-\n");
  packetbuf_copyfrom_hdr(ccc, 5);
  print_packetbuf();

  /* test4: compact */
  printf("-test4-\n");
  packetbuf_compact();
  print_packetbuf();

  /* test5: queuebuf */
  packetbuf_clear();
  packetbuf_copyfrom(aaa, 4);
  packetbuf_copyfrom_hdr(ccc, 5);
  packet = queuebuf_new_from_packetbuf();

  packetbuf_clear();
  packetbuf_copyfrom(bbb, 4);
  packetbuf_copyfrom_hdr(aaa, 4);
  packet = queuebuf_new_from_packetbuf();

  printf("-test5-1-\n");
  queuebuf_to_packetbuf(packet);
  print_packetbuf();
  printf("-test5-2-\n");
  queuebuf_to_packetbuf(packet);
  print_packetbuf();

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

void print_packetbuf()
{
  printf("len: %u | %u\n", packetbuf_hdrlen(), packetbuf_datalen());
  if(packetbuf_hdrlen()){
    printf("header: %s\n", (char*) packetbuf_hdrptr());
  }
  if(packetbuf_datalen()){
    printf("data: %s\n", (char*) packetbuf_dataptr());
  }
  if(packetbuf_is_reference()){
    printf("reference: %s\n", (char*) packetbuf_reference_ptr());
  }
}

void packetbuf_copyfrom_hdr(const void* from, uint16_t len)
{
  uint16_t cp_len = (len<PACKETBUF_HDR_SIZE)?len:PACKETBUF_HDR_SIZE;
  if(packetbuf_hdralloc(cp_len))
    memcpy(packetbuf_hdrptr(), from, cp_len);
}
