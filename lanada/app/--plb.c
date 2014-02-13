/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         A PLB (Periodic Listen & Beacon) implementation that uses framer for headers.
 * \author
 *         Deawoo Kim 	<dwkim@lanada.kaist.ac.kr>
 *         Jinyeob Kim <urmsori@kaist.ac.kr>
 *         S.H Kim <>
 */

#include "net/mac/plb.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/netstack.h"
#include "sys/pt.h"
#include "sys/rtimer.h"
#include "net/rime.h"
#include <string.h>/*need?*/
#include <stdio.h>

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define PRINT_FUNC 1

/*---------------------------------------------------------------------------*/
/* Constans */
#define RTIMER_ARCH_MSECOND RTIMER_ARCH_SECOND/100
#define BEACON_NUM_MAX 1 //3 //kdw
#define PC_ON_TIME RTIMER_ARCH_MSECOND*100
#define PC_OFF_TIME RTIMER_ARCH_MSECOND*100
#define MAX_BEACON_SIZE 100
#define MAX_ACK_SIZE 100

#define INTER_PACKET_INTERVAL              RTIMER_ARCH_SECOND / 5000
#define ACK_LEN 3
#define AFTER_ACK_DETECTECT_WAIT_TIME      RTIMER_ARCH_SECOND / 1000
/*---------------------------------------------------------------------------*/
/* Static variables */
typedef enum {BEACON_SD,		// '00000010'
				BEACON_SD_ACK,// '00000011'
				BEACON_DS,		// '00000100'
				BEACON_DS_ACK,// '00000101'
				PREAMBLE,		// '00001000'
				PREAMBLE_ACK,	// '00001001'
				DATA,			// '00010000'
				DATA_ACK,		// '00010010'
				SYNC_START,	// '00100000'
				SYNC_REQ,		// '01000000'
				SYNC_ACK,		// '01000010'
				SYNC_END		// '10000000'
				}packet_type; // if you want to change something, please ask JH

static struct rtimer rt;
static struct pt pt;

static int dst_acked;
static int src_acked;

static int has_data;
static int send_req;

static int wait_packet;
static int is_radio_on;

static int is_init = 0;
static int is_plb_on = 0;

static rimeaddr_t addr_src;
static rimeaddr_t addr_dst;
static rimeaddr_t addr_ack;

static int c_wait = 0;

/* send */
static mac_callback_t sent_callback;
static void* sent_ptr;
/*---------------------------------------------------------------------------*/
static char plb_powercycle(void);
static int plb_beacon_sd(void);
static int plb_beacon_ds(void);
static void hold_time(rtimer_clock_t interval);
static void print_packet(uint8_t *packet, int len);
static void plb_init(void);
static int plb_create_header(rimeaddr_t *dst, uint16_t type);

/*---------------------------------------------------------------------------*/
static int
plb_on(void)
{
  PRINTF("plb_on\n");

  if(!is_plb_on){
    is_plb_on = 1;
    c_wait = 0;
    plb_beacon_sd();
    plb_beacon_ds();
    plb_powercycle();
  }
  else{
    PRINTF("already on\n");
  }

  return 0;
}
static int
plb_off(int keep_radio_on)
{
#if PRINT_FUNC
  printf("plb_off\n");
#endif
  if(wait_packet) {
    return NETSTACK_RADIO.on();
  } else {
    is_plb_on = 0;
    return NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static void
radio_on(){
//  PRINTF("radio_on\n");

  if(is_radio_on ==0){
    NETSTACK_RADIO.on();
    is_radio_on = 1;
  }
}
static void
radio_off(){

//  PRINTF("radio_off\n");

  if(is_radio_on ==1){
    NETSTACK_RADIO.off();
    is_radio_on = 0;
  }
}
/*---------------------------------------------------------------------------*/
static int
plb_send_data(mac_callback_t sent, void *ptr)
{

  PRINTF("send_one_packet\n");

  int ret;
  int last_sent_ok = 0;

  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &rimeaddr_node_addr);

  if(NETSTACK_FRAMER.create() < 0) {
    /* Failed to allocate space for headers */
    PRINTF("nullrdc: send failed, too large header\n");
    ret = MAC_TX_ERR_FATAL;
  } 
  else {
    switch(NETSTACK_RADIO.send(packetbuf_hdrptr(), packetbuf_totlen())) {
    case RADIO_TX_OK:
      ret = MAC_TX_OK;
      break;
    case RADIO_TX_COLLISION:
      ret = MAC_TX_COLLISION;
      break;
    case RADIO_TX_NOACK:
      ret = MAC_TX_NOACK;
      break;
    default:
      ret = MAC_TX_ERR;
      break;
    }
  }

  if(ret == MAC_TX_OK) {
    last_sent_ok = 1;
  }
  mac_call_sent_callback(sent, ptr, ret, 1);
  return last_sent_ok;
}
/*---------------------------------------------------------------------------*/
static void
plb_send(mac_callback_t sent, void *ptr)
{
  PRINTF("plb_send\n");

  send_req = 1;
  sent_callback = sent;
  sent_ptr = ptr;
}
/*---------------------------------------------------------------------------*/
static void
plb_send_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list)
{
  while(buf_list != NULL) {
    /* We backup the next pointer, as it may be nullified by
     * mac_call_sent_callback() */
    struct rdc_buf_list *next = buf_list->next;
    int last_sent_ok;

    queuebuf_to_packetbuf(buf_list->buf);
    last_sent_ok = plb_send_data(sent, ptr);

    /* If packet transmission was not successful, we should back off and let
     * upper layers retransmit, rather than potentially sending out-of-order
     * packet fragments. */
    if(!last_sent_ok) {
      return;
    }
    buf_list = next;
  }
}
/*---------------------------------------------------------------------------*/
static void
plb_input(void)
{
  PRINTF("plb_input\n");
  uint8_t type = 1;	// type: beacon, data, sync
  uint8_t strobe_type = ((uint8_t *) packetbuf_dataptr())[9];	//strobe_type: beacon_sd,beacon_ds,preable

  printf("type: %u\n", strobe_type);
  int original_datalen;
  uint8_t *original_dataptr;

  original_datalen = packetbuf_datalen();
  original_dataptr = packetbuf_dataptr();
  print_packet(original_dataptr, original_datalen);
  if(NETSTACK_FRAMER.parse() < 0) {
    PRINTF("nullrdc: failed to parse %u\n", packetbuf_datalen());
    return;
  } 

  if (type == 1) // if type == beacon
  {
/*
	  // send ack
		uint8_t ack[MAX_ACK_SIZE];
		int ack_len = 0;
		int common_len = 0;
		PRINTF("plb_send_ack; dst: %u.%u\n",packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[0],packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[1]);
		if ((common_len = plb_create_header(packetbuf_addr(PACKETBUF_ADDR_SENDER), (uint16_t)4)) < 0) // type 4 = ack
		{
			PRINTF("ERROR: plb_create_header ");
			return;
		}
		//Make ack frame//
		ack_len = common_len + 1;//sizeof(struct plb_beacon_hdr);
		if (ack_len > (int) sizeof(ack)) {
			// Failed to send //
			PRINTF("plb: send failed, too large header\n");
			return ;
		}
		memcpy(ack, packetbuf_hdrptr(), common_len);
		ack[common_len] = 4;// type 4 = ack
		PRINTF("(ack)pbe len: %u | %u\n", packetbuf_hdrlen(), packetbuf_datalen()); PRINTF("data: %s\n", (char*) packetbuf_dataptr());

		// Send beacon and wait ack : BEACON_NUM_MAX times //
		radio_on();
		PRINTF("plb: send strobe ack \n");
		print_packet(ack, ack_len);
		if (NETSTACK_RADIO.send(ack, ack_len) != RADIO_TX_OK) {
			PRINTF("ERROR: plb ack send");
			return ;
		}
		radio_off();
*/
		//////////////////////////////////////////

		if (strobe_type == 1)	//beacon_sd
		{
			PRINTF("input: beacon_sd -> set c_wait \n");
			c_wait = 1;
		}
		else if (strobe_type == 3) //preamble // send ack and wait for packet
		{
			PRINTF("input: preamble -> wait for listen\n");
		}
  }

  NETSTACK_MAC.input();

}
/*---------------------------------------------------------------------------*/
static unsigned short
plb_channel_check_interval(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static void
plb_init(void)
{

  PRINTF("plb_init\n");

  PT_INIT(&pt);
  dst_acked = 0;
  src_acked = 0;
  has_data = 0;
  wait_packet = 0;
  is_init = 1;

  //plb_on //not use this, plb_on is called from app
  //  rtimer_set(&rt, RTIMER_NOW() + RTIMER_ARCH_SECOND, 1,(void (*)(struct rtimer *, void *))plb_on, NULL);

  /* init addr_src, addr_dst */
  addr_dst.u8[0] = rimeaddr_node_addr.u8[0]+1;
  if(rimeaddr_node_addr.u8[0] > 0){
    addr_src.u8[0] = rimeaddr_node_addr.u8[0]-1;
  }
  else{
    addr_src.u8[0] = 0;
  }


}
/*---------------------------------------------------------------------------*/
/* Put common frame header into packetbuf */
static int
plb_create_header(rimeaddr_t *dst, uint16_t type)
{
  PRINTF("plb_create_header\n");

  /* Set address */
  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &rimeaddr_node_addr);

//  packetbuf_attr_t temp_type = type; // for framer.create; not used now
  int i=0;
  for(i=0; i<RIMEADDR_SIZE; i++){
    packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, dst);
  }

// setting a type of packet at the dataptr's first byte(8bit).
  int length;
  length=packetbuf_datalen();
  uint8_t *dataptr_temp;
  dataptr_temp=(uint8_t *)packetbuf_dataptr();
  for(i=length;i>0;i--)
  {
	  dataptr_temp[i]=dataptr_temp[i-1]; //dataptr data shift to insert type at first byte
  }

  switch(type)
  {
    case BEACON_SD:dataptr_temp[0]|=0x02;
    	break;
    case BEACON_SD_ACK:dataptr_temp[0]|=0x03;
    	break;
    case BEACON_DS:dataptr_temp[0]|=0x04;
    	break;
    case BEACON_DS_ACK:dataptr_temp[0]|=0x05;
    	break;
    case PREAMBLE:dataptr_temp[0]|=0x08;
    	break;
    case PREAMBLE_ACK:dataptr_temp[0]|=0x09;
    	break;
    case DATA:dataptr_temp[0]|=0x10;
    	break;
    case DATA_ACK:dataptr_temp[0]|=0x12;
    	break;
    case SYNC_START:dataptr_temp[0]|=0x20;
    	break;
    case SYNC_REQ:dataptr_temp[0]|=0x40;
    	break;
    case SYNC_ACK:dataptr_temp[0]|=0x42;
    	break;
    case SYNC_END:dataptr_temp[0]|=0x80;
    	break;
  }
  packetbuf_set_datalen(++length);

  /* Create frame */
  int len =0;
  len = NETSTACK_FRAMER.create();

  return len;
}
/*---------------------------------------------------------------------------*/
static int
plb_wait_ack(void)
{

//  PRINTF("plb_wait_ack\n");

  wait_packet = 1;
  
  int ack_received = 0;
  hold_time(INTER_PACKET_INTERVAL*10);
  
  /* Check for incoming ACK. */
  if((NETSTACK_RADIO.receiving_packet() ||
      NETSTACK_RADIO.pending_packet() ||
      NETSTACK_RADIO.channel_clear() == 0)) {
//	  PRINTF("plb_wait_ack: have some packet\n");
    int len;
    uint8_t ackbuf[ACK_LEN + 2];
      
    hold_time(AFTER_ACK_DETECTECT_WAIT_TIME);
    
    len = NETSTACK_RADIO.read(ackbuf, ACK_LEN);
    
    if(len == ACK_LEN) {
      /* fill this : ack check */
      ack_received = 1;
      PRINTF("plb_wait_ack: ACKED\n");
    }
  }

  wait_packet = 0;
    
  return ack_received;
}
/*---------------------------------------------------------------------------*/
static char
plb_send_strobe(rimeaddr_t *dst, int *acked, uint16_t type)
{
//  PRINTF("plb_send_strobe; dst: %u.%u\n",dst->u8[0],dst->u8[1]);

  uint8_t beacon[MAX_BEACON_SIZE];
  int beacon_len = 0;
  int common_len = 0;
  /*Make common frame*/
  /* fill this: /net/rime/rimeaddr.h addr modify is needed */
  if( (common_len = plb_create_header(dst,type)) < 0 ){
    return -1;
  }
  /*Make beacon frame*/
  beacon_len = common_len + sizeof(struct plb_beacon_hdr); //fill this: struct is needed
  if( beacon_len > (int)sizeof(beacon)) {
    /* Failed to send */
    PRINTF("plb: send failed, too large header\n");
    return -1;
  }
  memcpy(beacon, packetbuf_hdrptr(), common_len);
  beacon[common_len] = (uint8_t)type;//TYPE_BEACON;	////////////////////////////////////////////////
  PRINTF("pbe len: %u | %u\n", packetbuf_hdrlen(), packetbuf_datalen());
  PRINTF("data: %s\n", (char*) packetbuf_dataptr());

  /* Send beacon and wait ack : BEACON_NUM_MAX times */
  int beacon_num = 0;
  radio_on();
  while((beacon_num < BEACON_NUM_MAX) && ((*acked) == 0)){
    print_packet(beacon, beacon_len);
    if(NETSTACK_RADIO.send(beacon, beacon_len) != RADIO_TX_OK){
      return -1;
    }
    beacon_num ++;

    /* wait for beacon_ack*/
    (*acked) = plb_wait_ack();
    if(*acked){
      printf("ack!\n");
    }
  }
  radio_off();
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
plb_beacon_sd(void)
{
  PRINTF("plb_beacon_sd;  to %u.%u \n",addr_dst.u8[0],addr_dst.u8[1]);

  /* send beacon */
  if(plb_send_strobe(&addr_dst, &dst_acked,1) < 0){
    return MAC_TX_ERR_FATAL;
  }
  
  /* send data 
   * if this node is end node
   */
  if(dst_acked){
    PRINTF("beacon_sd_acked : set c_wait");
    /*if(send_req){
      if(has_data){
      // fill this: send data to destination 
      }
    }*/
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
plb_beacon_ds(void)
{

  PRINTF("plb_beacon_ds;  to %u.%u \n",addr_src.u8[0],addr_src.u8[1]);

  /* send beacon */
  if(plb_send_strobe(&addr_src, &src_acked,2) < 0){
    return MAC_TX_ERR_FATAL;
  }
  
  /* wait for data */
  if(src_acked){
    PRINTF("beacon_sd_acked : set c_wait");
    c_wait = 1;
    /*fill this*/
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static char
plb_powercycle(void)
{

//  PRINTF("plb_powercycle\n");

  PT_BEGIN(&pt);

  while(1) {
    /* check on/send state */
    if(send_req && has_data){
      plb_send_data(sent_callback, sent_ptr);
    }

    /* on */
    radio_on();
    rtimer_set(&rt, RTIMER_NOW() + PC_ON_TIME, 1,
	       (void (*)(struct rtimer *, void *))plb_powercycle, NULL);
    PT_YIELD(&pt);
    
    /* off */
    if(wait_packet == 0){
//      radio_off();
    }
    rtimer_set(&rt, RTIMER_NOW() + PC_OFF_TIME, 1,
	       (void (*)(struct rtimer *, void *))plb_powercycle, NULL);
    PT_YIELD(&pt);
  }

  PT_END(&pt);
}
/*---------------------------------------------------------------------------*/
static void hold_time(rtimer_clock_t interval)
{
  rtimer_clock_t rct;
  rct = RTIMER_NOW();
  while(RTIMER_CLOCK_LT(RTIMER_NOW(), rct+interval)){
  }
}
/*---------------------------------------------------------------------------*/
static void print_packet(uint8_t *packet, int len)
{
  int i,j;
  uint8_t num, num2;
  uint8_t jisu;

  for(i=0; i<len; i++){
    num = packet[i];
    for(j=7; j>-1; j--){
      jisu = 1 << j;
      num2 = num&jisu;
      num2 = num2 >> j;
      if(num&jisu){
    	  PRINTF("1");
      }
      else{
    	  PRINTF("0");
      }
    } 
    PRINTF(" ");
  }
  PRINTF("\n");
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver plb_driver = {
  "PLB",
  plb_init,
  plb_send,
  plb_send_list,
  plb_input,
  plb_on,
  plb_off,
  plb_channel_check_interval,
};
/*---------------------------------------------------------------------------*/
