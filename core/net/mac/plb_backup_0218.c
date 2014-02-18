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
#define PRINTF(...) printf("[PLB] ");printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define DEBUG_PACKET 0

#define DEBUG_POWER_CYCLE 0
#if DEBUG_POWER_CYCLE
#define PRINT_P(...) printf("[PLB] ");printf(__VA_ARGS__)
#else
#define PRINT_P(...)
#endif

/*---------------------------------------------------------------------------*/
/* Constans */
#define RTIMER_ARCH_MSECOND RTIMER_ARCH_SECOND/1000
#define STROBE_NUM_MAX 50 //3 //kdw
#define PC_ON_TIME RTIMER_ARCH_MSECOND*100
#define PC_OFF_TIME RTIMER_ARCH_MSECOND*100
//#define PC_ON_TIME (RTIMER_ARCH_SECOND / 160) //JJH3
//#define PC_OFF_TIME (RTIMER_ARCH_SECOND / NETSTACK_RDC_CHANNEL_CHECK_RATE - PC_ON_TIME) //JJH3
#define MAX_STROBE_SIZE 100
#define MAX_ACK_SIZE 100
#define MAX_SYNC_SIZE 100

#define INTER_PACKET_INTERVAL              RTIMER_ARCH_SECOND / 5000
#define ACK_LEN 100
#define AFTER_ACK_DETECTECT_WAIT_TIME      RTIMER_ARCH_SECOND / 1000
/*---------------------------------------------------------------------------*/
// Static variables : PACKET_TYPE
#define BEACON_SD 			0x02	// '00000010'
#define BEACON_SD_ACK 		0x03	// '00000011'
#define BEACON_DS 			0x04	// '00000100'
#define BEACON_DS_ACK		0x05	// '00000101'
#define PREAMBLE	 		0x08	// '00001000'
#define PREAMBLE_ACK 		0x09	// '00001001'
#define PREAMBLE_ACK_DATA 	0x19	// '00011001'
#define DATA 				0x10	// '00010000'
#define DATA_ACK			0x11	// '00010001'
#define	SYNC_START			0x20	// '00100000'
#define SYNC_REQ			0x21	// '00100001'
#define	SYNC_ACK			0x42	// '01000010'
#define	SYNC_END			0x80	// '10000000'
/*---------------------------------------------------------------------------*/
static struct rtimer rt;
static struct pt pt;

static int is_init;
static int is_plb_on;
static int is_radio_on;

static int has_data;
static int send_req;
static int wait_packet;

static int c_wait;

static rimeaddr_t addr_next;
static rimeaddr_t addr_prev;
static rimeaddr_t addr_ack;

static uint8_t *dataptr_temp;
static int temp_len;

// send
static mac_callback_t sent_callback;
static void* sent_ptr;
/*---------------------------------------------------------------------------*/
static void plb_init(void);
static void plb_send(mac_callback_t sent, void *ptr);
static void plb_send_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list);
static void plb_input(void);
static int plb_on(void);
static int plb_off(int keep_radio_on);
static unsigned short plb_channel_check_interval(void);
/*---------------------------------------------------------------------------*/
static int plb_beacon_sd(void);
static int plb_beacon_ds(void);
static char plb_powercycle(void);
static char plb_send_strobe(rimeaddr_t *dst, int *acked, uint8_t type);
static void radio_on();
static void radio_off();
static int plb_create_header(rimeaddr_t *dst, uint8_t type);
static int plb_wait_ack(uint8_t sending_type);
static int plb_wait_data_ack(uint8_t sending_type);
static int plb_send_data(mac_callback_t sent, void *ptr);
static int plb_send_sync_start(void);
static int plb_send_sync(uint8_t type);
static void print_packet(uint8_t *packet, int len);
static void hold_time(rtimer_clock_t interval);
/*---------------------------------------------------------------------------*/
static int
plb_beacon_sd(void)
{
  int acked = 0;
  PRINTF("plb_beacon_sd;  to %u.%u \n",addr_next.u8[0],addr_next.u8[1]);

  // send strobe
  if(plb_send_strobe(&addr_next, &acked, BEACON_SD) < 0){
    return MAC_TX_ERR_FATAL;
  }

  // check ack
  if(acked == 1){
    PRINTF("plb_beacon_sd : acked -> set c_wait\n");
    c_wait = 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
plb_beacon_ds(void)
{
  int acked = 0;
  PRINTF("plb_beacon_ds;  to %u.%u \n",addr_prev.u8[0],addr_prev.u8[1]);

  if (addr_prev.u8[0] == 0 && addr_prev.u8[1] == 0)
  {
	  PRINTF("plb_beacon_ds : no beacon (first node)\n");
	  return -1;
  }

  /* send beacon */
  if(plb_send_strobe(&addr_prev, &acked,BEACON_DS) < 0){
    return MAC_TX_ERR_FATAL;
  }

  /* wait for data */
  if(acked == 1){
	PRINTF("plb_beacon_ds : acked\n");
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static char
plb_powercycle(void)
{

  PRINT_P("plb_powercycle [sr:%d cw:%d]\n",send_req,c_wait);

  PT_BEGIN(&pt);
  while(1) {
    // check on/send state
    if(send_req == 1 && c_wait==1){
    	PRINTF("plb_powercycle send DATA <start>\n");
    	send_req = 0;	//avoid repeat sending
    	plb_send_data(sent_callback, sent_ptr);
    	PRINTF("plb_powercycle send DATA <end>\n");
    	radio_off();
    	NETSTACK_MAC.input();
    }

    /* on */
    radio_on();
    rtimer_set(&rt, RTIMER_NOW() + PC_ON_TIME, 1,
	       (void (*)(struct rtimer *, void *))plb_powercycle, NULL);
    PT_YIELD(&pt);

    /* off */
    if(wait_packet == 0){
      radio_off();
    }
    else if (wait_packet > 0)
    {
    	wait_packet = 0;
    }
    rtimer_set(&rt, RTIMER_NOW() + PC_OFF_TIME, 1,
	       (void (*)(struct rtimer *, void *))plb_powercycle, NULL);
    PT_YIELD(&pt);
  }
  PT_END(&pt);
}
/*---------------------------------------------------------------------------*/
static char
plb_send_strobe(rimeaddr_t *dst, int *acked, uint8_t type)
{
  PRINTF("plb_send_strobe  [dst: %u.%u] [type: %x]\n",dst->u8[0],dst->u8[1],type);

  uint8_t strobe[MAX_STROBE_SIZE];
  int strobe_len = 0;
  rtimer_clock_t wt;

  // Make PLB header
  packetbuf_clear();
  if( (strobe_len = plb_create_header(dst,type)) < 0 ){
    return -1;
  }

  // Make packet -> strobe
  strobe_len = strobe_len +1;	// assign space for packet type
  if( strobe_len > (int)sizeof(strobe)) {
    /* Failed to send */
    PRINTF("plb: send failed, too large header\n");
    return -1;
  }
  memcpy(strobe, packetbuf_hdrptr(), strobe_len);

  /* Send beacon and wait ack : STROBE_NUM_MAX times */
  int strobe_num = 0;
  radio_on();

  while((strobe_num < STROBE_NUM_MAX) && ((*acked) == 0)){
    PRINTF("plb_send_strobe : strobe %d\n",strobe_num);

#if DEBUG_PACKET
    print_packet(strobe, strobe_len);
#endif

    if(NETSTACK_RADIO.send(strobe, strobe_len) != RADIO_TX_OK){
      return -1;
    }
    strobe_num ++;

    (*acked) = plb_wait_ack(type);
    if(*acked){
      PRINTF("ack! return: %d\n", *acked);
    }
  }
  radio_off();
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Put common frame header into packetbuf */
static int
plb_create_header(rimeaddr_t *dst, uint8_t type)
{
	PRINTF("plb_create_header\n");
	int i =0;

	/* Set address */
	packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &rimeaddr_node_addr);
	packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, dst);

	// setting a type of packet at the dataptr's first byte(8bit).
	int length;
	length=packetbuf_datalen();
	uint8_t *dataptr_temp;
	dataptr_temp=(uint8_t *)packetbuf_dataptr();
	for(i=length;i>0;i--)
	{
		dataptr_temp[i]=dataptr_temp[i-1]; //dataptr data shift to insert type at first byte
	}
	dataptr_temp[0]=0;
	dataptr_temp[0]|=type;

	packetbuf_set_datalen(++length);

	/* Create frame */
	int len =0;
	len = NETSTACK_FRAMER.create();

	PRINTF("plb_create_header length: %d\n",len);
	return len;
}
/*---------------------------------------------------------------------------*/
static int
plb_wait_ack(uint8_t sending_type)
{
	PRINTF("plb_wait_ack\n");
	uint8_t ackbuf[ACK_LEN + 2];
	uint8_t type;
	int len;

	wait_packet = 1;

	int ack_received = 0;
	hold_time(INTER_PACKET_INTERVAL);

	/* Check for incoming ACK. */
	if ((NETSTACK_RADIO.receiving_packet() ||
	NETSTACK_RADIO.pending_packet() ||
	NETSTACK_RADIO.channel_clear() == 0)) {

		PRINTF("plb_wait_ack: has input!\n");
		hold_time(AFTER_ACK_DETECTECT_WAIT_TIME);

		len = NETSTACK_RADIO.read(ackbuf, ACK_LEN);
		type = ackbuf[len - 1];

		if (((sending_type + 1) & type) == type) {
			ack_received = 1;
			if (type==PREAMBLE_ACK_DATA) {
				ack_received = 2; // preamble_ack_data JJH
			}
			PRINTF("plb_wait_ack : ACKED\n"); // JJH
		}
	}
	wait_packet = 0;
	return ack_received;
}
/*---------------------------------------------------------------------------*/
static int
plb_wait_data_ack(uint8_t sending_type)
{
	PRINTF("plb_wait_data_ack\n");
	uint8_t ackbuf[ACK_LEN + 2];
	uint8_t type;
	int len;

	wait_packet = 1;

	int ack_received = 0;
	hold_time(INTER_PACKET_INTERVAL*5000);//JJH3

	/* Check for incoming ACK. */
	if ((NETSTACK_RADIO.receiving_packet() ||
	NETSTACK_RADIO.pending_packet() ||
	NETSTACK_RADIO.channel_clear() == 0)) {

		PRINTF("plb_wait_ack: has input!\n");
		hold_time(AFTER_ACK_DETECTECT_WAIT_TIME);

		len = NETSTACK_RADIO.read(ackbuf, ACK_LEN);
		type = ackbuf[len - 1];

		if (((sending_type + 1) & type) == type) {
			ack_received = 1;
			if (type==PREAMBLE_ACK_DATA) {
				ack_received = 2; // preamble_ack_data JJH
			}
			PRINTF("plb_wait_ack : ACKED\n"); // JJH
		}
	}
	wait_packet = 0;
	return ack_received;
}
/*---------------------------------------------------------------------------*/
static void
plb_send_ack(uint8_t type){

	uint8_t ack[MAX_ACK_SIZE];
	int ack_len = 0;

	//set address from input packet
	addr_ack.u8[0]=packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[0];
	addr_ack.u8[1]=packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[1];

	PRINTF("plb_send_ack [dst: %u.%u] [type: %x]\n",  addr_ack.u8[0], addr_ack.u8[1], type);

	packetbuf_clear();
	if ((ack_len = plb_create_header(&addr_ack, type)) < 0) {
		PRINTF("ERROR: plb_create_header ");
		return;
	}
	ack_len++;


	//Make ack frame//
	if (ack_len > (int) sizeof(ack)) {
		// Failed to send //
		PRINTF("plb: send failed, too large header\n");
		return;
	}

	memcpy(ack, packetbuf_hdrptr(), ack_len);

	// Send beacon and wait ack : STROBE_NUM_MAX times //
	radio_on();

#if DEBUG_PACKET
	print_packet(ack, ack_len);
#endif

	if (NETSTACK_RADIO.send(ack, ack_len) != RADIO_TX_OK) {
		PRINTF("ERROR: plb ack send");
		return;
	}
	radio_off();
	return;
}
/*---------------------------------------------------------------------------*/
static int
plb_send_data(mac_callback_t sent, void *ptr)
{

	PRINTF("plb_send_data\n");

	int ret; //what is it? JJH
	int last_sent_ok = 0;
	int acked;
	int temp=0;

	acked = 0;

	if(packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE)==0) //if packet_type ==0, DATA JJH_START
	{
		plb_send_strobe(packetbuf_addr(PACKETBUF_ADDR_NEXT),&acked,PREAMBLE);
		if(acked==1)
		{
			PRINTF("plb_send_data DATA_PREAMBLE_ACKED!\n");
			packetbuf_clear();
			packetbuf_copyfrom(dataptr_temp,temp_len);
			if (plb_create_header(packetbuf_addr(PACKETBUF_ADDR_NEXT),DATA) < 0)
			{
				PRINTF("ERROR: plb_create_header ");
				return -1; //ERROR case : -1
			}
//			hold_time(INTER_PACKET_INTERVAL * 5000); //future
			radio_on();
			PRINTF("plb_send_data send DATA packet\n");
			print_packet(packetbuf_hdrptr(), packetbuf_totlen());

			if(NETSTACK_RADIO.send(packetbuf_hdrptr(), packetbuf_totlen())!=RADIO_TX_OK)
			{
				PRINTF("plb_send_data DATA error!\n");
				return -1; //ERROR case
				ret = MAC_TX_ERR;
			}
			acked=plb_wait_data_ack(DATA); //just once?

			if(acked==1) //data ack received
			{
				PRINTF("plb_send_data DATA_ACKED!\n");
				last_sent_ok=1;
				ret=MAC_TX_OK;
			}
			else if(!acked)
			{
				PRINTF("plb_send_data DATA error: no ack!\n");
				ret=MAC_TX_ERR;
			}
			//sent connect
			mac_call_sent_callback(sent, ptr, ret, 1);
			radio_off();
			packetbuf_clear(); // add kdw
			return last_sent_ok;
		}
		else if(acked==2)//if receive preamble ack data
		{
			PRINTF("PREAMBLE ACK DATA RECEIVED!\n");
		}
		else//do not receive any ack, do nothing
		{
			PRINTF("DO NOT RECEIVED PREAMBLE ACK : error!!!\n");
		}

	}//JJH_END

	return 0;
}
/*---------------------------------------------------------------------------*/
static int
plb_send_sync_start(void) //kdw sync
{
	PRINTF("(sync) plb_send_sync_start\n");
	int acked = 0;

	wait_packet = 2;	//wait_packet: sync
	if (plb_send_strobe(&addr_prev, &acked, SYNC_START) < 0) {
		return MAC_TX_ERR_FATAL;
	}

	if (acked == 1) {
		PRINTF("(sync) plb_send_sync_start [recv : sync_req]\n");
		// fill this
		// send sync_ack !!!
		plb_send_sync(SYNC_ACK);
	}
	return 0;

}
/*---------------------------------------------------------------------------*/
static int
plb_send_sync(uint8_t type) //kdw sync
{
	PRINTF("(sync) plb_send_sync [type : %x]\n",type);

	wait_packet = 2;	//wait_packet: sync
	uint8_t sync[MAX_SYNC_SIZE];
	int sync_len = 0;
	int sync_data_len = 1;	//1 byte for app check
	int sync_hdr_len = 0;
	rimeaddr_t * temp_addr;

	if (type == SYNC_REQ){
		temp_addr = &addr_next;
//		put data
//		sync_data_len += len_clock * 1;
	}
	else if (type == SYNC_ACK)
	{
		temp_addr = &addr_prev;
//		put data
//		sync_data_len += len_clock * 3;
	}
	else if (type == SYNC_END){
		temp_addr = &addr_next;
	}
	else //error
	{
		PRINTF("ERROR: plb_create_header ");
		return -1;
	}



	PRINTF("[sync] plb_send_sync| type: %x dst: %u.%u\n",type, temp_addr->u8[0], temp_addr->u8[1]);

	if ((sync_hdr_len = plb_create_header(temp_addr, type)) < 0)
			{
		PRINTF("ERROR: plb_create_header ");
		return -1;
	}
	sync_len = sync_hdr_len + sync_data_len;
	if (sync_len > (int) sizeof(sync)) {
		// Failed to send //
		PRINTF("plb: send failed, too large header\n");
		return -1;
	}

	memcpy(sync, packetbuf_hdrptr(), sync_hdr_len);


	// Send sync//
	radio_on();
	PRINTF("[sync] send strobe ack \n");
#if DEBUG_PACKET
	print_packet(sync, sync_len);
#endif
	if (NETSTACK_RADIO.send(sync, sync_len) != RADIO_TX_OK) {
		PRINTF("ERROR: plb ack send");
		return -1;
	}
//	radio_off();

	return type;
}
/*---------------------------------------------------------------------------*/
static void
print_packet(uint8_t *packet, int len)
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
    	  printf("1");
      }
      else{
    	  printf("0");
      }
    }
    printf(" ");
  }
  printf("\n");
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
static void
radio_on(){
  PRINT_P("radio_on\n");
  if(is_radio_on ==0){
    NETSTACK_RADIO.on();
    is_radio_on = 1;
  }
}
static void
radio_off(){
  PRINT_P("radio_off\n");
  if(is_radio_on ==1){
    NETSTACK_RADIO.off();
    is_radio_on = 0;
  }
}
/*---------------------------------------------------------------------------*/
static void
plb_init(void)
{
  PRINTF("plb_init\n");

  is_init = 1;
  PT_INIT(&pt);

  // init value
  is_plb_on = 0;
  is_radio_on = 0;
  has_data = 0;
  send_req = 0;
  wait_packet = 0;
  c_wait = 0;
  dataptr_temp=(uint8_t*)malloc(sizeof(uint8_t)*temp_len); // need to be free JJH3
  // set address
  addr_next.u8[0] = rimeaddr_node_addr.u8[0]+1;
  if(rimeaddr_node_addr.u8[0] > 0){
	  addr_prev.u8[0] = rimeaddr_node_addr.u8[0]-1;
  }
  else{
	  addr_prev.u8[0] = 0;
  }
  return;
}
/*---------------------------------------------------------------------------*/
static void
plb_send(mac_callback_t sent, void *ptr)
{
  PRINTF("plb_send\n");

  if ( packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE) == 0 )	//data
   {
 	  PRINTF("plb_send : DATA\n");
 	  send_req = 1;
 	  sent_callback = sent;
 	  sent_ptr = ptr;
 	 //packetbuf_clear_hdr();
 	  temp_len=packetbuf_datalen();
 	  packetbuf_copyto(dataptr_temp);
 	  print_packet(dataptr_temp,packetbuf_totlen());//JJH3
   }
   //kdw sync
   else if ( packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE) == 1 ) //sync
   {
 		sent_callback = sent;
 		sent_ptr = ptr;
 		plb_send_sync_start();
   }
   else // error
   {
 	  mac_call_sent_callback(sent, ptr, MAC_TX_ERR, 1); //error   fill this
   }

  return;
}
/*---------------------------------------------------------------------------*/
/*
 * made by nullrdc
 */
static void
plb_send_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list)
{
  PRINTF("plb_send_list\n");
  while(buf_list != NULL) {
    /* We backup the next pointer, as it may be nullified by
     * mac_call_sent_callback() */
    struct rdc_buf_list *next = buf_list->next;
    int last_sent_ok;

    queuebuf_to_packetbuf(buf_list->buf);
//    last_sent_ok = plb_send_data(sent, ptr); //removed by kdw
        last_sent_ok = 0; //kdw
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
/*
 * BEACON_SD,		BEACON_SD_ACK 보내줌,
 * BEACON_SD_ACK, 	무시
 * BEACON_DS,		BEACON_DS_ACK 보내줌, c_wait set
 * BEACON_DS_ACK, 	무시
 * PREAMBLE,		power cycle data wait 모드
 * PREAMBLE_ACK, 	무시
 * PREAMBLE_ACK_DATA 	무시
 * DATA,			app 으로 올림,
 * DATA_ACK,		끝 아무것도 딱히 안해도됨
 * SYNC_START,		SYNC_REQ 전송, time stamp 찍어서
 * SYNC_REQ,		SYNC_ACK 전송, time stamp 찍어서
 * SYNC_ACK,		SYNC_END 전송, time stamp,  app 으로 올려서 계산
 * SYNC_END			app 으로 올림
 */
static void
plb_input(void)
{

if (NETSTACK_FRAMER.parse() >= 0) {
	if (rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
			&rimeaddr_node_addr)) {

#if DEBUG_PACKET
		print_packet(packetbuf_dataptr(),packetbuf_datalen());
#endif

		uint8_t type = ((uint8_t*) packetbuf_dataptr())[0];
		PRINTF("plb_input [src: %d.%d] [type: %x]\n", packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[0],packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[1], type);
		printf("plb_input [src: %d.%d] [type: %x]\n", packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[0],packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[1], type);

		switch (type) {
		case BEACON_SD:
			//		if( a_wait == 0 ){
			plb_send_ack(BEACON_SD_ACK);
//			a_wait = 1;
			//		}
			break;
		case BEACON_DS:
			//		if( c_wait == 0 ){
			plb_send_ack(BEACON_DS_ACK);
			c_wait = 1;
			//		}0x80
			break;
		case PREAMBLE:
			//		if (preamble_got == 0)	{
			if (has_data == 0) {
				plb_send_ack(PREAMBLE_ACK);
				wait_packet = 1;
			} else if (has_data == 1) {
				plb_send_ack(PREAMBLE_ACK_DATA);
			}
//			preamble_got = 1;
			//		}
			break;
		case DATA:
			NETSTACK_MAC.input();
			plb_send_ack(DATA_ACK);
			break;
		case SYNC_START:
			plb_send_sync(SYNC_REQ);
			break;
		case SYNC_REQ:
			break;
		case SYNC_ACK:
			plb_send_sync(SYNC_END);
			NETSTACK_MAC.input();
			break;
		case SYNC_END:
			NETSTACK_MAC.input();
			break;
		}
	} else{
//		PRINTF("THIS INPUT IS NOT FOR US\n");
	}
}
}
/*---------------------------------------------------------------------------*/
static int
plb_on(void)
{
  PRINTF("plb_on\n");
  if(!is_plb_on)
  {
    is_plb_on = 1;

    // init value
    is_radio_on = 0;
    has_data = 0;
    send_req = 0;
    wait_packet = 0;
    c_wait = 0;

    // run procedure
    plb_beacon_sd();
    plb_beacon_ds();
    plb_powercycle();

  }
  else
  {
    PRINTF("already on\n");
    return -1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
plb_off(int keep_radio_on)
{
  PRINTF("plb_off\n");
  if(wait_packet)
  {
    return NETSTACK_RADIO.on();
  }
  else {
    is_plb_on = 0;
    return NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static unsigned short
plb_channel_check_interval(void)
{
  return 0;
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


