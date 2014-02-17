/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 *         APP layer for sensor network
 * \author
 *         Jinhwan, Jung <jhjun@lanada.kaist.ac.kr>
 */

#include "contiki.h"
#include "net/rime.h"
#include "net/netstack.h" // for using netstack

#include "dev/button-sensor.h"

#include "dev/leds.h"

#include <stdio.h>
#define DEBUGPRINT 1


/*---------------------------------------------------------------------------*/
PROCESS(app_layer_process, "Sensor network App layer start");
AUTOSTART_PROCESSES(&app_layer_process);
/*---------------------------------------------------------------------------*/
//static variables
typedef enum {DATA, SYNC, END, ERROR}packet_type;

static uint8_t result_data;

static packet_type is_data_or_sync;

static int clock_drift;
static uint8_t is_sleep_mode;

/*---------------------------------------------------------------------------*/
//functions declaration
static void Address_setup();
static int Sensor_start();
static uint8_t Sensor_calc(int);
static uint8_t Plb_on();
static uint8_t Send(packet_type);
static void Data_aggregation();
static int Sync_calc();
static void Sync_modifying(int);

/*---------------------------------------------------------------------------*/
//callback functions
static void
recv_uc(struct unicast_conn *c, const rimeaddr_t *from)
{
	uint8_t* packet_temp;
	uint8_t check_bit;
	packet_temp=(uint8_t*)packetbuf_dataptr();
	check_bit=0;

	check_bit|=packet_temp[0]>>6; //6 right shifts
	if(check_bit==0) // check_bit == '00'
	{
		is_data_or_sync=DATA;
	}
	else if(check_bit==2) // check_bit == '10'
	{
		is_data_or_sync=SYNC;
	}
	else if(check_bit==3) // check_bit == '11'
	{
		is_data_or_sync=END;
	}
	else
	{
		is_data_or_sync=ERROR;
	}

	switch(is_data_or_sync)
	{
	case DATA: Data_aggregation();
	Send(DATA);
#if DEBUGPRINT
	printf("Receiving DATA and Sending aggregation data\n");
#endif
	break;
	case SYNC: clock_drift=Sync_calc();
	Sync_modifying(clock_drift);
	Send(SYNC);
#if DEBUGPRINT
	printf("Receiving SYNC and Sending a SYNC start signal\n");
#endif
	break;
	case END: is_sleep_mode=1;
#if DEBUGPRINT
	printf("Receiving END\n");
#endif
	break;
	default:printf("exception case, ERROR occur\n");
	break;
	}
}
/*static void
sent_uc(struct unicast_conn *c, int status, int num_tx)
{
	//printf("unicast message sent status : %d, number of tx : %d\n message is %s\n",status,num_tx,(char*)packetbuf_dataptr());
}
*/ //it is no longer useful
static const struct unicast_callbacks unicast_callbacks = {recv_uc};//sent_uc is not used at the present stage
static struct unicast_conn uc;
/*---------------------------------------------------------------------------*/
//Address function
static void
Address_setup()
{
	rimeaddr_t next_node_addr;
	rimeaddr_t prev_node_addr;
	if(rimeaddr_node_addr.u8[0]==1)
	{
		packetbuf_set_datalen(0);

	}
	next_node_addr.u8[0]=rimeaddr_node_addr.u8[0]+1;
	next_node_addr.u8[1]=rimeaddr_node_addr.u8[1];
	prev_node_addr.u8[0]=rimeaddr_node_addr.u8[0]-1;
	prev_node_addr.u8[1]=rimeaddr_node_addr.u8[1]; //it only can handle the case of MAX_NODE <256

	packetbuf_set_addr(PACKETBUF_ADDR_NOW,&rimeaddr_node_addr);
	packetbuf_set_addr(PACKETBUF_ADDR_NEXT,&next_node_addr);
	packetbuf_set_addr(PACKETBUF_ADDR_PREVIOUS,&prev_node_addr);

#if DEBUGPRINT
	printf("Setting addresses\n");
#endif
	//address setting using packetbuf_set_addr
}
/*---------------------------------------------------------------------------*/
//Sensor functions
static int //return type check
Sensor_start()
{
	//sending a signal to Sensor to operate
#if DEBUGPRINT
	printf("Sensing complete\n");
#endif
	return 1; //return sensing result
}
static uint8_t
Sensor_calc(int data)
{
	//calculate sensing data and return result value 0 or 1
#if DEBUGPRINT
	printf("Sensing calculation complete\n");
#endif
	return 1;
}
/*---------------------------------------------------------------------------*/
//plb signals
static uint8_t
Plb_on()
{

	//sending a signal to plb layer
#if DEBUGPRINT
	printf("Sending a signal to plb\n");
#endif
	NETSTACK_RDC.on();
	return 1;
}

static uint8_t
Send(packet_type type)
{

	if(type==DATA) //if type is DATA
	{
#if DEBUGPRINT
	printf("[APP] send data (%d)\n",result_data);
#endif
		packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,DATA);
		//send DATA to NEXT node
		return unicast_send(&uc, packetbuf_addr(PACKETBUF_ADDR_NEXT));//TR result return
	}
	else//if type is SYNC
	{
		packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,SYNC);
		//send SYNC_start to PREV node
		return unicast_send(&uc, packetbuf_addr(PACKETBUF_ADDR_PREVIOUS));//TR result return
	}
}
/*---------------------------------------------------------------------------*/
//data function
static void
Data_aggregation()
{
	uint16_t length=packetbuf_datalen();
	uint16_t bit_length=(uint16_t)rimeaddr_node_addr.u8[0]+1; //this length means bit level, the bit position that is result data locates
	uint8_t index,shift_amt; //ptr index value, the number of shift operations to apply
	uint8_t *dataptr_temp;
	dataptr_temp=(uint8_t *)packetbuf_dataptr();
	if(bit_length==2) //it means END node case, because of bit_length=node_addr+2
	{
		dataptr_temp[0]=0;// setting hdr of dataptr '00'
	}
	index=bit_length/8;
	shift_amt=8-((bit_length)%8+1);
	dataptr_temp[index]|=result_data<<shift_amt;

	length=index+1;
	packetbuf_set_datalen(length);
	//data aggregation using dataptr
#if DEBUGPRINT
	printf("received data aggregation\n");
#endif
}
/*---------------------------------------------------------------------------*/
//sync functions
static int
Sync_calc()
{
	//calculate Sync
#if DEBUGPRINT
	printf("calculating clock drift\n");
#endif
	return 1; //return clock drift(int)
}

static void
Sync_modifying(int clock_drift)
{
#if DEBUGPRINT
	printf("modifying clock\n");
#endif
	//modifying clock
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(app_layer_process, ev, data)
{
	int sensor_value; // type check
	static struct etimer et;

	PROCESS_EXITHANDLER(unicast_close(&uc);)

	PROCESS_BEGIN();
#if DEBUGPRINT
	printf("PROCESS BEGINNING\n");
#endif
	unicast_open(&uc, 146, &unicast_callbacks);
#if DEBUGPRINT
	printf("Setting a unicast channel\n");
#endif
	Address_setup();



	while(1) {
		is_sleep_mode=0;
		sensor_value=Sensor_start();
		result_data=Sensor_calc(sensor_value);

		Plb_on();
		if(rimeaddr_node_addr.u8[0]==1 && rimeaddr_node_addr.u8[1]==0)
		{
			Data_aggregation();
			Send(DATA);
		}
#if DEBUGPRINT
		printf("waiting until intterupt\n");
#endif
//		PROCESS_WAIT_EVENT_UNTIL(is_sleep_mode);
		while(!is_sleep_mode) // can i replace it with PROCESS_WAIT_UNTIL or some PROCESS function?
		{
			  etimer_set(&et, CLOCK_SECOND/1000);
			  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
			//waiting
		}
#if DEBUGPRINT
		printf("goto sleep mode, wake up after 21days\n");
#endif
		//goto sleep mode and prepare clock to wake up
		/*while(1) // replace it with PROCESS_WAIT_UNTIL/ same as above
		{
			//clock tic tok
		}*/

	}



	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
