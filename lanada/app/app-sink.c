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
 *         app-sink to collect data from nodes
 * \author
 *         Jinhwan, Jung <jhjung@lanada.kaist.ac.kr>
 */

#include "contiki.h"
#include "net/rime.h"
#include "net/netstack.h" // for using netstack

#include "dev/button-sensor.h"

#include "dev/leds.h"

#include <stdio.h>

/*---------------------------------------------------------------------------*/
PROCESS(app_sink_process, "Sensor network App sink for test start");
AUTOSTART_PROCESSES(&app_sink_process);
/*---------------------------------------------------------------------------*/
//functions declaration
static void Data_print();
/*---------------------------------------------------------------------------*/

static void
recv_uc(struct unicast_conn *c, const rimeaddr_t *from)
{
	uint8_t* packet_temp;
	uint8_t check_bit;
	packet_temp=(uint8_t*)packetbuf_dataptr();
	check_bit=0;

	check_bit|=packet_temp[0]>>6;

	if(check_bit==0)// check_bit == '00'
	{
		Data_print();
	}
	else
	{
		printf("Error occur\n");
	}
	//printf("unicast message received from %d.%d\n",
	// from->u8[0], from->u8[1]);
}
static const struct unicast_callbacks unicast_callbacks = {recv_uc};
static struct unicast_conn uc;

static void
Data_print()
{
	uint16_t length=packetbuf_datalen(); //this length means bit level, ex) length=3 -> dataptr[0]=000xxxxx
	uint8_t index;
	int i; //loop variable
	uint8_t *dataptr_temp;

	dataptr_temp=(uint8_t *)packetbuf_dataptr;

	printf("App-sink Received DATA : ");
	for(i=0;i<length;i++)
	{
		index=i/8;
		if(dataptr_temp[index]>127) //if the value is greater than 127, the MSB of byte is '1'
		{
			printf("1");
		}
		else
		{
			printf("0");
		}
		dataptr_temp[index]=dataptr_temp[index]<<1;
		if(i%8==7)
		{
			printf(" ");
		}
	}
	printf("\n");

}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_sink_process, ev, data)
{
  PROCESS_EXITHANDLER(unicast_close(&uc);)

  PROCESS_BEGIN();

  unicast_open(&uc, 146, &unicast_callbacks);
  printf("Sink channel open\n waiting for data\n");
  while(1) {


  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
