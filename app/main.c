/***************************************************************************************************
 ***************************************************************************************************
 *
 *	Copyright (c) 2019, Networked Embedded Systems Lab, TU Dresden
 *	All rights reserved.
 *
 *	Redistribution and use in source and binary forms, with or without
 *	modification, are permitted provided that the following conditions are met:
 *		* Redistributions of source code must retain the above copyright
 *		  notice, this list of conditions and the following disclaimer.
 *		* Redistributions in binary form must reproduce the above copyright
 *		  notice, this list of conditions and the following disclaimer in the
 *		  documentation and/or other materials provided with the distribution.
 *		* Neither the name of the NES Lab or TU Dresden nor the
 *		  names of its contributors may be used to endorse or promote products
 *		  derived from this software without specific prior written permission.
 *
 *	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 *	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************//**
 *
 *	@file					main.c
 *
 *	@brief					main entry point
 *
 *	@version				$Id$
 *	@date					TODO
 *
 *	@author					Fabian Mager
 *
 ***************************************************************************************************

 	@details

	TODO

 **************************************************************************************************/
//***** Trace Settings *****************************************************************************

#include "gpi/trace.h"

// message groups for TRACE messages (used in GPI_TRACE_MSG() calls)
// define groups appropriate for your needs, assign one bit per group
// values > GPI_TRACE_LOG_USER (i.e. upper bits) are reserved
#define TRACE_INFO		GPI_TRACE_MSG_TYPE_INFO

// select active message groups, i.e., the messages to be printed (others will be dropped)
#ifndef GPI_TRACE_BASE_SELECTION
	#define GPI_TRACE_BASE_SELECTION	GPI_TRACE_LOG_STANDARD | GPI_TRACE_LOG_PROGRAM_FLOW
#endif
GPI_TRACE_CONFIG(main, GPI_TRACE_BASE_SELECTION);

//**************************************************************************************************
//***** Includes ***********************************************************************************

#include "mixer/mixer.h"

#include "gpi/tools.h"
#include "gpi/platform.h"
#include "gpi/interrupts.h"
#include "gpi/clocks.h"
#include "gpi/olf.h"

#include GPI_PLATFORM_PATH(radio.h)

#include <nrf.h>
#include <wireless_control.h>
#include <bolt.h>

#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

//**************************************************************************************************
//***** Profile Settings ***************************************************************************

#if WC_PROFILE_MAIN
	#include "gpi/profile.h"
	GPI_PROFILE_SETUP("main.c", 1000, 2);
	#define PROFILE_MAIN(...)				GPI_PROFILE(100, ## __VA_ARGS__)
	#define PROFILE_MAIN_P(priority, ...)	GPI_PROFILE(priority, ## __VA_ARGS__)
#else
	#define PROFILE_MAIN(...)		while (0)
	#define PROFILE_MAIN_P(...)		while (0)
#endif

//**************************************************************************************************
//***** Local Defines and Consts *******************************************************************

#ifndef WEAK_RELEASE_SLOT
	#warning WEAK_RELEASE_SLOT not defined, set it to MX_ROUND_LENGTH / 2
	#define WEAK_RELEASE_SLOT	MX_ROUND_LENGTH / 2
#endif

// important for the log parser
#define PRINT_HEADER()	printf("# ID:%u ", TOS_NODE_ID)

// Function to access aggregate field.
//  |00000000   00000000   0000|0000 | 0000|0000   0|00000|00
//  |          flags           | p1  |  p2 |   n1   |  n2 |
// ATTENTION: Macros assume agg to be a pointer to uint8_t.
// ATTENTION: This is a fixed 2 nodes setting!
#define CONTROL_MSG_LIMIT				1
//#define SET_AGG_FLAG_NODE(agg, node)	( agg[(node - 1) / 8] |= (0x1 << (7 - ((node - 1) % 8))) )
static inline void SET_AGG_FLAG_NODE(uint8_t *agg, uint8_t node) {
	uint8_t byte = (node - 1) / 8;
	uint8_t bit = 7 - ((node - 1) % 8);
	agg[byte] |= (1 << bit);
}
#define SET_PRIO1(agg, p) 				( agg[2] = (agg[2] & 0xF0) | (p & 0x0F) )
#define GET_PRIO1(agg) 					( agg[2] & 0x0F )
#define SET_PRIO2(agg, p)				( agg[3] = ((p & 0x0F) << 4) | (agg[3] & 0x0F) )
#define GET_PRIO2(agg)					( (agg[3] & 0xF0) >> 4 )
#define SET_NODE1(agg, n)                           \
   do                                               \
   {                                                \
	 agg[3] = (agg[3] & 0xF0) | ((n >> 1) & 0x0F);  \
	 agg[4] = ((n & 0x1) << 7) | (agg[4] & 0x7C);   \
   } while(0)
#define GET_NODE1(agg)					( ((agg[3] & 0x0F) << 1) | ((agg[4] & 0x80) >> 7) )
#define SET_NODE2(agg, n)				( agg[4] = (agg[4] & 0x80) | ((n << 2) & 0x7C) )
#define GET_NODE2(agg)					( (agg[4] & 0x7C) >> 2 )


//**************************************************************************************************
//***** Local Typedefs and Class Declarations ******************************************************



//**************************************************************************************************
//***** Forward Declarations ***********************************************************************



//**************************************************************************************************
//***** Local (Static) Variables *******************************************************************

static uint8_t			node_id;
static uint32_t			round;
static uint32_t			initiator_msg_decoded;
static uint32_t			control_msg_decoded;
static uint32_t			prio_msg_decoded;
static uint32_t			msgs_not_decoded;
static uint32_t			msgs_weak;
static uint32_t			msgs_weak_fake;

static uint8_t			rank;
static uint8_t			hash;
static uint8_t			slot_full_rank;
static uint32_t			radio_on_time;

static uint8_t			all_ranks[MX_GENERATION_SIZE - 1]; // subtract control msg
static uint8_t			all_versions[MX_GENERATION_SIZE - 1]; // subtract control msg
static uint8_t			all_priorities[MX_GENERATION_SIZE - 1]; // subtract control msg
static uint8_t			all_currTrigger[MX_GENERATION_SIZE - 1]; // subtract control msg
static uint8_t			all_slot_full_rank[MX_GENERATION_SIZE - 1]; // subtract control msg
static uint32_t			all_radio_on_time[MX_GENERATION_SIZE - 1]; // subtract control msg

static bolt_pkt_t		bolt_agg_pkt = {.type = BOLT_AGG_DATA};
uint8_t * const			agg = bolt_agg_pkt.agg_data.data;
static uint8_t			agg_input[AGGREGATE_SIZE];

//**************************************************************************************************
//***** Global Variables ***************************************************************************

// TOS_NODE_ID is a variable with very special handling: on FLOCKLAB and INDRIYA, its init value
// gets overridden with the id of the node in the testbed during device programming (by calling
// tos-set-symbol (a script) on the elf file). Thus, it is well suited as a node id variable.
// ATTENTION: it is important to have TOS_NODE_ID in .data (not in .bss), otherwise tos-set-symbol
// will not work
uint16_t __attribute__((section(".data")))	TOS_NODE_ID = 0;

//**************************************************************************************************
//***** Local Functions ****************************************************************************

// ATTENTION: This is a fixed 2 nodes setting!

unsigned int all_flags_set(uint8_t *agg)
{
	for (int i = 0; i < MX_NUM_NODES; ++i)
	{
		int byte_index = i / 8;
		int bit_index = 7 - (i % 8);
		if ((agg[byte_index] & (1 << bit_index)) == 0)
			return 0;
	}
	return 1;
}

//**************************************************************************************************

// ATTENTION: This is a fixed 2 nodes setting!

static void merge_all_flags(uint8_t *dst, const uint8_t *src)
{
    // Merge flags for actual number of nodes (each bit represents a node)
    unsigned int num_flag_bytes = (MX_NUM_NODES + 7) / 8;

    for (unsigned int i = 0; i < num_flag_bytes; i++) {
        dst[i] |= src[i];
    }

    // Now merge the upper 4 bits of dst[2] with src[2], if it exists
    if (num_flag_bytes > 2) {
        dst[2] |= (src[2] & 0xF0);
    } else if (num_flag_bytes == 2) {
        // dst[2] exists, but we don’t want to touch lower 4 bits (priority bits)
        dst[2] = (dst[2] & 0x0F) | (src[2] & 0xF0);
    }
}

//**************************************************************************************************

static void agg_rx_cb(volatile uint8_t *agg_is_valid, uint8_t *agg_local, uint8_t *agg_rx)
{
	typedef struct
	{
		uint8_t prio;
		uint8_t node;
	} pn;

	pn local1	= { GET_PRIO1(agg_local), GET_NODE1(agg_local) };
	pn local2	= { GET_PRIO2(agg_local), GET_NODE2(agg_local) };
	pn rx1		= { GET_PRIO1(agg_rx), GET_NODE1(agg_rx) };
	pn rx2		= { GET_PRIO2(agg_rx), GET_NODE2(agg_rx) };

	pn array[4] = {local1, local2, rx1, rx2};

	// Sort elements with respect to their priorities and node IDs.
	uint8_t i, j;
	for (i = 0; i < 4; i++)
	{
		pn cur_element = array[i];
		uint8_t new_element_idx = i;

		for (j = i + 1; j < 4; j++)
		{
			// avoid duplicates
			if (array[j].node == array[new_element_idx].node)
			{
				array[j].node = 0;
				array[j].prio = 0;
				continue;
			}

			if (array[j].prio > array[new_element_idx].prio)
			{
				new_element_idx = j;
			}
			else if (array[j].prio == array[new_element_idx].prio)
			{
				if (array[j].node < array[new_element_idx].node)
				{
					new_element_idx = j;
				}
			}
		}

		// new_element points to the element with the highest priority (and lowest node ID if equal
		// priorities)
		array[i] = array[new_element_idx];
		array[new_element_idx] = cur_element;
	}

	// invalidate aggregate before modification
	*agg_is_valid = 0;

	merge_all_flags(agg_local, agg_rx);
	SET_PRIO1(agg_local, array[0].prio);
	SET_NODE1(agg_local, array[0].node);
	SET_PRIO2(agg_local, array[1].prio);
	SET_NODE2(agg_local, array[1].node);

	// activate aggregate after modification
	*agg_is_valid = 1;
}

//**************************************************************************************************

// print results for the log parser
static void print_results(uint8_t log_id)
{
	unsigned int	slot, slot_min, i;

	#if WC_PROFILE_MAIN
		Gpi_Profile_Ticket	ticket;
		const char			*module_name;
		uint16_t			line;
		uint32_t			timestamp;
		memset(&ticket, 0, sizeof(ticket));

		while (gpi_profile_read(&ticket, &module_name, &line, &timestamp))
		{
			printf("profile %s %4" PRIu16 ": %" PRIu32 "\n", module_name, line, timestamp);
		}
	#endif

	// stats
	// mixer_print_statistics();

	// #define PRINT(n) printf(#n ": %" PRIu16 "\n", mixer_statistics()->n)
	// PRINT(num_sent);
	// PRINT(num_received);
	// PRINT(num_resync);
	// PRINT(num_rx_timeout);
	// // PRINT(slot_full_rank);
	// PRINT(slot_off);
	// #undef PRINT

	// #define PRINT(n) printf(#n ": %luus\n", (unsigned long)gpi_tick_hybrid_to_us(mixer_statistics()->n))
	// PRINT(radio_on_time);
	// #undef PRINT

	slot_full_rank = mixer_statistics()->slot_full_rank;
	radio_on_time = gpi_tick_hybrid_to_us(mixer_statistics()->radio_on_time);

	for (i = 0; i < MX_GENERATION_SIZE; i++)
	{
		if (mixer_stat_slot(i) >= 0) ++rank;
	}

	PRINT_HEADER();
	printf("round=%" PRIu32 " rank=%" PRIu8 " initDec=%" PRIu32 " ctrlDec=%" PRIu32
		   " priDec=%" PRIu32 " notDec=%" PRIu32 " weak=%" PRIu32 " weakF=%" PRIu32 "\n",
		   round, rank, initiator_msg_decoded, control_msg_decoded, prio_msg_decoded,
		   msgs_not_decoded, msgs_weak, msgs_weak_fake);

	PRINT_HEADER();
	printf("aggregate: %02" PRIx8 " %02" PRIx8 " %02" PRIx8 " (n1=%" PRIu8 " p1=%" PRIu8 " n2=%" PRIu8 " p2=%" PRIu8 ")\n", agg[0], agg[1], agg[2] & 0xF0, GET_NODE1(agg), GET_PRIO1(agg), GET_NODE2(agg), GET_PRIO2(agg));

	// PRINT_HEADER();
	// printf("agg flags = %02" PRIx8 " %02" PRIx8 " %02" PRIx8 "\n", agg[0], agg[1], agg[2] & 0xF0);
	// PRINT_HEADER();
	// printf("node1=%" PRIu8 " prio1=%" PRIu8 " node2=%" PRIu8 " prio2=%" PRIu8 "\n", GET_NODE1(agg), GET_PRIO1(agg), GET_NODE2(agg), GET_PRIO2(agg));

	PRINT_HEADER();
	printf("prios=[");
	for (i = 0; i < (MX_GENERATION_SIZE - 1); i++)
	{
		printf("%" PRIu8 ";", all_priorities[i]);
	}
	printf("]\n");
	memset(all_priorities, 0, sizeof(all_priorities));

	PRINT_HEADER();
	printf("curTrigger=[");
	for (i = 0; i < (MX_GENERATION_SIZE - 1); i++)
	{
		printf("%" PRIu8 ";", all_currTrigger[i]);
	}
	printf("]\n");
	memset(all_currTrigger, 0, sizeof(all_currTrigger));

	PRINT_HEADER();
	printf("rank=[");
	for (i = 0; i < (MX_GENERATION_SIZE - 1); i++)
	{
		printf("%" PRIu8 ";", all_ranks[i]);
	}
	printf("]\n");
	memset(all_ranks, 0, sizeof(all_ranks));

	PRINT_HEADER();
	printf("full_rank=[");
	for (i = 0; i < (MX_GENERATION_SIZE - 1); i++)
	{
		printf("%" PRIu8 ";", all_slot_full_rank[i]);
	}
	printf("]\n");
	memset(all_slot_full_rank, 0, sizeof(all_slot_full_rank));

	PRINT_HEADER();
	printf("radio=[");
	for (i = 0; i < (MX_GENERATION_SIZE - 1); i++)
	{
		printf("%" PRIu32 ";", all_radio_on_time[i]);
	}
	printf("]\n");
	memset(all_radio_on_time, 0, sizeof(all_radio_on_time));

	uint8_t version = all_versions[0];
	for (i = 1; i < (MX_GENERATION_SIZE - 1); i++)
	{
		if (version != all_versions[i])
			printf("version mismatch (i=%u)\n", i);
	}

	// PRINT_HEADER();
	// printf("version=[");
	// for (i = 0; i < (MX_GENERATION_SIZE - 1); i++)
	// {
	// 	printf("%" PRIu8 ";", all_versions[i]);
	// }
	// printf("]\n");





	// PRINT_HEADER();
	// printf("rank_up_slot=[");
	// for (slot_min = 0; 1; )
	// {
	// 	slot = -1u;
	// 	for (i = 0; i < MX_GENERATION_SIZE; ++i)
	// 	{
	// 		if (mixer_stat_slot(i) < slot_min)
	// 			continue;

	// 		if (slot > (uint16_t)mixer_stat_slot(i))
	// 			slot = mixer_stat_slot(i);
	// 	}

	// 	if (-1u == slot)
	// 		break;

	// 	for (i = 0; i < MX_GENERATION_SIZE; ++i)
	// 	{
	// 		if (mixer_stat_slot(i) == slot)
	// 			printf("%u;", slot);
	// 	}

	// 	slot_min = slot + 1;
	// }
	// printf("]\n");

	// PRINT_HEADER();
	// printf("rank_up_row=[");
	// for (slot_min = 0; 1; )
	// {
	// 	slot = -1u;
	// 	for (i = 0; i < MX_GENERATION_SIZE; ++i)
	// 	{
	// 		if (mixer_stat_slot(i) < slot_min)
	// 			continue;

	// 		if (slot > (uint16_t)mixer_stat_slot(i))
	// 			slot = mixer_stat_slot(i);
	// 	}

	// 	if (-1u == slot)
	// 		break;

	// 	for (i = 0; i < MX_GENERATION_SIZE; ++i)
	// 	{
	// 		if (mixer_stat_slot(i) == slot)
	// 			printf("%u;", i);
	// 	}

	// 	slot_min = slot + 1;
	// }
	// printf("]\n");
}

//**************************************************************************************************

static void initialization(void)
{
	// init platform
	gpi_platform_init();
	gpi_int_enable();

	// Start random number generator (RNG) now so that we definitely have some random value as a seed later in the initialization.
	NRF_RNG->INTENCLR = BV_BY_NAME(RNG_INTENCLR_VALRDY, Clear);
	NRF_RNG->CONFIG = BV_BY_NAME(RNG_CONFIG_DERCEN, Enabled);
	NRF_RNG->TASKS_START = 1;

	// enable SysTick timer if needed
	//#if MX_VERBOSE_PROFILE
		SysTick->LOAD  = -1u;
		SysTick->VAL   = 0;
		SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
	//#endif

	// init RF transceiver
	gpi_radio_init(MX_PHY_MODE);
	gpi_radio_set_tx_power(gpi_radio_dbm_to_power_level(MX_TX_PWR_DBM));
	switch (MX_PHY_MODE)
	{
		case BLE_1M:
		case BLE_2M:
		case BLE_125k:
		case BLE_500k:
			gpi_radio_set_channel(39);
			gpi_radio_ble_set_access_address(~0x8E89BED6);
			break;

		case IEEE_802_15_4:
			gpi_radio_set_channel(26);
			break;

		default:
			printf("ERROR: MX_PHY_MODE is invalid!\n");
			assert(0);
	}

	// The Bolt MCU has to be up and running before we init Bolt here.
	gpi_milli_sleep(500);
	if (bolt_init() != 0)
	{
		while (1)
		{
			printf("Bolt cannot be initialized!\n");
			gpi_milli_sleep(500);
		}
	}

	printf("Hardware initialized. Compiled at __DATE__ __TIME__ = " __DATE__ " " __TIME__ "\n");


	/*
	* Pearson hashing (from Wikipedia)
	*
	* Pearson hashing is a hash function designed for fast execution on processors with 8-bit registers.
	* Given an input consisting of any number of bytes, it produces as output a single byte that is strongly
	* dependent on every byte of the input. Its implementation requires only a few instructions, plus a
	* 256-byte lookup table containing a permutation of the values 0 through 255.
	*/

	// T table for Pearson hashing from RFC 3074.
	uint8_t T[256] = {
		251, 175, 119, 215, 81, 14, 79, 191, 103, 49, 181, 143, 186, 157,  0,
		232, 31, 32, 55, 60, 152, 58, 17, 237, 174, 70, 160, 144, 220, 90, 57,
		223, 59,  3, 18, 140, 111, 166, 203, 196, 134, 243, 124, 95, 222, 179,
		197, 65, 180, 48, 36, 15, 107, 46, 233, 130, 165, 30, 123, 161, 209, 23,
		97, 16, 40, 91, 219, 61, 100, 10, 210, 109, 250, 127, 22, 138, 29, 108,
		244, 67, 207,  9, 178, 204, 74, 98, 126, 249, 167, 116, 34, 77, 193,
		200, 121,  5, 20, 113, 71, 35, 128, 13, 182, 94, 25, 226, 227, 199, 75,
		27, 41, 245, 230, 224, 43, 225, 177, 26, 155, 150, 212, 142, 218, 115,
		241, 73, 88, 105, 39, 114, 62, 255, 192, 201, 145, 214, 168, 158, 221,
		148, 154, 122, 12, 84, 82, 163, 44, 139, 228, 236, 205, 242, 217, 11,
		187, 146, 159, 64, 86, 239, 195, 42, 106, 198, 118, 112, 184, 172, 87,
		2, 173, 117, 176, 229, 247, 253, 137, 185, 99, 164, 102, 147, 45, 66,
		231, 52, 141, 211, 194, 206, 246, 238, 56, 110, 78, 248, 63, 240, 189,
		93, 92, 51, 53, 183, 19, 171, 72, 50, 33, 104, 101, 69, 8, 252, 83, 120,
		76, 135, 85, 54, 202, 125, 188, 213, 96, 235, 136, 208, 162, 129, 190,
		132, 156, 38, 47, 1, 7, 254, 24, 4, 216, 131, 89, 21, 28, 133, 37, 153,
		149, 80, 170, 68, 6, 169, 234, 151
	};

	// Pearsong hashing algorithm as described in RFC 3074.
	// -> http://www.apps.ietf.org/rfc/rfc3074.html
	char *key = __TIME__;
	hash = 8; // length of __TIME__ string
	for (uint8_t i = 8; i > 0;) hash = T[hash ^ key[--i]];

	printf("version hash = %" PRIu8 "\n", hash);

	// get TOS_NODE_ID
	// if not set by programming toolchain on testbed
	if (0 == TOS_NODE_ID)
	{
		uint16_t	data[2];

		// read from nRF UICR area
		gpi_nrf_uicr_read(&data, 0, sizeof(data));

		// check signature
		if (0x55AA == data[0])
		{
			GPI_TRACE_MSG(TRACE_INFO, "non-volatile config is valid");
			TOS_NODE_ID = data[1];
		}
		else GPI_TRACE_MSG(TRACE_INFO, "non-volatile config is invalid");

		// if signature is invalid
		while (0 == TOS_NODE_ID)
		{
			printf("TOS_NODE_ID not set. enter value: ");

			// read from console
			// scanf("%u", &TOS_NODE_ID);
			char s[8];
			TOS_NODE_ID = atoi(getsn(s, sizeof(s)));

			printf("\nTOS_NODE_ID set to %u\n", TOS_NODE_ID);

			// until input value is valid
			if (0 == TOS_NODE_ID)
				continue;

			// store new value in UICR area

			data[0] = 0x55AA;
			data[1] = TOS_NODE_ID;

			gpi_nrf_uicr_erase();
			gpi_nrf_uicr_write(0, &data, sizeof(data));

			// ATTENTION: Writing to UICR requires NVMC->CONFIG.WEN to be set which in turn
			// invalidates the instruction cache (permanently). Besides that, UICR updates take
			// effect only after reset (spec. 4413_417 v1.0 4.3.3 page 24). Therefore we do a soft
			// reset after the write procedure.
			printf("Restarting system...\n");
			gpi_milli_sleep(100);		// safety margin (e.g. to empty UART Tx FIFO)
			NVIC_SystemReset();

			break;
		}
	}

	printf("starting node %u ...\n", TOS_NODE_ID);

	// Stop RNG because we only need one random number as seed.
	NRF_RNG->TASKS_STOP = 1;
	uint8_t rng_value = BV_BY_VALUE(RNG_VALUE_VALUE, NRF_RNG->VALUE);
	uint32_t rng_seed = rng_value * gpi_mulu_16x16(TOS_NODE_ID, gpi_tick_fast_native());
	printf("random seed for Mixer is %" PRIu32"\n", rng_seed);
	// init RNG with randomized seed
	mixer_rand_seed(rng_seed);

	// translate TOS_NODE_ID to logical node id used with mixer
	for (node_id = 0; node_id < NUM_ELEMENTS(nodes); ++node_id)
	{
		if (nodes[node_id] == TOS_NODE_ID)
			break;
	}
	if (node_id >= NUM_ELEMENTS(nodes))
	{
		printf("!!! PANIC: node mapping not found for node %u !!!\n", TOS_NODE_ID);
		while (1);
	}
	printf("mapped physical node %u to logical id %u\n", TOS_NODE_ID, node_id);


	Gpi_Hybrid_Tick app_time = ROUND_PERIOD - MIXER_INITIATOR_DELAY - MIXER_DURATION -
							   MIXER_DEADLINE_BUFFER - BOLT_WRITE_DURATION - MIXER_ARM_DURATION -
							   BOLT_READ_DURATION +
							   MX_SLOT_LENGTH; // TODO: unclear (refers to MIXER_OFFSET)
	printf("AP has a time window of %" PRIu32 " us for calculation and writing to Bolt.\n", gpi_tick_hybrid_to_us(app_time));

	mixer_print_config();

	// We send the node ID of the CP to the AP for easier deployment (no need to specify AP node ID).
	bolt_pkt_t init_pkt = {.type = BOLT_INIT, .init.phyNodeID = TOS_NODE_ID, .init.modeID = DEFAULT_MODE};
	bolt_write(&init_pkt, LEN_BOLT_INIT);



}

//**************************************************************************************************
//***** Global Functions ***************************************************************************

int main()
{
	// don't TRACE before gpi_platform_init()
	// GPI_TRACE_FUNCTION();

	Gpi_Hybrid_Tick	t_ref; // time reference point at the end of a Mixer round
	unsigned int i;
	unsigned int plant_idx	= -1u;

	bolt_pkt_t	bolt_control_pkt =
		{.type					= BOLT_CONTROL,
		 .control.curSchedID	= DEFAULT_MODE,
		 .control.newSchedID	= DEFAULT_MODE,
		 .control.newSchedRnds	= 0,
		 .control.seqNum		= 0};

	bolt_pkt_t	bolt_stats_pkt =
		{.type					= BOLT_PRINT};


	initialization();

	for (i = 0; i < NUM_PLANTS; i++)
	{
		if (plants[i] == TOS_NODE_ID)
		{
			plant_idx	= i;
			break;
		}
	}

	CLR_COM_GPIO1();
	CLR_COM_GPIO2();

	// t_ref for first round is now (-> start as soon as possible)
	t_ref = gpi_tick_hybrid();

	// run
	for (round = 1; 1; round++)
	{
		// init mixer
		mixer_init(node_id);
		mixer_set_weak_release_slot(WEAK_RELEASE_SLOT);
		mixer_set_weak_return_msg((void*)-1);

		// init aggregate callback
		mixer_init_agg(&agg_rx_cb);
		// reset aggregate
		memset(agg_input, 0, AGGREGATE_SIZE);

		// Initiator sends control packet
		if (MX_INITIATOR_ID == TOS_NODE_ID)
		{
			bolt_control_pkt.control.seqNum = (uint16_t)round;
			// NOTE: We do not neet to include the type member in the packets exchanged with Mixer
			// because we specified that the control packet uses index 0 and data packets use
			// indexes > 0. Including packet type information is only needed when data is
			// transferred over Bolt.
			mixer_write(0, &bolt_control_pkt.control, sizeof(control_pkt_t));
		}


		// First we check if all nodes participated in the aggregate.
		// NOTE: In the rare case that a node did not receive all probabilities (at least one
		// 0 in agg array), it is impossible to ensure that all nodes will pick the same
		// CONTROL_MSG_LIMIT number of nodes in the same order. The node locally decides to send
		// nothing in order to break nothing.
		unsigned int unique_order = all_flags_set(agg);

		// Determine which node uses which Mixer index. The initiator uses index 0.
		unsigned int send_idx = -1u;
        
		if (unique_order)
		{

			if (TOS_NODE_ID == GET_NODE1(agg))
			{
				send_idx = 1;
			}
			else if (TOS_NODE_ID == GET_NODE2(agg))
			{
				send_idx = 2;
			}
			#if PLANT_STATE_LOGGING
				else
				{
					send_idx = TOS_NODE_ID;
					if (TOS_NODE_ID < GET_NODE1(agg))
						send_idx++;
					if (TOS_NODE_ID < GET_NODE2(agg))
						send_idx++;
				}
			#endif
		}


		SET_COM_GPIO1();
		// wait before reading from Bolt to give application enough time for computation
		while (gpi_tick_compare_hybrid(gpi_tick_hybrid(), READ_AND_ARM_OFFSET(t_ref, ROUND_PERIOD)) < 0);
		CLR_COM_GPIO1();

		PROFILE_MAIN("read and arm start");

		if (plant_idx != -1u)
		{
			// Read packet from Bolt.
			bolt_pkt_t AP_pkt;
			unsigned int AP_pkt_error = -1u;
			if (BOLT_DATA_AVAILABLE)
			{
				PROFILE_MAIN("bolt read start");

				if (bolt_read(&AP_pkt) == 0)
					AP_pkt_error = 1;
				else
					AP_pkt_error = 0;

				if (AP_pkt.type != BOLT_DATA_PROB) AP_pkt_error = 2;

				PROFILE_MAIN("bolt read end");
			}

			// Write current probability into Mixer's aggregate field.
			// NOTE: We assume each node only sends one message and the initiator at index 0
			// does not have a pendulum!
			// NOTE: To distinguish between "lowest probability" and "probability not received", we
			// use 1 as lowest possible p and 0 as default value.
			uint8_t prob = 1;
			uint8_t trigger = 1;
			if (AP_pkt_error == 0)
			{
				prob = AP_pkt.data_prob.prob_info.prob;
				trigger = AP_pkt.data_prob.prob_info.currTrigger;
			}

			#if SIMULATE_MESSAGES
			{
				unsigned int k = 0;
				if (TOS_NODE_ID == 1)
				{
					for (k = 1; k < 11; k++)
					{
						SET_AGG_FLAG_NODE(agg_input, k);
					}
				}
				else if (TOS_NODE_ID == 2)
				{
					for (k = 11; k < 21; k++)
					{
						SET_AGG_FLAG_NODE(agg_input, k);
					}
				}
			}
			#else
				SET_AGG_FLAG_NODE(agg_input, TOS_NODE_ID);
			#endif

			SET_NODE1(agg_input, TOS_NODE_ID);
			SET_PRIO1(agg_input, prob);
			mixer_write_agg(agg_input);


			PROFILE_MAIN("mixer write start");

			#if !SIMULATE_MESSAGES
				// Check if the local node is allowed to send.
                                if (send_idx != -1u)
				{
					// Write data if AP packet was ok, otherwise write weak zero.

                                        if (AP_pkt_error == 0)
					{
						#if !PLANT_STATE_LOGGING
							// Check second (instantaneous) trigger.
							if (AP_pkt.data_prob.prob_info.currTrigger == 1)
							{
								mixer_write(send_idx, NULL, 1);
							}
							else
						#endif
						{
							AP_pkt.enh_data.rank = rank;
							AP_pkt.enh_data.version = hash;
							AP_pkt.enh_data.priority = prob;
							AP_pkt.enh_data.currTrigger = trigger;
							AP_pkt.enh_data.slot_full_rank = slot_full_rank;
							AP_pkt.enh_data.radio_on_time = radio_on_time;
							mixer_write(send_idx, &AP_pkt.enh_data, sizeof(enhanced_data_pkt_t));
						}
					}
					else
					{
						mixer_write(send_idx, NULL, 1);
					}
				}
			#else
				unsigned int k = 0;
				if (TOS_NODE_ID == 1)
				{
					for (k = 1; k < 11; k++)
					{
						#if !PLANT_STATE_LOGGING
							// Check second (instantaneous) trigger.
							if (AP_pkt.data_prob.prob_info.currTrigger == 1)
							{
								mixer_write(k, NULL, 1);
							}
							else
						#endif
						{
							AP_pkt.enh_data.senderID = k;
							AP_pkt.enh_data.rank = rank;
							AP_pkt.enh_data.version = hash;
							AP_pkt.enh_data.priority = prob;
							AP_pkt.enh_data.currTrigger = trigger;
							AP_pkt.enh_data.slot_full_rank = slot_full_rank;
							AP_pkt.enh_data.radio_on_time = radio_on_time;
							mixer_write(k, &AP_pkt.enh_data, sizeof(enhanced_data_pkt_t));
						}
					}
				}
				else if (TOS_NODE_ID == 2)
				{
					for (k = 11; k < 21; k++)
					{
						#if !PLANT_STATE_LOGGING
							// Check second (instantaneous) trigger.
							if (AP_pkt.data_prob.prob_info.currTrigger == 1)
							{
								mixer_write(k, NULL, 1);
							}
							else
						#endif
						{
							AP_pkt.enh_data.senderID = k;
							AP_pkt.enh_data.rank = rank;
							AP_pkt.enh_data.version = hash;
							AP_pkt.enh_data.priority = prob;
							AP_pkt.enh_data.currTrigger = trigger;
							AP_pkt.enh_data.slot_full_rank = slot_full_rank;
							AP_pkt.enh_data.radio_on_time = radio_on_time;
							mixer_write(k, &AP_pkt.enh_data, sizeof(enhanced_data_pkt_t));
						}
					}
				}
			#endif


			PROFILE_MAIN("mixer write end");
		}


		// arm mixer
		// start first round with infinite scan
		// -> nodes join next available round, does not require simultaneous boot-up
		mixer_arm(((MX_INITIATOR_ID == TOS_NODE_ID) ? MX_ARM_INITIATOR : 0) | ((1 == round) ? MX_ARM_INFINITE_SCAN : 0));

		PROFILE_MAIN("read and arm end");

		// delay initiator a bit
		// -> increase probability that all nodes are ready when initiator starts the round
		// -> avoid problems in view of limited t_ref accuracy
		SET_COM_GPIO1();
		if (MX_INITIATOR_ID == TOS_NODE_ID)
		{
			while (gpi_tick_compare_hybrid(gpi_tick_hybrid(), MIXER_OFFSET(t_ref, ROUND_PERIOD) + MIXER_INITIATOR_DELAY) < 0);
		}
		else
		{
			while (gpi_tick_compare_hybrid(gpi_tick_hybrid(), MIXER_OFFSET(t_ref, ROUND_PERIOD)) < 0);
		}
		CLR_COM_GPIO1();

		// ATTENTION: don't delay after the polling loop (-> print before)
		t_ref = mixer_start();

		PROFILE_MAIN("after mixer start");

		// If the timings are correct, flushing after the Mixer round is safe because new messages
		// should be available via Bolt earliest after the sync line event.
		SET_COM_GPIO1();
		bolt_flush();
		CLR_COM_GPIO1();

		PROFILE_MAIN("bolt flush end");

		initiator_msg_decoded = 0;
		control_msg_decoded = 0;
		prio_msg_decoded = 0;
		msgs_not_decoded = 0;
		msgs_weak = 0;
		msgs_weak_fake = 0;
		rank = 0;
		slot_full_rank = 0;
		radio_on_time = 0;

		// read received data.
		for (i = 0; i < MX_GENERATION_SIZE; i++)
		{
			PROFILE_MAIN("mixer read start");

			void *p = mixer_read(i);

			PROFILE_MAIN("mixer read end");

			if (NULL == p)
			{
				++msgs_not_decoded;
			}
			else if ((void*)-1 == p)
			{
				++msgs_weak;
			}
			else
			{
				if (i == 0)
				{
					PROFILE_MAIN("memcpy and bolt write start");

					initiator_msg_decoded = 1;

					// send current control packet to AP
					memcpy(&bolt_control_pkt.control, p, sizeof(control_pkt_t));
					bolt_write(&bolt_control_pkt, LEN_BOLT_CONTROL);

					PROFILE_MAIN("memcpy and bolt write end");

					// synchronize to the initiator node
					if (1 == round)
					{
						round = bolt_control_pkt.control.seqNum;
					}
					// resynchronize when round number does not match
					else if (bolt_control_pkt.control.seqNum != round)
					{
						round = 0;	// increments to 1 with next round loop iteration
					}
				}
				else if ((i < (CONTROL_MSG_LIMIT + 1)) && unique_order) // +1 because initiator control msg at index 0
				{
					#if PLANT_STATE_LOGGING
						// check second trigger and if agent sent priority 1 in the round before (so it would not have been scheduled)
						if ((((enhanced_data_pkt_t*)p)->currTrigger == 1)
							|| ((i == 1) && (GET_PRIO1(agg) == 1))
							|| ((i == 2) && (GET_PRIO2(agg) == 1)))
						{
							++msgs_weak_fake;
							bolt_pkt_t CP_pkt  = {.type = BOLT_DATA_FAKE};
							memcpy(&CP_pkt.data, p, sizeof(data_pkt_t));
							bolt_write(&CP_pkt, LEN_BOLT_DATA_FAKE);
						}
						else
					#endif
					{
						control_msg_decoded++;
						bolt_pkt_t CP_pkt  = {.type = BOLT_DATA};
						memcpy(&CP_pkt.data, p, sizeof(data_pkt_t));
						bolt_write(&CP_pkt, LEN_BOLT_DATA);
					}

					uint8_t index = ((enhanced_data_pkt_t*)p)->senderID;
					all_ranks[index-1] = ((enhanced_data_pkt_t*)p)->rank;
					all_versions[index-1] = ((enhanced_data_pkt_t*)p)->version;
					all_priorities[index-1] = ((enhanced_data_pkt_t*)p)->priority;
					all_currTrigger[index-1] = ((enhanced_data_pkt_t*)p)->currTrigger;
					all_slot_full_rank[index-1] = ((enhanced_data_pkt_t*)p)->slot_full_rank;
					all_radio_on_time[index-1] = ((enhanced_data_pkt_t*)p)->radio_on_time;
				}
				else
				{
					prio_msg_decoded++;
					bolt_pkt_t CP_pkt  = {.type = BOLT_DATA_FAKE};
					memcpy(&CP_pkt.data, p, sizeof(data_pkt_t));
					bolt_write(&CP_pkt, LEN_BOLT_DATA_FAKE);

					uint8_t index = ((enhanced_data_pkt_t*)p)->senderID;
					all_ranks[index-1] = ((enhanced_data_pkt_t*)p)->rank;
					all_versions[index-1] = ((enhanced_data_pkt_t*)p)->version;
					all_priorities[index-1] = ((enhanced_data_pkt_t*)p)->priority;
					all_currTrigger[index-1] = ((enhanced_data_pkt_t*)p)->currTrigger;
					all_slot_full_rank[index-1] = ((enhanced_data_pkt_t*)p)->slot_full_rank;
					all_radio_on_time[index-1] = ((enhanced_data_pkt_t*)p)->radio_on_time;
				}
			}
		}

		// process aggregate data
		memcpy(agg, mixer_read_agg(), AGGREGATE_SIZE);
		// bolt_write(&bolt_agg_pkt, LEN_BOLT_AGG_DATA);

		PROFILE_MAIN("after mixer end");

		SET_COM_GPIO1();
		while (gpi_tick_compare_hybrid(gpi_tick_hybrid(), SYNC_LINE_OFFSET(t_ref)) < 0);
		CLR_COM_GPIO1();

		SET_COM_GPIO2();

		// We toggle the sync line to signal the AP. We delay the pull down to
		// prevent the AP from missing the event, because the CP runs faster
		// than the AP (64 vs. 48 MHz).
		NRF_P0->OUTSET = BV(2);
		gpi_micro_sleep(1);
		NRF_P0->OUTCLR = BV(2);

		CLR_COM_GPIO2();

		PROFILE_MAIN("print start");

		SET_COM_GPIO1();
		print_results(node_id);
		CLR_COM_GPIO1();

		PROFILE_MAIN("print end");
	}

	GPI_TRACE_RETURN(0);
}

//**************************************************************************************************
//**************************************************************************************************
