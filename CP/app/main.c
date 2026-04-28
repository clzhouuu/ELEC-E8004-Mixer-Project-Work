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

//**************************************************************************************************
//***** Local Defines and Consts *******************************************************************

//**************************************************************************************************
//***** Local Typedefs and Class Declarations ******************************************************

//**************************************************************************************************
//***** Forward Declarations ***********************************************************************

//**************************************************************************************************
//***** Local (Static) Variables *******************************************************************

static uint8_t  node_id;    
static uint32_t round = 0;

static pose_pkt_t received_poses[MX_NUM_NODES];
static uint8_t    pose_received[MX_NUM_NODES]; 



//**************************************************************************************************
//***** Global Variables ***************************************************************************
uint16_t __attribute__((section(".data"))) TOS_NODE_ID = 0;

//**************************************************************************************************
//***** Local Functions ****************************************************************************

//**************************************************************************************************

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

static void initialization(void)
{
    gpi_platform_init();
    gpi_int_enable();
 
    // SysTick
    SysTick->LOAD = -1u;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
 
    // Radio
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
            printf("ERROR: invalid MX_PHY_MODE\n");
            while (1);
    }
 
    // Wait for BOLT MCU to be ready
    gpi_milli_sleep(500);
    if (bolt_init() != 0)
    {
        while (1)
        {
            printf("Bolt init failed!\n");
            gpi_milli_sleep(500);
        }
    }
    printf("Hardware initialized.\n");
 
    // Get TOS_NODE_ID
    if (0 == TOS_NODE_ID)
    {
        uint16_t data[2];
        gpi_nrf_uicr_read(&data, 0, sizeof(data));
        if (0x55AA == data[0])
            TOS_NODE_ID = data[1];
 
        while (0 == TOS_NODE_ID)
        {
            printf("TOS_NODE_ID not set. Enter value (1, 2, or 3): ");
            char s[8];
            TOS_NODE_ID = atoi(getsn(s, sizeof(s)));
            printf("\nTOS_NODE_ID = %u\n", TOS_NODE_ID);
            if (0 == TOS_NODE_ID) continue;
 
            data[0] = 0x55AA;
            data[1] = TOS_NODE_ID;
            gpi_nrf_uicr_erase();
            gpi_nrf_uicr_write(0, &data, sizeof(data));
            printf("Restarting...\n");
            gpi_milli_sleep(100);
            NVIC_SystemReset();
        }
    }
    printf("Node ID: %u\n", TOS_NODE_ID);
 
    // Map physical node ID to logical Mixer ID
    for (node_id = 0; node_id < NUM_ELEMENTS(nodes); node_id++)
    {
        if (nodes[node_id] == TOS_NODE_ID) break;
    }
    if (node_id >= NUM_ELEMENTS(nodes))
    {
        printf("PANIC: node %u not in nodes[]\n", TOS_NODE_ID);
        while (1);
    }
    printf("Logical node id: %u\n", node_id);
 
    // Seed Mixer RNG
    NRF_RNG->INTENCLR = BV_BY_NAME(RNG_INTENCLR_VALRDY, Clear);
    NRF_RNG->CONFIG   = BV_BY_NAME(RNG_CONFIG_DERCEN, Enabled);
    NRF_RNG->TASKS_START = 1;
    gpi_milli_sleep(10);
    uint32_t rng_seed = NRF_RNG->VALUE * gpi_mulu_16x16(TOS_NODE_ID, gpi_tick_fast_native());
    NRF_RNG->TASKS_STOP = 1;
    mixer_rand_seed(rng_seed);
 
    mixer_print_config();
}


//**************************************************************************************************
//***** Global Functions ***************************************************************************

int main(void)
{
    initialization();
 
    Gpi_Hybrid_Tick t_ref = gpi_tick_hybrid();
 
    for (round = 1; ; round++)
    {
        // initialize mixer        
        mixer_init(node_id);
        mixer_set_weak_release_slot(MX_ROUND_LENGTH / 2);
        mixer_set_weak_return_msg((void*)-1);
 
        // initiator writes sync packet to slot 0
        if (MX_INITIATOR_ID == TOS_NODE_ID)
        {
            bolt_pkt_t sync_pkt;
            memset(&sync_pkt, 0, sizeof(sync_pkt));
            sync_pkt.type = BOLT_SYNC;
            sync_pkt.sync.round = (uint16_t)round;
            mixer_write(0, &sync_pkt.sync, sizeof(sync_pkt_t));
        }
 
        // bolt to mixer read 
        while (gpi_tick_compare_hybrid(gpi_tick_hybrid(), READ_AND_ARM_OFFSET(t_ref, ROUND_PERIOD)) < 0);
 
        // pose from BOLT 
        if (BOLT_DATA_AVAILABLE)
        {
            bolt_pkt_t ap_pkt;
            uint16_t len = bolt_read(&ap_pkt);

        printf("# ID:%u BOLT len=%u type=%u expected=%u\r\n",
            TOS_NODE_ID,
            (unsigned)len,
            (unsigned)ap_pkt.type,
            (unsigned)BOLT_POSE);
 
            if (len == LEN_BOLT_POSE && ap_pkt.type == BOLT_POSE)
            {
                // write pose into our mixer slot
                mixer_write(TOS_NODE_ID, &ap_pkt.pose, sizeof(pose_pkt_t));
                printf("# ID:%u TX pose x=%ld y=%ld\n",
                    TOS_NODE_ID,
                    (long)ap_pkt.pose.x_fp,
                    (long)ap_pkt.pose.y_fp);
            }
            else
            {
                // no valid pose 
                mixer_write(TOS_NODE_ID, NULL, 1);
                printf("# ID:%u no valid pose from AP\n", TOS_NODE_ID);
            }
        }
        else
        {
            // nothing from AP 
            mixer_write(TOS_NODE_ID, NULL, 1);
            printf("# ID:%u BOLT empty\n", TOS_NODE_ID);
        }
 
        // ---------------------------------------------------------------
        // 5. Arm and start Mixer
        // ---------------------------------------------------------------
        mixer_arm(
            ((MX_INITIATOR_ID == TOS_NODE_ID) ? MX_ARM_INITIATOR : 0) |
            ((1 == round) ? MX_ARM_INFINITE_SCAN : 0)
        );
 
        // Initiator waits a bit before starting so all nodes are ready
        if (MX_INITIATOR_ID == TOS_NODE_ID)
        {
            while (gpi_tick_compare_hybrid(
                gpi_tick_hybrid(),
                MIXER_OFFSET(t_ref, ROUND_PERIOD) + MIXER_INITIATOR_DELAY) < 0);
        }
        else
        {
            while (gpi_tick_compare_hybrid(
                gpi_tick_hybrid(),
                MIXER_OFFSET(t_ref, ROUND_PERIOD)) < 0);
        }
 
        t_ref = mixer_start();
 
        // Flush stale BOLT messages after Mixer round
        bolt_flush();
 
        // all mixer slots
        memset(pose_received, 0, sizeof(pose_received));
        uint8_t msgs_decoded = 0;
        uint8_t msgs_failed  = 0;
        uint8_t i;
 
        for (i = 0; i < MX_GENERATION_SIZE; i++)
        {
            void *p = mixer_read(i);
 
            if (i == 0)
            {
                // slot 0 = sync packet from initiator
                if (p != NULL && p != (void*)-1)
                {
                    sync_pkt_t *s = (sync_pkt_t*)p;
                    if (round == 1)
                        round = s->round;
                    else if (s->round != round)
                        round = s->round - 1; // will increment at loop top
                }
                continue;
            }
 
            // slots 1-3 = pose from node i
            if (p == NULL || p == (void*)-1)
            {
                msgs_failed++;
                continue;
            }
 
            pose_pkt_t *pose = (pose_pkt_t*)p;
            uint8_t idx = pose->robot_id - 1; // 0-indexed
 
            if (idx < MX_NUM_NODES)
            {
                received_poses[idx]  = *pose;
                pose_received[idx]   = 1;
                msgs_decoded++;
            }
        }
 
        printf("# ID:%u round=%lu decoded=%u failed=%u\n",
            TOS_NODE_ID, (unsigned long)round, msgs_decoded, msgs_failed);
 
        // write all received poses back to BOLT for the AP
 
        // wait for sync
        while (gpi_tick_compare_hybrid(
            gpi_tick_hybrid(),
            SYNC_LINE_OFFSET(t_ref)) < 0);
 
        // toggle sync line to signal AP
        NRF_P0->OUTSET = BV(BOLT_CONF_TIMEREQ_PIN);
        gpi_micro_sleep(1);
        NRF_P0->OUTCLR = BV(BOLT_CONF_TIMEREQ_PIN);
 
        //----------------------
        bolt_pkt_t out_pkt;
        memset(&out_pkt, 0, sizeof(out_pkt));

        out_pkt.type = BOLT_POSE;
        out_pkt.pad  = 0;

        out_pkt.pose.robot_id = 2;
        out_pkt.pose.x_fp     = 80609;  // 1.23
        out_pkt.pose.y_fp     = 29491;  // 0.45
        out_pkt.pose.theta_fp = 51118;  // 0.78
        out_pkt.pose.v_fp     = 13107;  // 0.20
        out_pkt.pose.timestamp_ms = 1234;

        uint8_t ok = bolt_write(&out_pkt, LEN_BOLT_POSE);

        printf("\r\n# FAKE CP -> AP sent robot=%u ok=%u\r\n",
               out_pkt.pose.robot_id,
               ok);

        //----------------------

        for (i = 0; i < MX_NUM_NODES; i++)
        {
            if (!pose_received[i]) continue;
            if (received_poses[i].robot_id == TOS_NODE_ID) continue;
 
            bolt_pkt_t out_pkt;
            memset(&out_pkt, 0, sizeof(out_pkt));
            out_pkt.type = BOLT_POSE;
            out_pkt.pose = received_poses[i];
 
            bolt_write(&out_pkt, LEN_BOLT_POSE);
 
            printf("\r\n# ID:%u forwarded pose from robot %u to AP\n",
                TOS_NODE_ID, received_poses[i].robot_id);
        }
    }
 
    GPI_TRACE_RETURN(0);
}


//**************************************************************************************************
//**************************************************************************************************
