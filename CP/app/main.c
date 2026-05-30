/***************************************************************************************************
 ***************************************************************************************************
 *
 *  Copyright (c) 2019, Networked Embedded Systems Lab, TU Dresden
 *  All rights reserved.
 *
 **************************************************************************************************/

//***** Trace Settings *****************************************************************************

#include "gpi/trace.h"

#define TRACE_INFO      GPI_TRACE_MSG_TYPE_INFO

#ifndef GPI_TRACE_BASE_SELECTION
    #define GPI_TRACE_BASE_SELECTION GPI_TRACE_LOG_STANDARD | GPI_TRACE_LOG_PROGRAM_FLOW
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
//***** Local Defines and Consts *******************************************************************

// Slot 0 is reserved for the initiator sync packet.
// Robot pose slots start at index 1: slot = node_id + 1
// This requires MX_GENERATION_SIZE = MX_NUM_NODES + 1.
#define POSE_SLOT(logical_node_id)   ((logical_node_id) + 1)

#define TEST_WINDOW_ROUNDS           5u
#define ROUND_PERIOD_MS_NUM          200u
#define MAX_POSE_AGE_ROUNDS          5u

//**************************************************************************************************
//***** Local Static Variables *********************************************************************

static uint8_t  node_id;
static uint32_t round = 0;

static pose_pkt_t received_poses[MX_NUM_NODES];
static uint8_t    pose_received[MX_NUM_NODES];

static uint16_t g_pose_new_count = 0;
static uint16_t g_pose_rx_hz = 0;
static uint16_t g_last_hz_round = 0;

static uint16_t g_ap_pose_count = 0;
static uint16_t g_ap_pose_lost = 0;
static uint16_t g_ap_last_seq = 0;
static uint8_t  g_ap_have_last_seq = 0;

static uint16_t g_peer_pose_count = 0;
static uint16_t g_peer_pose_lost = 0;
static uint16_t g_peer_last_seq[MX_NUM_NODES];
static uint8_t  g_peer_have_last_seq[MX_NUM_NODES];

static uint16_t g_last_stats_round = 0;

//**************************************************************************************************
//***** Global Variables ***************************************************************************

// TOS_NODE_ID is placed in .data so tos-set-symbol can override it during testbed programming.
uint16_t __attribute__((section(".data"))) TOS_NODE_ID = 0;

//**************************************************************************************************
//***** Local Functions ****************************************************************************

static void print_fp(int32_t v)
{
    if (v < 0)
    {
        printf("-");
        v = -v;
    }

    uint32_t whole = ((uint32_t)v) >> 16;
    uint32_t frac  = (((uint32_t)v) & 0xFFFFu) * 1000u / 65536u;

    printf("%lu.%03lu", (unsigned long)whole, (unsigned long)frac);
}

unsigned int all_flags_set(uint8_t *agg)
{
    int i;

    for (i = 0; i < MX_NUM_NODES; ++i)
    {
        int byte_index = i / 8;
        int bit_index = 7 - (i % 8);

        if ((agg[byte_index] & (1 << bit_index)) == 0)
        {
            return 0;
        }
    }

    return 1;
}

static void agg_rx_cb(
    volatile uint8_t *agg_is_valid,
    uint8_t *agg_local,
    uint8_t *agg_rx
)
{
    (void)agg_is_valid;
    (void)agg_local;
    (void)agg_rx;
}

//**************************************************************************************************

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
        {
            TOS_NODE_ID = data[1];
        }

        while (0 == TOS_NODE_ID)
        {
            printf("TOS_NODE_ID not set. Enter value (1, 2, or 3): ");

            char s[8];

            TOS_NODE_ID = atoi(getsn(s, sizeof(s)));

            printf("\nTOS_NODE_ID = %u\n", TOS_NODE_ID);

            if (0 == TOS_NODE_ID)
            {
                continue;
            }

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
        if (nodes[node_id] == TOS_NODE_ID)
        {
            break;
        }
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

    uint32_t rng_seed =
        NRF_RNG->VALUE * gpi_mulu_16x16(TOS_NODE_ID, gpi_tick_fast_native());

    NRF_RNG->TASKS_STOP = 1;

    mixer_rand_seed(rng_seed);

    mixer_print_config();
}

//**************************************************************************************************
//***** Main ***************************************************************************************

int main(void)
{
    initialization();

    Gpi_Hybrid_Tick t_ref = gpi_tick_hybrid();

    const uint8_t my_slot = POSE_SLOT(node_id);

    printf("# ID:%u logical_node=%u my_slot=%u\r\n",
           TOS_NODE_ID,
           node_id,
           my_slot);

    pose_pkt_t last_pose;
    uint8_t have_last_pose = 0;
    uint8_t pose_age_rounds = 0;

    for (round = 1; ; round++)
    {
        // Initialize Mixer
        mixer_init(node_id);
        mixer_init_agg(&agg_rx_cb);

#if MX_WEAK_ZEROS
        mixer_set_weak_release_slot(MX_ROUND_LENGTH / 2);
        mixer_set_weak_return_msg((void*)-1);
#endif

        // Initiator writes sync packet to slot 0
        if (MX_INITIATOR_ID == TOS_NODE_ID)
        {
            bolt_pkt_t sync_pkt;

            memset(&sync_pkt, 0, sizeof(sync_pkt));

            sync_pkt.type       = BOLT_SYNC;
            sync_pkt.sync.round = (uint16_t)round;

            mixer_write(0, &sync_pkt.sync, sizeof(sync_pkt_t));
        }

        // Wait until it is time to read from Bolt and arm Mixer
        while (
            gpi_tick_compare_hybrid(
                gpi_tick_hybrid(),
                READ_AND_ARM_OFFSET(t_ref, ROUND_PERIOD)
            ) < 0
        );

        uint8_t wrote_pose_this_round = 0;

        // Read pose from Bolt and write it into the correct Mixer slot.
        if (BOLT_DATA_AVAILABLE)
        {
            bolt_pkt_t ap_pkt;
            uint16_t len = bolt_read(&ap_pkt);

            printf("# ID:%u BOLT READ len=%u expected=%u type=%u expected_type=%u\r\n",
                   TOS_NODE_ID,
                   len,
                   (unsigned)LEN_BOLT_POSE,
                   ap_pkt.type,
                   BOLT_POSE);

            if (len == LEN_BOLT_POSE && ap_pkt.type == BOLT_POSE)
            {
                static uint64_t last_t_ns = 0;

                printf("# ID:%u BOLT POSE ACCEPTED seq=%u t_ns_low=%lu x=",
                       TOS_NODE_ID,
                       ap_pkt.pose.seq,
                       (unsigned long)(ap_pkt.pose.t_ns & 0xffffffffu));
                print_fp(ap_pkt.pose.x_fp);
                printf("\r\n");

                // Always write accepted AP pose into this robot's Mixer slot.
                ap_pkt.pose.robot_id = TOS_NODE_ID;
                ap_pkt.pose.cp_tx_round = (uint16_t)round;

                last_pose = ap_pkt.pose;
                have_last_pose = 1;
                pose_age_rounds = 0;

                mixer_write(my_slot, &last_pose, sizeof(pose_pkt_t));
                wrote_pose_this_round = 1;

                // Metrics only: count new AP poses by timestamp/sequence.
                if (ap_pkt.pose.t_ns != last_t_ns)
                {
                    last_t_ns = ap_pkt.pose.t_ns;

                    g_pose_new_count++;
                    g_ap_pose_count++;

                    if (g_ap_have_last_seq)
                    {
                        uint16_t expected = g_ap_last_seq + 1u;

                        if (ap_pkt.pose.seq != expected)
                        {
                            g_ap_pose_lost +=
                                (uint16_t)(ap_pkt.pose.seq - expected);
                        }
                    }

                    g_ap_last_seq = ap_pkt.pose.seq;
                    g_ap_have_last_seq = 1u;
                }

                printf("# ID:%u TX fresh pose slot=%u x=",
                       TOS_NODE_ID,
                       my_slot);
                print_fp(last_pose.x_fp);
                printf(" y=");
                print_fp(last_pose.y_fp);
                printf(" z=");
                print_fp(last_pose.z_fp);
                printf("\n");
            }
            else
            {
                printf("# ID:%u BOLT POSE REJECTED\r\n", TOS_NODE_ID);
            }
        }
        else
        {
            static uint32_t no_bolt_count = 0;

            no_bolt_count++;

            if ((no_bolt_count % 100u) == 0u)
            {
                printf("# ID:%u BOLT_DATA_AVAILABLE = 0\r\n", TOS_NODE_ID);
            }
        }

        // No fresh AP pose this round: reuse cached pose if still young.
        if (!wrote_pose_this_round)
        {
            if (have_last_pose && pose_age_rounds < MAX_POSE_AGE_ROUNDS)
            {
                pose_age_rounds++;

                mixer_write(my_slot, &last_pose, sizeof(pose_pkt_t));

                printf("# ID:%u TX cached pose slot=%u age=%u x=",
                       TOS_NODE_ID,
                       my_slot,
                       pose_age_rounds);
                print_fp(last_pose.x_fp);
                printf(" y=");
                print_fp(last_pose.y_fp);
                printf(" z=");
                print_fp(last_pose.z_fp);
                printf("\n");
            }
            else
            {
                mixer_write(my_slot, NULL, 1);

                printf("# ID:%u no fresh/cached pose\n", TOS_NODE_ID);
            }
        }

        // Arm and start Mixer
        uint8_t arm_flags = 0;

        if (MX_INITIATOR_ID == TOS_NODE_ID)
        {
            arm_flags = MX_ARM_INITIATOR;
        }
        else if (round == 1)
        {
            arm_flags = MX_ARM_INFINITE_SCAN;
        }

        printf("# ID:%u arm_flags=0x%x round=%lu\n",
               TOS_NODE_ID,
               arm_flags,
               (unsigned long)round);

        mixer_arm(arm_flags);

        if (arm_flags & MX_ARM_INFINITE_SCAN)
        {
            // Follower is trying to join/rejoin: start immediately.
        }
        else if (MX_INITIATOR_ID == TOS_NODE_ID)
        {
            while (
                gpi_tick_compare_hybrid(
                    gpi_tick_hybrid(),
                    MIXER_OFFSET(t_ref, ROUND_PERIOD) + MIXER_INITIATOR_DELAY
                ) < 0
            );
        }
        else
        {
            while (
                gpi_tick_compare_hybrid(
                    gpi_tick_hybrid(),
                    MIXER_OFFSET(t_ref, ROUND_PERIOD)
                ) < 0
            );
        }

        printf("# ID:%u before mixer_start round=%lu\n",
               TOS_NODE_ID,
               (unsigned long)round);

        t_ref = mixer_start();

        printf("# ID:%u after mixer_start round=%lu\n",
               TOS_NODE_ID,
               (unsigned long)round);

        // Read all Mixer slots
        memset(pose_received, 0, sizeof(pose_received));

        uint8_t msgs_decoded = 0;
        uint8_t msgs_failed  = 0;
        uint8_t i;

        for (i = 0; i < MX_GENERATION_SIZE; i++)
        {
            void *p = mixer_read(i);

            // Slot 0 = sync packet from initiator
            if (i == 0)
            {
                if (p != NULL && p != (void*)-1)
                {
                    sync_pkt_t *s = (sync_pkt_t*)p;

                    if (round == 1)
                    {
                        round = s->round;

                        printf("# ID:%u synced to round=%lu\n",
                               TOS_NODE_ID,
                               (unsigned long)round);
                    }
                    else if (s->round != round)
                    {
                        printf("# ID:%u round mismatch sync=%u local=%lu -> resync\n",
                               TOS_NODE_ID,
                               s->round,
                               (unsigned long)round);

                        round = 0;
                    }
                }
                else if (MX_INITIATOR_ID != TOS_NODE_ID)
                {
                    printf("# ID:%u missed sync slot -> resync\n", TOS_NODE_ID);

                    round = 0;
                }

                continue;
            }

            // Pose slots: slot 1 -> robot 1, slot 2 -> robot 2, etc.
            if (i >= 1u && i <= MX_NUM_NODES)
            {
                uint8_t robot_index = i - 1u;

                if (p == NULL || p == (void*)-1)
                {
                    msgs_failed++;

                    printf("# ID:%u RX pose slot=%u empty/failed\n",
                           TOS_NODE_ID,
                           i);
                    continue;
                }

                pose_pkt_t *pose = (pose_pkt_t*)p;

                if (pose->robot_id < 1u || pose->robot_id > MX_NUM_NODES)
                {
                    msgs_failed++;

                    printf("# ID:%u RX pose slot=%u bad robot_id=%u\n",
                           TOS_NODE_ID,
                           i,
                           pose->robot_id);
                    continue;
                }

                if (pose->robot_id != TOS_NODE_ID)
                {
                    uint8_t ridx = pose->robot_id - 1u;

                    g_peer_pose_count++;

                    if (g_peer_have_last_seq[ridx])
                    {
                        uint16_t expected = g_peer_last_seq[ridx] + 1u;

                        if (pose->seq != expected)
                        {
                            g_peer_pose_lost +=
                                (uint16_t)(pose->seq - expected);
                        }
                    }

                    g_peer_last_seq[ridx] = pose->seq;
                    g_peer_have_last_seq[ridx] = 1u;
                }

                received_poses[robot_index] = *pose;
                pose_received[robot_index]  = 1;
                msgs_decoded++;

                printf("# ID:%u RX pose slot=%u robot_id=%u x=",
                       TOS_NODE_ID,
                       i,
                       pose->robot_id);
                print_fp(pose->x_fp);
                printf(" y=");
                print_fp(pose->y_fp);
                printf(" z=");
                print_fp(pose->z_fp);
                printf("\n");

                continue;
            }
        }

        printf("# ID:%u round=%lu decoded=%u failed=%u\n",
               TOS_NODE_ID,
               (unsigned long)round,
               msgs_decoded,
               msgs_failed);

        // Wait for sync-line timing before writing back to Bolt.
        while (
            gpi_tick_compare_hybrid(
                gpi_tick_hybrid(),
                SYNC_LINE_OFFSET(t_ref)
            ) < 0
        );

        // Forward received poses from other robots back to AP via Bolt.
        for (i = 0; i < MX_NUM_NODES; i++)
        {
            if (!pose_received[i])
            {
                printf("# ID:%u no pose for robot %u this round\n",
                       TOS_NODE_ID,
                       i + 1u);
                continue;
            }

            pose_pkt_t *pose = &received_poses[i];

            // Do not echo the local node's own pose back to its AP.
            if (pose->robot_id == TOS_NODE_ID)
            {
                continue;
            }

            bolt_pkt_t out_pkt;

            memset(&out_pkt, 0, sizeof(out_pkt));

            out_pkt.type = BOLT_POSE;
            out_pkt.pose = *pose;

            bolt_write((uint8_t*)&out_pkt, LEN_BOLT_POSE);

            printf("\r\n# ID:%u forwarded pose from robot %u to AP\n",
                   TOS_NODE_ID,
                   pose->robot_id);
        }

        if ((uint16_t)(round - g_last_hz_round) >= 5u)
        {
            g_last_hz_round = (uint16_t)round;

            g_pose_rx_hz = g_pose_new_count;
            g_pose_new_count = 0;

            printf("\r\n# ID:%u AP->CP NEW POSE RX RATE = %u Hz\r\n",
                   TOS_NODE_ID,
                   g_pose_rx_hz);
        }

        if ((uint16_t)(round - g_last_stats_round) >= TEST_WINDOW_ROUNDS)
        {
            uint16_t ap_total;
            uint16_t peer_total;

            g_last_stats_round = (uint16_t)round;

            ap_total = g_ap_pose_count + g_ap_pose_lost;
            peer_total = g_peer_pose_count + g_peer_pose_lost;

            printf("\r\n# STATS ID:%u AP_CP_rx=%u AP_CP_lost=%u",
                   TOS_NODE_ID,
                   g_ap_pose_count,
                   g_ap_pose_lost);

            if (ap_total > 0)
            {
                printf(" AP_CP_loss_pct=%u",
                       (unsigned)((100u * g_ap_pose_lost) / ap_total));
            }

            printf(" MIXER_peer_rx=%u MIXER_peer_lost=%u",
                   g_peer_pose_count,
                   g_peer_pose_lost);

            if (peer_total > 0)
            {
                printf(" MIXER_loss_pct=%u",
                       (unsigned)((100u * g_peer_pose_lost) / peer_total));
            }

            printf("\r\n");

            g_ap_pose_count = 0;
            g_ap_pose_lost = 0;
            g_peer_pose_count = 0;
            g_peer_pose_lost = 0;
        }
    }

    GPI_TRACE_RETURN(0);
}

//**************************************************************************************************
//**************************************************************************************************
