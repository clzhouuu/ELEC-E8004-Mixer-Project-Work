/**
 * main.c  –  nRF52840 Communication Processor (CP)
 * =================================================
 * Responsibilities
 *   1. Run Mixer (Glossy flood) to exchange robot poses at MX_ROUND_LENGTH
 *      slots per round (ROUND_PERIOD ms).
 *   2. Read own robot's pose from the AP (MSP432) via BOLT before each round.
 *      Cache the last valid pose for up to MAX_POSE_AGE_ROUNDS rounds so that
 *      a temporarily-quiet AP does not create a silent Mixer slot.
 *   3. After each round, forward every decoded peer pose back to the AP via
 *      BOLT (own pose is NOT echoed).
 *   4. Maintain round synchronisation: initiator drives the round counter;
 *      followers re-scan on mismatch or missed sync slot.
 *
 * Slot layout (MX_GENERATION_SIZE = MX_NUM_NODES + 1):
 *   Slot 0              – sync packet from initiator
 *   Slot n (1..N_NODES) – pose for robot with logical node_id = n-1
 *
 * Physical ↔ logical mapping is defined in mixer_config.h:
 *   static const uint8_t nodes[] = { 1, 2, 3 };
 *   nodes[logical_id] = physical TOS_NODE_ID
 */

#include "gpi/trace.h"
#define TRACE_INFO GPI_TRACE_MSG_TYPE_INFO
#ifndef GPI_TRACE_BASE_SELECTION
    #define GPI_TRACE_BASE_SELECTION (GPI_TRACE_LOG_STANDARD | GPI_TRACE_LOG_PROGRAM_FLOW)
#endif
GPI_TRACE_CONFIG(main, GPI_TRACE_BASE_SELECTION);

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

/* ── Configuration ─────────────────────────────────────────────────────────── */

/**
 * How many consecutive Mixer rounds a cached pose may be reused when the AP
 * does not provide a fresh one.  After this the slot is sent as NULL.
 */
#define MAX_POSE_AGE_ROUNDS     5u

/** Slot index for a given logical node_id (0-based). */
#define POSE_SLOT(logical_id)   ((logical_id) + 1u)

/* ── TOS_NODE_ID ────────────────────────────────────────────────────────────── */
/* Placed in .data so tos-set-symbol can override it at testbed programming time. */
uint16_t __attribute__((section(".data"))) TOS_NODE_ID = 0;

/* ── Module state ───────────────────────────────────────────────────────────── */

static uint8_t  node_id = 0;          /* logical (0-based) Mixer ID           */
static uint32_t round   = 0;          /* current round counter                */

static pose_pkt_t received_poses[MX_NUM_NODES];
static uint8_t    pose_received [MX_NUM_NODES];

/* ── Helpers ────────────────────────────────────────────────────────────────── */

static void print_fp(int32_t v)
{
    if (v < 0) { printf("-"); v = -v; }
    printf("%lu.%03lu",
           (unsigned long)((uint32_t)v >> 16),
           (unsigned long)((((uint32_t)v) & 0xFFFFu) * 1000u / 65536u));
}

/** Aggregation callback – unused but required by mixer_init_agg. */
static void agg_rx_cb(volatile uint8_t *valid,
                      uint8_t *local,
                      uint8_t *rx)
{
    (void)valid; (void)local; (void)rx;
}

/* ── Initialisation ─────────────────────────────────────────────────────────── */

static void initialization(void)
{
    gpi_platform_init();
    gpi_int_enable();

    /* SysTick – free-running, used only for gpi_tick_hybrid() */
    SysTick->LOAD = ~0u;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Radio */
    gpi_radio_init(MX_PHY_MODE);
    gpi_radio_set_tx_power(gpi_radio_dbm_to_power_level(MX_TX_PWR_DBM));
    switch (MX_PHY_MODE)
    {
        case BLE_1M:
        case BLE_2M:
        case BLE_125k:
        case BLE_500k:
            gpi_radio_set_channel(39);
            gpi_radio_ble_set_access_address(~0x8E89BED6u);
            break;
        case IEEE_802_15_4:
            gpi_radio_set_channel(26);
            break;
        default:
            printf("ERROR: invalid MX_PHY_MODE\n");
            while (1);
    }

    /* Wait for AP (MSP432) to be ready before touching BOLT */
    gpi_milli_sleep(500);
    if (bolt_init() != 0)
    {
        while (1)
        {
            printf("[CP] BOLT init failed – retrying\n");
            gpi_milli_sleep(500);
        }
    }
    printf("[CP] Hardware initialised.\n");

    /* ── Resolve TOS_NODE_ID ── */
    if (TOS_NODE_ID == 0)
    {
        uint16_t data[2];
        gpi_nrf_uicr_read(&data, 0, sizeof(data));
        if (data[0] == 0x55AAu)
            TOS_NODE_ID = data[1];

        while (TOS_NODE_ID == 0)
        {
            printf("[CP] TOS_NODE_ID not set. Enter (1/2/3): ");
            char s[8];
            TOS_NODE_ID = (uint16_t)atoi(getsn(s, sizeof(s)));
            printf("\nTOS_NODE_ID = %u\n", TOS_NODE_ID);
            if (TOS_NODE_ID == 0) continue;

            data[0] = 0x55AAu;
            data[1] = TOS_NODE_ID;
            gpi_nrf_uicr_erase();
            gpi_nrf_uicr_write(0, &data, sizeof(data));
            printf("[CP] Written to UICR – restarting\n");
            gpi_milli_sleep(100);
            NVIC_SystemReset();
        }
    }
    printf("[CP] TOS_NODE_ID=%u\n", TOS_NODE_ID);

    /* ── Map physical → logical node_id ── */
    for (node_id = 0; node_id < NUM_ELEMENTS(nodes); node_id++)
    {
        if (nodes[node_id] == TOS_NODE_ID) break;
    }
    if (node_id >= NUM_ELEMENTS(nodes))
    {
        printf("[CP] PANIC: node %u not in nodes[]\n", TOS_NODE_ID);
        while (1);
    }
    printf("[CP] Logical node_id=%u\n", node_id);

    /* ── Seed Mixer RNG ── */
    NRF_RNG->INTENCLR    = BV_BY_NAME(RNG_INTENCLR_VALRDY, Clear);
    NRF_RNG->CONFIG      = BV_BY_NAME(RNG_CONFIG_DERCEN, Enabled);
    NRF_RNG->TASKS_START = 1;
    gpi_milli_sleep(10);
    uint32_t seed = NRF_RNG->VALUE
                  * gpi_mulu_16x16(TOS_NODE_ID, (uint16_t)gpi_tick_fast_native());
    NRF_RNG->TASKS_STOP = 1;
    mixer_rand_seed(seed);

    mixer_print_config();
}

/* ── read_pose_from_bolt ────────────────────────────────────────────────────── */
/**
 * Attempt to read one pose packet from BOLT (AP → CP direction).
 * Returns 1 and fills *out if a fresh, correctly-typed packet was available;
 * returns 0 otherwise.
 *
 * Also flushes any stale/extra packets so the BOLT queue does not back up.
 */
static uint8_t read_pose_from_bolt(pose_pkt_t *out)
{
    uint8_t got_pose = 0;

    /* Drain all available packets; keep only the most recent valid pose. */
    while (BOLT_DATA_AVAILABLE)
    {
        bolt_pkt_t pkt;
        uint16_t   len = bolt_read(&pkt);

        if (len != LEN_BOLT_POSE)
            continue;

        if (pkt.type != BOLT_POSE)
        {
            printf("[CP] unexpected BOLT type=%u\n", pkt.type);
            continue;
        }

        *out     = pkt.pose;
        got_pose = 1;
        /* Continue loop to flush any further queued packets. */
    }

    return got_pose;
}

/* ── write_poses_to_bolt ────────────────────────────────────────────────────── */
/**
 * Forward all decoded peer poses from this Mixer round back to the AP.
 * Own pose is never echoed (AP already has it).
 */
static void write_poses_to_bolt(void)
{
    uint8_t i;
    for (i = 0; i < MX_NUM_NODES; i++)
    {
        if (!pose_received[i])
        {
            printf("[CP] no pose for robot %u this round\n", i + 1u);
            continue;
        }

        pose_pkt_t *pose = &received_poses[i];

        /* Do not echo own pose back to the AP. */
        if (pose->robot_id == TOS_NODE_ID)
            continue;

        bolt_pkt_t out;
        memset(&out, 0, sizeof(out));
        out.type = BOLT_POSE;
        out.pose = *pose;

        uint8_t ok = bolt_write((uint8_t *)&out, LEN_BOLT_POSE);

        printf("[CP] FWD robot=%u x=", pose->robot_id);
        print_fp(pose->x_fp);
        printf(" y=");
        print_fp(pose->y_fp);
        printf(" %s\n", ok ? "OK" : "FAILED");
    }
}

/* ── main ───────────────────────────────────────────────────────────────────── */

int main(void)
{
    initialization();

    const uint8_t my_slot = POSE_SLOT(node_id);

    /* Own-pose cache */
    pose_pkt_t last_pose;
    uint8_t    have_last_pose   = 0;
    uint8_t    pose_age_rounds  = 0;

    Gpi_Hybrid_Tick t_ref = gpi_tick_hybrid();

    for (round = 1; ; round++)
    {
        /* ── 1. Initialise Mixer for this round ── */
        mixer_init(node_id);
        mixer_init_agg(&agg_rx_cb);

#if MX_WEAK_ZEROS
        mixer_set_weak_release_slot(MX_ROUND_LENGTH / 2);
        mixer_set_weak_return_msg((void *)-1);
#endif

        /* ── 2. Initiator writes sync packet into slot 0 ── */
        if (MX_INITIATOR_ID == TOS_NODE_ID)
        {
            bolt_pkt_t sync;
            memset(&sync, 0, sizeof(sync));
            sync.type       = BOLT_SYNC;
            sync.sync.round = (uint16_t)round;
            mixer_write(0, &sync.sync, sizeof(sync_pkt_t));
        }

        /* ── 3. Wait until READ_AND_ARM window ── */
        while (gpi_tick_compare_hybrid(
                   gpi_tick_hybrid(),
                   READ_AND_ARM_OFFSET(t_ref, ROUND_PERIOD)) < 0);

        /* ── 4. Read pose from AP via BOLT ── */
        uint8_t    wrote_pose = 0;
        pose_pkt_t fresh_pose;

        if (read_pose_from_bolt(&fresh_pose))
        {
            /*
             * Only accept poses whose timestamp has actually advanced.
             * This prevents writing the same pose twice if the AP sends
             * it in two successive BOLT packets.
             */
            static uint32_t last_ts = 0;

            if (fresh_pose.timestamp_ms != last_ts)
            {
                last_ts = fresh_pose.timestamp_ms;

                /* Enforce physical node identity. */
                fresh_pose.robot_id = TOS_NODE_ID;

                last_pose       = fresh_pose;
                have_last_pose  = 1;
                pose_age_rounds = 0;

                mixer_write(my_slot, &last_pose, sizeof(pose_pkt_t));
                wrote_pose = 1;

                printf("[CP] TX fresh pose slot=%u x=", my_slot);
                print_fp(last_pose.x_fp);
                printf(" y=");
                print_fp(last_pose.y_fp);
                printf("\n");
            }
        }

        /* ── 5. No fresh pose – reuse cache or send NULL ── */
        if (!wrote_pose)
        {
            if (have_last_pose && pose_age_rounds < MAX_POSE_AGE_ROUNDS)
            {
                pose_age_rounds++;
                mixer_write(my_slot, &last_pose, sizeof(pose_pkt_t));

                printf("[CP] TX cached pose slot=%u age=%u x=",
                       my_slot, pose_age_rounds);
                print_fp(last_pose.x_fp);
                printf(" y=");
                print_fp(last_pose.y_fp);
                printf("\n");
            }
            else
            {
                mixer_write(my_slot, NULL, 1);
                printf("[CP] no fresh/cached pose for slot %u\n", my_slot);
            }
        }

        /* ── 6. Arm Mixer ── */
        uint8_t arm_flags = 0;

        if (MX_INITIATOR_ID == TOS_NODE_ID)
        {
            arm_flags = MX_ARM_INITIATOR;
        }
        else if (round == 1u)
        {
            /* First round for a follower: scan until we find the initiator. */
            arm_flags = MX_ARM_INFINITE_SCAN;
        }

        printf("[CP] arm_flags=0x%x round=%lu\n",
               arm_flags, (unsigned long)round);

        mixer_arm(arm_flags);

        /* ── 7. Wait for start time ── */
        if (arm_flags & MX_ARM_INFINITE_SCAN)
        {
            /* Start scanning immediately – no timing constraint yet. */
        }
        else if (MX_INITIATOR_ID == TOS_NODE_ID)
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

        /* ── 8. Start Mixer – blocks until round completes ── */
        printf("[CP] mixer_start round=%lu\n", (unsigned long)round);
        t_ref = mixer_start();
        printf("[CP] mixer_done  round=%lu\n", (unsigned long)round);

        /* ── 9. Decode received slots ── */
        memset(pose_received, 0, sizeof(pose_received));
        uint8_t decoded = 0;
        uint8_t failed  = 0;
        uint8_t i;

        for (i = 0; i < MX_GENERATION_SIZE; i++)
        {
            void *p = mixer_read(i);

            /* ── Slot 0: sync packet ── */
            if (i == 0)
            {
                if (p != NULL && p != (void *)-1)
                {
                    sync_pkt_t *s = (sync_pkt_t *)p;

                    if (round == 1u)
                    {
                        /* Follower joining mid-session: adopt initiator round. */
                        round = s->round;
                        printf("[CP] synced to round=%lu\n",
                               (unsigned long)round);
                    }
                    else if (s->round != (uint16_t)round)
                    {
                        printf("[CP] round mismatch: sync=%u local=%lu -> resync\n",
                               s->round, (unsigned long)round);
                        round = 0u;   /* next iteration → round=1 → re-scan */
                    }
                }
                else if (MX_INITIATOR_ID != TOS_NODE_ID)
                {
                    printf("[CP] missed sync slot -> resync\n");
                    round = 0u;
                }
                continue;
            }

            /* ── Slots 1..N: pose packets ── */
            uint8_t robot_index = i - 1u;
            if (robot_index >= MX_NUM_NODES)
                continue;

            if (p == NULL || p == (void *)-1)
            {
                failed++;
                printf("[CP] slot=%u empty/failed\n", i);
                continue;
            }

            pose_pkt_t *pose = (pose_pkt_t *)p;
            received_poses[robot_index] = *pose;
            pose_received [robot_index] = 1;
            decoded++;

            printf("[CP] RX slot=%u robot=%u x=", i, pose->robot_id);
            print_fp(pose->x_fp);
            printf(" y=");
            print_fp(pose->y_fp);
            printf("\n");
        }

        printf("[CP] round=%lu decoded=%u failed=%u\n",
               (unsigned long)round, decoded, failed);

        /* ── 10. Wait for SYNC_LINE timing window before writing to BOLT ── */
        while (gpi_tick_compare_hybrid(
                   gpi_tick_hybrid(),
                   SYNC_LINE_OFFSET(t_ref)) < 0);

        /* ── 11. Forward peer poses to AP via BOLT ── */
        write_poses_to_bolt();
    }

    GPI_TRACE_RETURN(0);
}
