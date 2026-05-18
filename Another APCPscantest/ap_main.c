/**
 * main.c  –  MSP432 Application Processor (AP)
 * =============================================
 * Responsibilities
 *   1. Receive own pose from Raspberry Pi via pi_uart (type 0x02 frame).
 *   2. Broadcast own pose to the Communication Processor (CP / nRF) via BOLT.
 *   3. Receive peer-robot poses from CP via BOLT.
 *   4. Forward peer poses back to the Pi via pi_uart (type 0x03 frame).
 *   5. Mark peer poses stale when no update has arrived for >500 ms.
 *
 * Control loop: CONTROL_PERIOD_MS (50 ms → 20 Hz)
 */

#include <msp432p401r.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>

#include "../include/bolt.h"
#include "../include/message.h"
#include "../include/pi_uart.h"
#include <MSP432P4xx/cs.h>

/* ── Configuration ─────────────────────────────────────────────────────────── */

/** This robot's physical ID (1-based).  Change per board. */
#define OWN_ROBOT_ID        1u

/** SMCLK frequency – must match CS_setDCOCenteredFrequency() call below. */
#define SMCLK_HZ            12000000u

/** Main control loop period in milliseconds (20 Hz). */
#define CONTROL_PERIOD_MS   50u

/**
 * Minimum time between forwarding the same peer's pose to the Pi (ms).
 * Prevents flooding the Pi when BOLT delivers poses faster than needed.
 */
#define PEER_FWD_MIN_MS     50u

/**
 * How long without a BOLT update before a peer is marked stale (ms).
 * Must be > CONTROL_PERIOD_MS to avoid false positives.
 */
#define STALE_TIMEOUT_MS    500u

/* ── System tick ───────────────────────────────────────────────────────────── */

static volatile uint32_t g_tick_ms = 0;

void SysTick_Handler(void)
{
    g_tick_ms++;
}

static void systick_init(void)
{
    SysTick->LOAD = (SMCLK_HZ / 1000u) - 1u;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;
}

static inline uint32_t get_tick(void)
{
    return g_tick_ms;
}

/* ── Minimal debug printf over eUSCI_A0 ────────────────────────────────────── */
/*
 * eUSCI_A0 is used for debug output (USB-UART adapter / CCS console).
 * eUSCI_A1 is used by pi_uart for the Pi link.
 * Both are initialised independently.
 */

int fputc(int c, FILE *f)
{
    (void)f;
    while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A0->TXBUF = (uint8_t)c;
    return c;
}

static void debug_uart_init(void)
{
    /* P1.2 = A0 RX, P1.3 = A0 TX */
    P1->SEL0 |=  (BIT2 | BIT3);
    P1->SEL1 &= ~(BIT2 | BIT3);

    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST;
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_A_CTLW0_SSEL__SMCLK;
    /* 115200 @ 12 MHz: BRW=6, MCTLW = (8<<4)|OS16  (UCBRFx=8, UCBRSx=0) */
    EUSCI_A0->BRW   = 6u;
    EUSCI_A0->MCTLW = (8u << 4) | EUSCI_A_MCTLW_OS16;
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;
}

static void print_str(const char *s)
{
    while (*s) fputc((unsigned char)*s++, stdout);
}

static void print_u32(uint32_t v)
{
    char buf[11];
    int i = 10;
    buf[i] = '\0';
    if (v == 0) { fputc('0', stdout); return; }
    while (v && i > 0) { buf[--i] = '0' + (int)(v % 10); v /= 10; }
    print_str(buf + i);
}

static void print_fp(int32_t v)
{
    if (v < 0) { fputc('-', stdout); v = -v; }
    uint32_t whole = (uint32_t)v >> 16;
    uint32_t frac  = ((uint32_t)v & 0xFFFFu) * 1000u / 65536u;
    print_u32(whole);
    fputc('.', stdout);
    if (frac < 100u) fputc('0', stdout);
    if (frac < 10u)  fputc('0', stdout);
    print_u32(frac);
}

/* ── Global pose state ──────────────────────────────────────────────────────── */

/** Latest known poses for all robots (index = robot_id - 1). */
static RobotPoseMsg_t g_poses[NUM_ROBOTS];

/** Wall-clock time at which we last received a BOLT update for each peer. */
static uint32_t g_pose_rx_time_ms[NUM_ROBOTS];

/** Wall-clock time at which we last forwarded each peer's pose to the Pi. */
static uint32_t g_pose_fwd_time_ms[NUM_ROBOTS];

/* ── receive_from_bolt ──────────────────────────────────────────────────────── */
/**
 * Drain all pending BOLT messages.  For each valid peer pose:
 *   - Update g_poses[] and g_pose_rx_time_ms[].
 *   - Forward the pose to the Pi via pi_uart (rate-limited).
 */
static void receive_from_bolt(uint32_t now_ms)
{
    while (bolt_data_available())
    {
        uint8_t buf[BOLT_MAX_PAYLOAD];
        uint8_t len = 0;

        if (!bolt_read(buf, &len))
        {
            print_str("[AP] BOLT read failed\r\n");
            continue;
        }

        if (len != (uint8_t)LEN_BOLT_POSE)
        {
            /* Ignore non-pose packets (e.g. stale or sync frames). */
            continue;
        }

        bolt_pkt_t *pkt = (bolt_pkt_t *)buf;

        if (pkt->type != BOLT_POSE)
        {
            print_str("[AP] BOLT unexpected type=");
            print_u32(pkt->type);
            print_str("\r\n");
            continue;
        }

        uint8_t id = pkt->pose.robot_id;

        if (id < 1u || id > (uint8_t)NUM_ROBOTS)
        {
            print_str("[AP] BOLT invalid robot_id=");
            print_u32(id);
            print_str("\r\n");
            continue;
        }

        /* Ignore echoes of our own pose. */
        if (id == OWN_ROBOT_ID)
            continue;

        uint8_t idx = id - 1u;

        /* Update receive timestamp and pose. */
        g_pose_rx_time_ms[idx] = now_ms;

        RobotPoseMsg_t *p = &g_poses[idx];
        p->robot_id     = id;
        p->x_fp         = pkt->pose.x_fp;
        p->y_fp         = pkt->pose.y_fp;
        p->theta_fp     = pkt->pose.theta_fp;
        p->v_fp         = pkt->pose.v_fp;
        p->timestamp_ms = pkt->pose.timestamp_ms;
        p->status       = POSE_STATUS_VALID | POSE_STATUS_INITIALISED;

        print_str("[AP] BOLT RX robot=");
        print_u32(id);
        print_str(" x="); print_fp(p->x_fp);
        print_str(" y="); print_fp(p->y_fp);
        print_str(" t="); print_u32(p->timestamp_ms);
        print_str(" ms\r\n");

        /*
         * Forward peer pose to Pi.
         * Rate-limit per peer to avoid saturating the UART when BOLT
         * delivers bursts faster than PEER_FWD_MIN_MS.
         */
        if ((now_ms - g_pose_fwd_time_ms[idx]) >= PEER_FWD_MIN_MS)
        {
            g_pose_fwd_time_ms[idx] = now_ms;
            pi_uart_send_pose(p);
        }
    }
}

/* ── send_own_pose ──────────────────────────────────────────────────────────── */
/**
 * Read the latest own pose from the Pi, stamp it, store it locally, and
 * broadcast it to the CP via BOLT.
 *
 * Returns without transmitting if the Pi has not sent a fresh pose since
 * the last call.
 */
static void send_own_pose(uint32_t now_ms)
{
    RobotPoseMsg_t pose;

    if (!pi_uart_get_pose(&pose))
    {
        /*
         * No fresh pose from the Pi this cycle.
         * The CP handles pose-age caching on its end (MAX_POSE_AGE_ROUNDS),
         * so we simply skip rather than re-sending stale data.
         */
        return;
    }

    /* Enforce our own robot_id regardless of what the Pi sent. */
    pose.robot_id     = OWN_ROBOT_ID;
    pose.timestamp_ms = now_ms;
    pose.status       = POSE_STATUS_VALID | POSE_STATUS_INITIALISED;

    /* Store locally so stale-detection sees it. */
    g_poses[OWN_ROBOT_ID - 1u] = pose;

    /* Build BOLT packet. */
    bolt_pkt_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.type              = BOLT_POSE;
    pkt.pad               = 0;
    pkt.pose.robot_id     = OWN_ROBOT_ID;
    pkt.pose.x_fp         = pose.x_fp;
    pkt.pose.y_fp         = pose.y_fp;
    pkt.pose.theta_fp     = pose.theta_fp;
    pkt.pose.v_fp         = pose.v_fp;
    pkt.pose.timestamp_ms = now_ms;

    print_str("[AP] POSE TX robot=");
    print_u32(OWN_ROBOT_ID);
    print_str(" x="); print_fp(pose.x_fp);
    print_str(" y="); print_fp(pose.y_fp);
    print_str(" theta="); print_fp(pose.theta_fp);
    print_str(" v="); print_fp(pose.v_fp);
    print_str(" t="); print_u32(now_ms);
    print_str(" ms\r\n");

    if (bolt_write((uint8_t *)&pkt, (uint16_t)LEN_BOLT_POSE))
    {
        print_str("[AP] BOLT TX OK\r\n");
    }
    else
    {
        print_str("[AP] BOLT TX FAILED\r\n");
    }
}

/* ── update_stale_flags ─────────────────────────────────────────────────────── */
/**
 * Mark any peer whose last BOLT update is older than STALE_TIMEOUT_MS.
 * Once stale, the pose is kept in memory (for logging) but flagged invalid.
 */
static void update_stale_flags(uint32_t now_ms)
{
    uint8_t i;
    for (i = 0; i < (uint8_t)NUM_ROBOTS; i++)
    {
        if (i == OWN_ROBOT_ID - 1u)
            continue;

        if (!(g_poses[i].status & POSE_STATUS_VALID))
            continue;

        if ((now_ms - g_pose_rx_time_ms[i]) > STALE_TIMEOUT_MS)
        {
            g_poses[i].status |= POSE_STATUS_STALE;
            g_poses[i].status &= ~POSE_STATUS_VALID;

            print_str("[AP] STALE robot=");
            print_u32(i + 1u);
            print_str(" at t=");
            print_u32(now_ms);
            print_str(" ms\r\n");
        }
    }
}

/* ── main ───────────────────────────────────────────────────────────────────── */

int main(void)
{
    /* Stop watchdog immediately. */
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    /* Clock: 12 MHz DCO → SMCLK */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Peripherals */
    debug_uart_init();
    pi_uart_init(OWN_ROBOT_ID);
    systick_init();

    /* Zero all state */
    memset(g_poses,           0, sizeof(g_poses));
    memset(g_pose_rx_time_ms, 0, sizeof(g_pose_rx_time_ms));
    memset(g_pose_fwd_time_ms,0, sizeof(g_pose_fwd_time_ms));

    print_str("\r\n=== AP BOOT ===\r\n");
    print_str("OWN_ROBOT_ID=");
    print_u32(OWN_ROBOT_ID);
    print_str("\r\n");

    print_str("[AP] bolt_init...\r\n");
    uint8_t bolt_ok = bolt_init();
    print_str("[AP] bolt_init=");
    print_u32(bolt_ok);
    print_str("\r\n");

    __enable_irq();

    print_str("[AP] entering main loop\r\n");

    uint32_t last_control_ms = 0u;

    while (1)
    {
        uint32_t now = get_tick();

        /*
         * Poll UART RX as fast as possible – this feeds the frame parser
         * and latches fresh poses for pi_uart_get_pose().
         */
        pi_uart_poll(now);

        /*
         * Drain BOLT messages and forward any peer poses to the Pi.
         * Done outside the rate-limited block so we never miss a BOLT frame.
         */
        receive_from_bolt(now);

        /* 20 Hz control tick */
        if ((now - last_control_ms) >= CONTROL_PERIOD_MS)
        {
            last_control_ms = now;
            send_own_pose(now);
            update_stale_flags(now);
        }
    }
}
