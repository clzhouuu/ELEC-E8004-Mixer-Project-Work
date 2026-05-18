#include <msp432p401r.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>
#include "../include/bolt.h"
#include "../include/message.h"
#include "../include/pi_UART.h"
#include <MSP432P4xx/cs.h>

// -----------------------------------------------------------------------
// Configuration — change per robot
// -----------------------------------------------------------------------
#define OWN_ROBOT_ID        1u
#define SMCLK_HZ            12000000u
#define CONTROL_PERIOD_MS   50u
#define SAFETY_RADIUS_M     0.5f

// -----------------------------------------------------------------------
// System timer
// -----------------------------------------------------------------------
static volatile uint32_t g_tick_ms = 0;

void SysTick_Handler(void) {
    g_tick_ms++;
}

static void systick_init(void) {
    SysTick->LOAD = (SMCLK_HZ / 1000) - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;
}

static uint32_t get_tick(void) {
    return g_tick_ms;
}

// -----------------------------------------------------------------------
// Debug print helpers (use UART TX directly, no stdlib heap)
// -----------------------------------------------------------------------
int fputc(int c, FILE *f) {
    (void)f;
    while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A0->TXBUF = (uint8_t)c;
    return c;
}

static void print_str(const char *s) {
    while (*s) fputc(*s++, stdout);
}

static void print_u32(uint32_t v) {
    char buf[11];
    int i = 10;
    buf[i] = '\0';
    if (v == 0) { fputc('0', stdout); return; }
    while (v && i > 0) { buf[--i] = '0' + (v % 10); v /= 10; }
    print_str(buf + i);
}

static void print_fp(int32_t v) {
    if (v < 0) { fputc('-', stdout); v = -v; }
    uint32_t whole = (uint32_t)v >> 16;
    uint32_t frac  = ((uint32_t)v & 0xFFFF) * 1000u / 65536u;
    print_u32(whole);
    fputc('.', stdout);
    if (frac < 100) fputc('0', stdout);
    if (frac < 10)  fputc('0', stdout);
    print_u32(frac);
}

// -----------------------------------------------------------------------
// Global state
// -----------------------------------------------------------------------
static RobotPoseMsg_t g_poses[NUM_ROBOTS];

// -----------------------------------------------------------------------
// Receive peer pose from BOLT, forward to Pi
// -----------------------------------------------------------------------
static void receive_from_bolt(uint32_t now_ms) {
    while (bolt_data_available()) {

        uint8_t buf[BOLT_MAX_PAYLOAD];
        uint8_t len = 0;

        if (!bolt_read(buf, &len)) {
            printf("BOLT RX read failed\r\n");
            continue;
        }

        bolt_pkt_t *pkt = (bolt_pkt_t *)buf;

        // ---- Peer pose ----
        if (pkt->type == BOLT_POSE) {

            if (len < (uint8_t)LEN_BOLT_POSE) {
                printf("BOLT RX POSE too short len=%u\r\n", len);
                continue;
            }

            uint8_t id = pkt->pose.robot_id;

            if (id < 1 || id > NUM_ROBOTS) {
                printf("BOLT RX POSE invalid robot_id=%u\r\n", id);
                continue;
            }
            if (id == OWN_ROBOT_ID) {
                printf("BOLT RX POSE ignoring own id=%u\r\n", id);
                continue;
            }

            RobotPoseMsg_t *p = &g_poses[id - 1];
            p->robot_id     = id;
            p->x_fp         = pkt->pose.x_fp;
            p->y_fp         = pkt->pose.y_fp;
            p->theta_fp     = pkt->pose.theta_fp;
            p->v_fp         = pkt->pose.v_fp;
            p->timestamp_ms = now_ms;
            p->status       = POSE_STATUS_VALID | POSE_STATUS_INITIALISED;

            printf("BOLT RX POSE robot=%u  x=%.3f  y=%.3f  theta=%.3f  v=%.3f\r\n",
                   p->robot_id,
                   FP_TO_FLOAT(p->x_fp),
                   FP_TO_FLOAT(p->y_fp),
                   FP_TO_FLOAT(p->theta_fp),
                   FP_TO_FLOAT(p->v_fp));

            pi_uart_send_pose(p);

        // ---- Scan fragment received from CP (not currently expected but handled) ----
        } else if (pkt->type == BOLT_SCAN) {
            // CP does not send scan data back to APP in this design.
            // This case is included for completeness/future use.
            printf("BOLT RX SCAN frag=%u/%u (unexpected from CP)\r\n",
                   pkt->scan.frag_id, pkt->scan.total_frags);

        } else {
            printf("BOLT RX unknown type=%u\r\n", pkt->type);
        }
    }
}

// -----------------------------------------------------------------------
// Receive own pose from Pi via UART, forward to CP via BOLT
// -----------------------------------------------------------------------
static void send_own_pose(uint32_t now_ms) {
    RobotPoseMsg_t pose;

    if (!pi_uart_get_pose(&pose)) {
        return;
    }

    g_poses[OWN_ROBOT_ID - 1] = pose;

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

    print_str("POSE TX t=");  print_u32(now_ms);
    print_str("  x=");        print_fp(pose.x_fp);
    print_str("  y=");        print_fp(pose.y_fp);
    print_str("  theta=");    print_fp(pose.theta_fp);
    print_str("  v=");        print_fp(pose.v_fp);
    print_str("\r\n");

    if (bolt_write((uint8_t *)&pkt, (uint16_t)LEN_BOLT_POSE)) {
        print_str("BOLT TX POSE OK\r\n");
    } else {
        print_str("BOLT TX POSE FAILED\r\n");
    }
}

// -----------------------------------------------------------------------
// Receive LiDAR scan fragments from Pi via UART, forward to CP via BOLT.
//
// The Pi sends one PiScanFrag_t per UART frame (MSG_SCAN_FRAG).
// Each fragment maps 1:1 onto a BOLT_SCAN packet (scan_pkt_t fields match).
// We forward every fragment immediately as it arrives so the CP receives
// the full scan across successive BOLT writes.
// -----------------------------------------------------------------------
static void send_scan_fragments(uint32_t now_ms) {
    PiScanFrag_t frag;

    // Drain all pending fragments this poll cycle
    while (pi_uart_get_scan_frag(&frag)) {

        bolt_pkt_t pkt;
        memset(&pkt, 0, sizeof(pkt));

        pkt.type = BOLT_SCAN;
        pkt.pad  = 0;

        pkt.scan.robot_id     = OWN_ROBOT_ID;
        pkt.scan.frag_id      = frag.frag_id;
        pkt.scan.total_frags  = frag.total_frags;
        pkt.scan.count        = frag.count;
        pkt.scan.angle_start  = frag.angle_start;
        pkt.scan.angle_step   = frag.angle_step;
        pkt.scan.timestamp_ms = now_ms;

        // Copy range data (frag.count valid entries, rest already zero from memset)
        uint8_t i;
        for (i = 0; i < frag.count && i < SCAN_POINTS_PER_FRAG; i++) {
            pkt.scan.ranges[i] = frag.ranges[i];
        }

        print_str("SCAN TX frag=");
        print_u32(frag.frag_id);
        print_str("/");
        print_u32(frag.total_frags);
        print_str(" count=");
        print_u32(frag.count);
        print_str("\r\n");

        if (!bolt_write((uint8_t *)&pkt, (uint16_t)LEN_BOLT_SCAN)) {
            print_str("BOLT TX SCAN FAILED frag=");
            print_u32(frag.frag_id);
            print_str("\r\n");
        }
    }
}

// -----------------------------------------------------------------------
// Mark peer poses stale if no update for >500 ms
// -----------------------------------------------------------------------
static void update_stale_flags(uint32_t now_ms) {
    uint8_t i;
    for (i = 0; i < NUM_ROBOTS; i++) {
        if (i == OWN_ROBOT_ID - 1) {
            continue;
        }
        if (!(g_poses[i].status & POSE_STATUS_VALID)) {
            continue;
        }
        if ((now_ms - g_poses[i].timestamp_ms) > 500u) {
            g_poses[i].status |= POSE_STATUS_STALE;
            printf("[STALE] robot %u at t=%lu\r\n", i + 1u, (unsigned long)now_ms);
        }
    }
}

// -----------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------
int main(void) {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    pi_uart_init(OWN_ROBOT_ID);
    systick_init();
    bolt_init();

    __enable_irq();

    uint32_t last_control_ms = 0u;

    while (1) {
        uint32_t now = get_tick();

        // Poll UART for incoming frames from Pi (pose + scan fragments)
        pi_uart_poll(now);

        // Forward any new scan fragments to CP immediately (not rate-limited,
        // so the CP gets all fragments of a scan without artificial delay)
        send_scan_fragments(now);

        // Receive peer data from CP and forward back to Pi
        receive_from_bolt(now);

        // Rate-limited control cycle (20 Hz)
        if ((now - last_control_ms) >= CONTROL_PERIOD_MS) {
            last_control_ms = now;
            send_own_pose(now);
            update_stale_flags(now);
        }
    }
}
