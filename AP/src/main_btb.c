#include <msp432p401r.h>
#include <string.h>
#include "../include/bolt.h"
#include "../include/message.h"
#include "../include/pi_UART.h"
#include <MSP432P4xx/cs.h>
#include <stdio.h>
#include <stddef.h>

// CHANGE THIS FOR EACH ROBOT
#define OWN_ROBOT_ID        1u

#define SMCLK_HZ            12000000u
#define CONTROL_PERIOD_MS   200u
#define STALE_TIMEOUT_MS    500u
#define PEER_FWD_MIN_MS     50u

// Real Pi mode:
//   TEST_FIXED_POSE 0
//   DEBUG_PRINTS 0
//   ENABLE_PI_UART_JSON_TX 1
//   DUMMY_JSON_TEST 0
#define TEST_FIXED_POSE             0
#define DEBUG_PRINTS                1
#define ENABLE_PI_UART_JSON_TX      1
#define DUMMY_JSON_TEST             0


static uint16_t g_pose_seq = 0;

static void get_fixed_pose(RobotPoseMsg_t *pose, uint32_t now_ms)
{
    memset(pose, 0, sizeof(*pose));
    pose->robot_id = OWN_ROBOT_ID;

    if (OWN_ROBOT_ID == 1u) {
        pose->x_fp = 65536;
        pose->y_fp = 65536;
    } else if (OWN_ROBOT_ID == 2u) {
        pose->x_fp = 131072;
        pose->y_fp = 131072;
    } else if (OWN_ROBOT_ID == 3u) {
        pose->x_fp = 196608;
        pose->y_fp = 196608;
    }

    pose->z_fp = 0;
    pose->qx_fp = 0;
    pose->qy_fp = 0;
    pose->qz_fp = 0;
    pose->qw_fp = FP_FROM_FLOAT(1.0f);
    pose->t_ns = ((uint64_t)now_ms) * 1000000ull;
    pose->status = POSE_STATUS_VALID | POSE_STATUS_INITIALISED;
}

static volatile uint32_t g_tick_ms = 0;

int fputc(int c, FILE *f)
{
    (void)f;
#if DEBUG_PRINTS
    while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A0->TXBUF = (uint8_t)c;
#endif
    return c;
}

static void print_str(const char *s)
{
#if DEBUG_PRINTS
    while (*s) {
        fputc(*s++, stdout);
    }
#else
    (void)s;
#endif
}

static void print_u32(uint32_t v)
{
#if DEBUG_PRINTS
    char buf[11];
    int i;

    i = 10;
    buf[i] = '\0';
    if (v == 0u) {
        fputc('0', stdout);
        return;
    }
    while (v && i > 0) {
        buf[--i] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    print_str(buf + i);
#else
    (void)v;
#endif
}

static void print_fp(int32_t v)
{
#if DEBUG_PRINTS
    uint32_t whole;
    uint32_t frac;

    if (v < 0) {
        fputc('-', stdout);
        v = -v;
    }
    whole = (uint32_t)v >> 16;
    frac = ((uint32_t)v & 0xFFFFu) * 1000u / 65536u;
    print_u32(whole);
    fputc('.', stdout);
    if (frac < 100u) fputc('0', stdout);
    if (frac < 10u)  fputc('0', stdout);
    print_u32(frac);
#else
    (void)v;
#endif
}

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

static uint32_t get_tick(void)
{
    return g_tick_ms;
}

static RobotPoseMsg_t g_poses[NUM_ROBOTS];
static uint32_t g_pose_rx_time_ms[NUM_ROBOTS];
static uint32_t g_pose_fwd_time_ms[NUM_ROBOTS];

static void receive_from_bolt(uint32_t now_ms)
{
    while (bolt_data_available()) {
        uint8_t buf[BOLT_MAX_PAYLOAD];
        uint8_t len;
        bolt_pkt_t *pkt;

        len = 0;
        if (!bolt_read(buf, &len)) {
            print_str("BOLT RX read failed\r\n");
            continue;
        }

        pkt = (bolt_pkt_t*)buf;

        if (len == (uint8_t)LEN_BOLT_POSE && pkt->type == BOLT_POSE) {
            uint8_t id;
            uint8_t idx;
            RobotPoseMsg_t *p;

            id = pkt->pose.robot_id;
            if (id < 1u || id > (uint8_t)NUM_ROBOTS) {
                continue;
            }
            if (id == OWN_ROBOT_ID) {
                continue;
            }

            idx = id - 1u;
            g_pose_rx_time_ms[idx] = now_ms;

            p = &g_poses[idx];
            p->robot_id     = id;
            p->x_fp         = pkt->pose.x_fp;
            p->y_fp         = pkt->pose.y_fp;
            p->seq          = pkt->pose.seq;
            p->t_ns         = pkt->pose.t_ns;
            p->z_fp         = pkt->pose.z_fp;
            p->qx_fp        = pkt->pose.qx_fp;
            p->qy_fp        = pkt->pose.qy_fp;
            p->qz_fp        = pkt->pose.qz_fp;
            p->qw_fp        = pkt->pose.qw_fp;
            p->status       = POSE_STATUS_VALID | POSE_STATUS_INITIALISED;

            print_str("BOLT RX POSE robot=");
            print_u32(p->robot_id);
            print_str(" x="); print_fp(p->x_fp);
            print_str(" y="); print_fp(p->y_fp);
            print_str(" z="); print_fp(p->z_fp);
            print_str("\r\n");

#if ENABLE_PI_UART_JSON_TX
            if ((now_ms - g_pose_fwd_time_ms[idx]) >= PEER_FWD_MIN_MS) {
                g_pose_fwd_time_ms[idx] = now_ms;
                pi_uart_send_pose(p);
            }
#endif
        }
    }
}

static void send_own_pose(uint32_t now_ms)
{
    RobotPoseMsg_t pose;
    bolt_pkt_t pkt;
    uint8_t ok;

#if TEST_FIXED_POSE
    get_fixed_pose(&pose, now_ms);
#else
    if (!pi_uart_get_pose(&pose)) {
        return;
    }
#endif

    pose.robot_id = OWN_ROBOT_ID;
    if (pose.t_ns == 0ull) {
        pose.t_ns = ((uint64_t)now_ms) * 1000000ull;
    }
    pose.status = POSE_STATUS_VALID | POSE_STATUS_INITIALISED;
    g_poses[OWN_ROBOT_ID - 1u] = pose;

    memset(&pkt, 0, sizeof(pkt));
    pkt.type = BOLT_POSE;
    pkt.pad = 0;
    pkt.pose.robot_id = OWN_ROBOT_ID;
    pkt.pose.x_fp = pose.x_fp;
    pkt.pose.y_fp = pose.y_fp;
    pkt.pose.z_fp = pose.z_fp;
    pkt.pose.qx_fp = pose.qx_fp;
    pkt.pose.qy_fp = pose.qy_fp;
    pkt.pose.qz_fp = pose.qz_fp;
    pkt.pose.qw_fp = pose.qw_fp;
    pkt.pose.t_ns = pose.t_ns;

    pkt.pose.seq = pose.seq ? pose.seq : ++g_pose_seq;
    pkt.pose.cp_tx_round = 0;

    print_str("POSE TX t=");
    print_u32(now_ms);
    print_str(" robot="); print_u32(OWN_ROBOT_ID);
    print_str(" x="); print_fp(pose.x_fp);
    print_str(" y="); print_fp(pose.y_fp);
    print_str(" z="); print_fp(pose.z_fp);
    print_str("\r\n");

    ok = bolt_write((uint8_t*)&pkt, (uint16_t)LEN_BOLT_POSE);
    if (ok) {
        print_str("BOLT TX POSE OK\r\n");
    } else {
        print_str("BOLT TX POSE FAILED\r\n");
    }
}

static void update_stale_flags(uint32_t now_ms)
{
    uint8_t i;

    for (i = 0; i < (uint8_t)NUM_ROBOTS; i++) {
        if (i == OWN_ROBOT_ID - 1u) {
            continue;
        }
        if (!(g_poses[i].status & POSE_STATUS_VALID)) {
            continue;
        }
        if ((now_ms - g_pose_rx_time_ms[i]) > STALE_TIMEOUT_MS) {
            g_poses[i].status |= POSE_STATUS_STALE;
            g_poses[i].status &= ~POSE_STATUS_VALID;
            print_str("[STALE] robot ");
            print_u32(i + 1u);
            print_str(" marked stale at t=");
            print_u32(now_ms);
            print_str("\r\n");
        }
    }
}

int main(void)
{
    uint8_t b;
    uint32_t last_control_ms;


    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    pi_uart_init(OWN_ROBOT_ID);

#if DUMMY_JSON_TEST
    pi_uart_test_dummy_json();
#if DEBUG_PRINTS
    {
        RobotPoseMsg_t test_pose;
        if (pi_uart_get_pose(&test_pose)) {
            print_str("DUMMY JSON PARSED x="); print_fp(test_pose.x_fp);
            print_str(" y="); print_fp(test_pose.y_fp);
            print_str(" z="); print_fp(test_pose.z_fp);
            print_str("\r\n");
            pi_uart_test_dummy_json();
        } else {
            print_str("DUMMY JSON FAILED\r\n");
        }
    }
#endif
#endif

#if DEBUG_PRINTS
    printf("\r\n*** AP JSON POSE BOOT ***\r\n");
    printf("OWN_ROBOT_ID=%u\r\n", OWN_ROBOT_ID);
    printf("TEST_FIXED_POSE=%u\r\n", TEST_FIXED_POSE);
#endif

    systick_init();

#if DEBUG_PRINTS
    printf("before bolt_init\r\n");
#endif

    b = bolt_init();
    (void)b;

#if DEBUG_PRINTS
    printf("after bolt_init = %u\r\n", b);
#endif

    __enable_irq();

#if DEBUG_PRINTS
    printf("enter loop\r\n");
#endif

    memset(g_poses, 0, sizeof(g_poses));
    memset(g_pose_rx_time_ms, 0, sizeof(g_pose_rx_time_ms));
    memset(g_pose_fwd_time_ms, 0, sizeof(g_pose_fwd_time_ms));

    last_control_ms = 0u;

    while (1) {
        uint32_t now;

        now = get_tick();
        pi_uart_poll(now);
        receive_from_bolt(now);

        if ((now - last_control_ms) >= CONTROL_PERIOD_MS) {
            last_control_ms = now;

#if DUMMY_JSON_TEST
            pi_uart_test_dummy_json();
#endif
            send_own_pose(now);
            update_stale_flags(now);
        }
    }
}
