#include <msp432p401r.h>
#include <string.h>
#include "../include/bolt.h"
#include "../include/message.h"
#include "../include/pi_UART.h"
#include <MSP432P4xx/cs.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>

// CHANGE THIS FOR EACH ROBOT
#define OWN_ROBOT_ID        1u

// WHAT DO WE WANT THE CLOCK SPEED TO BE? 20Hz?
#define SMCLK_HZ            12000000u

// HOW OFTEN DO WE WANT THIS TO RUN
#define CONTROL_PERIOD_MS   50u

// WHAT IS THE SAFETY RADIUS
#define SAFETY_RADIUS_M     0.5f

// Each board test position
#define TEST_FIXED_POSE 1

#define ENABLE_PI_UART_BINARY_TX 0

static void get_fixed_pose(RobotPoseMsg_t *pose, uint32_t now_ms)
{
    memset(pose, 0, sizeof(*pose));

    pose->robot_id = OWN_ROBOT_ID;

    if (OWN_ROBOT_ID == 1)
    {
        pose->x_fp     = 65536;   // 1.000
        pose->y_fp     = 65536;   // 1.000
        pose->theta_fp = 0;
        pose->v_fp     = 0;
    }
    else if (OWN_ROBOT_ID == 2)
    {
        pose->x_fp     = 131072;  // 2.000
        pose->y_fp     = 131072;  // 2.000
        pose->theta_fp = 0;
        pose->v_fp     = 0;
    }
    else if (OWN_ROBOT_ID == 3)
    {
        pose->x_fp     = 196608;  // 3.000
        pose->y_fp     = 196608;  // 3.000
        pose->theta_fp = 0;
        pose->v_fp     = 0;
    }

    pose->timestamp_ms = now_ms;
    pose->status = POSE_STATUS_VALID | POSE_STATUS_INITIALISED;
}

// system timer
static volatile uint32_t g_tick_ms = 0;


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
    uint32_t frac  = ((uint32_t)v & 0xFFFF) * 1000 / 65536;
    print_u32(whole);
    fputc('.', stdout);
    if (frac < 100) fputc('0', stdout);
    if (frac < 10)  fputc('0', stdout);
    print_u32(frac);
}


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

// latest poses
static RobotPoseMsg_t g_poses[NUM_ROBOTS];

// AP-local receive time for stale detection
static uint32_t g_pose_rx_time_ms[NUM_ROBOTS];

// scan for new BOLT msgs// scan for new BOLT msgs
static void receive_from_bolt(uint32_t now_ms)
{
    while (bolt_data_available())
    {
        uint8_t buf[BOLT_MAX_PAYLOAD];
        uint8_t len = 0;

        if (!bolt_read(buf, &len))
        {
            print_str("BOLT RX read failed\r\n");
            continue;
        }

        if (len != (uint8_t)LEN_BOLT_POSE)
        {
            // Ignore non-pose / stale / short BOLT packets.
            continue;
        }

        bolt_pkt_t *pkt = (bolt_pkt_t*)buf;

        if (pkt->type != BOLT_POSE)
        {
            print_str("BOLT RX unexpected type=");
            print_u32(pkt->type);
            print_str(" expected=");
            print_u32(BOLT_POSE);
            print_str("\r\n");
            continue;
        }

        uint8_t id = pkt->pose.robot_id;

        if (id < 1 || id > NUM_ROBOTS)
        {
            print_str("BOLT RX invalid robot_id=");
            print_u32(id);
            print_str(" range=1-");
            print_u32(NUM_ROBOTS);
            print_str("\r\n");
            continue;
        }

        if (id == OWN_ROBOT_ID)
        {
            // Normal: ignore own robot pose if it comes back.
            continue;
        }

        uint8_t idx = id - 1;

        uint32_t rx_gap_ms = 0;
        if (g_pose_rx_time_ms[idx] != 0)
        {
            rx_gap_ms = now_ms - g_pose_rx_time_ms[idx];
        }

        // This is the important line for stale detection
        g_pose_rx_time_ms[idx] = now_ms;

        RobotPoseMsg_t *p = &g_poses[idx];

        p->robot_id     = id;
        p->x_fp         = pkt->pose.x_fp;
        p->y_fp         = pkt->pose.y_fp;
        p->theta_fp     = pkt->pose.theta_fp;
        p->v_fp         = pkt->pose.v_fp;
        p->timestamp_ms = pkt->pose.timestamp_ms;
        p->status       = POSE_STATUS_VALID | POSE_STATUS_INITIALISED;

        print_str("BOLT RX t=");
        print_u32(p->timestamp_ms);

        print_str("  robot=");
        print_u32(p->robot_id);

        print_str("  x=");
        print_fp(p->x_fp);

        print_str("  y=");
        print_fp(p->y_fp);

        print_str("  theta=");
        print_fp(p->theta_fp);

        print_str("  v=");
        print_fp(p->v_fp);

        print_str("  pkt_age=");
        print_u32(rx_gap_ms);
        print_str(" ms");
                
        print_str("\r\n");

        #if ENABLE_PI_UART_BINARY_TX
        pi_uart_send_pose(p);
        #endif
    }
}

// build and send own pose to CP
static void send_own_pose(uint32_t now_ms) {
    RobotPoseMsg_t pose;

#if TEST_FIXED_POSE
    get_fixed_pose(&pose, now_ms);
#else
    if (!pi_uart_get_pose(&pose)) {
        return;
    }
#endif


    pose.robot_id     = OWN_ROBOT_ID;
    pose.timestamp_ms = now_ms;
    pose.status       = POSE_STATUS_VALID | POSE_STATUS_INITIALISED;
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

    print_str("POSE TX t=");
    print_u32(now_ms);
    print_str("  robot=");
    print_u32(OWN_ROBOT_ID);
    print_str("  x=");
    print_fp(pose.x_fp);
    print_str("  y=");
    print_fp(pose.y_fp);
    print_str("  theta=");
    print_fp(pose.theta_fp);
    print_str("  v=");
    print_fp(pose.v_fp);
    print_str("\r\n");

    uint8_t ok = bolt_write((uint8_t*)&pkt, (uint16_t)LEN_BOLT_POSE);

    if (ok) {
        print_str("BOLT TX sent ");
        print_u32(LEN_BOLT_POSE);
        print_str(" bytes OK\r\n");
    } else {
        print_str("BOLT TX write FAILED\r\n");
    }
}

// mark as stale data if no update for a while
static void update_stale_flags(uint32_t now_ms)
{
    uint8_t i;

    for (i = 0; i < NUM_ROBOTS; i++)
    {
        if (i == OWN_ROBOT_ID - 1)
            continue;

        if (!(g_poses[i].status & POSE_STATUS_VALID))
            continue;

        if ((now_ms - g_pose_rx_time_ms[i]) > 500u)
        {
            if (!(g_poses[i].status & POSE_STATUS_STALE))
            {
                g_poses[i].status |= POSE_STATUS_STALE;
                g_poses[i].status &= ~POSE_STATUS_VALID;

                print_str("[STALE] robot ");
                print_u32(i + 1);
                print_str(" marked stale at t=");
                print_u32(now_ms);
                print_str("\r\n");
            }
        }
    }
}

int main(void) {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12); 
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    pi_uart_init(OWN_ROBOT_ID);

    printf("\r\n*** AP FIXED POSE TEST BOOT ***\r\n");
    printf("OWN_ROBOT_ID=%u\r\n", OWN_ROBOT_ID);
    printf("TEST_FIXED_POSE=%u\r\n", TEST_FIXED_POSE);

    systick_init();

    printf("before bolt_init\r\n");
    uint8_t b = bolt_init();
    printf("after bolt_init = %u\r\n", b);

    __enable_irq();

    printf("enter loop\r\n");
    uint32_t last_control_ms = 0u;

    while (1) {
        uint32_t now = get_tick();
        
        pi_uart_poll(now);
        receive_from_bolt(now);

        
        if ((now - last_control_ms) >= CONTROL_PERIOD_MS) {
            last_control_ms = now;
            send_own_pose(now);
            update_stale_flags(now);
        }
        
        /* every X ms (20 Hz) send pose and correction
        if ((now - last_control_ms) >= CONTROL_PERIOD_MS) {
            last_control_ms = now;
            update_stale_flags(now);
            send_own_pose(now);
            send_correction(now);
        }
        */
    }
}
