#include <msp432p401r.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include "../include/bolt.h"
#include "../include/message.h"

#include <MSP432P4xx/uart.h>
#include <MSP432P4xx/gpio.h>
#include <MSP432P4xx/cs.h>

// ----------------------------------------------------------------
// Config
// ----------------------------------------------------------------
#define OWN_ROBOT_ID     0u
#define SMCLK_HZ         12000000u
#define TEST_PERIOD_MS   50u

// ----------------------------------------------------------------
// fputc — direct register access
// ----------------------------------------------------------------
int fputc(int c, FILE *f) {
    (void)f;
    while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A0->TXBUF = (uint8_t)c;
    return c;
}

// ----------------------------------------------------------------
// Print helpers — bypass printf integer support entirely
// ----------------------------------------------------------------

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

// print Q16.16 fixed-point value as x.xxx decimal
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

static void print_hex8(uint8_t v) {
    const char *h = "0123456789ABCDEF";
    fputc(h[v >> 4],  stdout);
    fputc(h[v & 0xF], stdout);
}

// ----------------------------------------------------------------
// SysTick — 1ms tick
// ----------------------------------------------------------------
static volatile uint32_t g_tick_ms = 0;

void SysTick_Handler(void) { g_tick_ms++; }

static void systick_init(void) {
    SysTick->LOAD = (SMCLK_HZ / 1000) - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;
}

// ----------------------------------------------------------------
// Fake pose table (Q16.16 fixed point, pre-computed)
// ----------------------------------------------------------------
typedef struct { int32_t x_fp, y_fp, theta_fp, v_fp; } FakePose_t;

static const FakePose_t fake_poses[] = {
    {      0,      0,      0,      0 },  // 0.00, 0.00, 0.000, 0.00
    {   6554,      0,      0,  13107 },  // 0.10, 0.00, 0.000, 0.20
    {  13107,      0,      0,  13107 },  // 0.20, 0.00, 0.000, 0.20
    {  19661,   3277,   6554,  13107 },  // 0.30, 0.05, 0.100, 0.20
    {  26214,   9830,  19661,  13107 },  // 0.40, 0.15, 0.300, 0.20
    {  29491,  19661,  51418,   9830 },  // 0.45, 0.30, 0.785, 0.15
    {  26214,  29491, 102958,  13107 },  // 0.40, 0.45, 1.571, 0.20
    {  26214,  39322, 102958,  13107 },  // 0.40, 0.60, 1.571, 0.20
    {  22938,  55705, 154399,   9830 },  // 0.35, 0.85, 2.356, 0.15
    {  13107,  58982, 205840,  13107 },  // 0.20, 0.90, 3.142, 0.20
    {      0,  58982, 205840,  13107 },  // 0.00, 0.90, 3.142, 0.20
    {      0,      0,      0,      0 },  // back to start
};
#define NUM_FAKE_POSES (sizeof(fake_poses) / sizeof(fake_poses[0]))

static uint8_t g_fake_idx = 0;

// ----------------------------------------------------------------
// Main
// ----------------------------------------------------------------
int main(void) {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // UART init — same as working hello test
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P1, GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(
        GPIO_PORT_P1, GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    const eUSCI_UART_Config cfg = {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        6, 8, 0x20,
        EUSCI_A_UART_NO_PARITY,
        EUSCI_A_UART_LSB_FIRST,
        EUSCI_A_UART_ONE_STOP_BIT,
        EUSCI_A_UART_MODE,
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
    };
    UART_initModule(EUSCI_A0_BASE, &cfg);
    UART_enableModule(EUSCI_A0_BASE);

    print_str("\r\n===========================\r\n");
    print_str("  AP hardware test\r\n");
    print_str("  Robot ID : "); print_u32(OWN_ROBOT_ID); print_str("\r\n");
    print_str("  Period   : "); print_u32(TEST_PERIOD_MS); print_str(" ms\r\n");
    print_str("  Poses    : "); print_u32(NUM_FAKE_POSES); print_str(" (cycling)\r\n");
    print_str("===========================\r\n\r\n");

    systick_init();
    print_str("SysTick OK\r\n");

    __enable_irq();

    uint8_t bolt_ok = bolt_init();
    print_str("[INIT] Bolt: ");
    print_str(bolt_ok ? "OK" : "FAILED - CP not responding");
    print_str("\r\n\r\n");

    uint32_t last_ms = 0;

    while (1) {
        uint32_t now = g_tick_ms;
        if ((now - last_ms) < TEST_PERIOD_MS) continue;
        last_ms = now;

        const FakePose_t *fp = &fake_poses[g_fake_idx];
        g_fake_idx = (g_fake_idx + 1) % NUM_FAKE_POSES;

        RobotPoseMsg_t pose;
        memset(&pose, 0, sizeof(pose));
        pose.timestamp_ms = now;
        pose.robot_id     = OWN_ROBOT_ID;
        pose.status       = POSE_STATUS_VALID | POSE_STATUS_INITIALISED;
        pose.x_fp         = fp->x_fp;
        pose.y_fp         = fp->y_fp;
        pose.theta_fp     = fp->theta_fp;
        pose.v_fp         = fp->v_fp;

        print_str("[t="); print_u32(now); print_str(" ms] FAKE POSE #");
        print_u32(g_fake_idx); print_str("\r\n");

        print_str("  x=");     print_fp(pose.x_fp);
        print_str("  y=");     print_fp(pose.y_fp);
        print_str("  theta="); print_fp(pose.theta_fp);
        print_str("  v=");     print_fp(pose.v_fp);
        print_str("\r\n");

        uint8_t tx_ok = bolt_write((uint8_t*)&pose, sizeof(pose));
        print_str("  BOLT TX: ");
        print_str(tx_ok ? "OK" : "FAILED");
        print_str(" ("); print_u32(sizeof(pose)); print_str(" bytes)\r\n");

        if (bolt_data_available()) {
            uint8_t buf[BOLT_MAX_PAYLOAD];
            uint8_t len = 0;
            uint8_t rx_ok = bolt_read(buf, &len);

            if (rx_ok && len == sizeof(RobotPoseMsg_t)) {
                RobotPoseMsg_t rx;
                memcpy(&rx, buf, sizeof(rx));
                print_str("  BOLT RX: robot="); print_u32(rx.robot_id);
                print_str("  x=");     print_fp(rx.x_fp);
                print_str("  y=");     print_fp(rx.y_fp);
                print_str("  theta="); print_fp(rx.theta_fp);
                print_str("  v=");     print_fp(rx.v_fp);
                print_str("  status=0x"); print_hex8(rx.status);
                print_str("\r\n");
            } else if (rx_ok) {
                print_str("  BOLT RX: unexpected len=");
                print_u32(len); print_str("\r\n");
            } else {
                print_str("  BOLT RX: read failed\r\n");
            }
        } else {
            print_str("  BOLT RX: nothing waiting\r\n");
        }

        print_str("\r\n");
    }
}
