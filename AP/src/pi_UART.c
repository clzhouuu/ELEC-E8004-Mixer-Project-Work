#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <msp432p401r.h>
#include "MSP432P4xx/gpio.h"
#include "MSP432P4xx/uart.h"
#include "board_config.h"
#include "pi_UART.h"

#define PI_UART_USE_460800 1

static RobotPoseMsg_t g_latest_pose;
static uint8_t g_robot_id = 0;
static uint8_t g_fresh_pose = 0;

static char g_line_buf[PI_UART_MAX_PAYLOAD + 1];
static uint16_t g_line_idx = 0;

static void process_json_line(const char *json, uint32_t now_ms)
{
    unsigned tag = 0u;
    unsigned seq = 0u;
    unsigned long long t_ns = 0ull;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 1.0f;

    int matched = sscanf(
        json,
        "{\"tag\":%u,\"seq\":%u,\"t_ns\":%llu,\"x\":%f,\"y\":%f,\"z\":%f,\"qx\":%f,\"qy\":%f,\"qz\":%f,\"qw\":%f}",
        &tag, &seq, &t_ns, &x, &y, &z, &qx, &qy, &qz, &qw
    );

    if (matched < 10) {
        matched = sscanf(
            json,
            "{\"tag\": %u, \"seq\": %u, \"t_ns\": %llu, \"x\": %f, \"y\": %f, \"z\": %f, \"qx\": %f, \"qy\": %f, \"qz\": %f, \"qw\": %f}",
            &tag, &seq, &t_ns, &x, &y, &z, &qx, &qy, &qz, &qw
        );
    }

    if (matched < 10) {
        char tag_text[16];
        matched = sscanf(
            json,
            "{\"tag\":\"%15[^\"]\",\"seq\":%u,\"t_ns\":%llu,\"x\":%f,\"y\":%f,\"z\":%f,\"qx\":%f,\"qy\":%f,\"qz\":%f,\"qw\":%f}",
            tag_text, &seq, &t_ns, &x, &y, &z, &qx, &qy, &qz, &qw
        );
    }

    if (matched < 10) {
        char tag_text[16];
        matched = sscanf(
            json,
            "{\"tag\": \"%15[^\"]\", \"seq\": %u, \"t_ns\": %llu, \"x\": %f, \"y\": %f, \"z\": %f, \"qx\": %f, \"qy\": %f, \"qz\": %f, \"qw\": %f}",
            tag_text, &seq, &t_ns, &x, &y, &z, &qx, &qy, &qz, &qw
        );
    }

    if (matched < 10) {
        return;
    }

    (void)tag;
    g_latest_pose.robot_id = g_robot_id;
    g_latest_pose.seq = (uint16_t)seq;
    g_latest_pose.t_ns = (uint64_t)t_ns;
    if (g_latest_pose.t_ns == 0ull) {
        g_latest_pose.t_ns = ((uint64_t)now_ms) * 1000000ull;
    }
    g_latest_pose.status = POSE_STATUS_VALID | POSE_STATUS_INITIALISED;
    g_latest_pose.x_fp = FP_FROM_FLOAT(x);
    g_latest_pose.y_fp = FP_FROM_FLOAT(y);
    g_latest_pose.z_fp = FP_FROM_FLOAT(z);
    g_latest_pose.qx_fp = FP_FROM_FLOAT(qx);
    g_latest_pose.qy_fp = FP_FROM_FLOAT(qy);
    g_latest_pose.qz_fp = FP_FROM_FLOAT(qz);
    g_latest_pose.qw_fp = FP_FROM_FLOAT(qw);
    g_fresh_pose = 1;
}

static void uart_feed_byte(uint8_t byte, uint32_t now_ms)
{
    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        g_line_buf[g_line_idx] = '\0';
        if (g_line_idx > 0u) {
            process_json_line(g_line_buf, now_ms);
        }
        g_line_idx = 0;
        return;
    }

    if (g_line_idx < PI_UART_MAX_PAYLOAD) {
        g_line_buf[g_line_idx++] = (char)byte;
    } else {
        g_line_idx = 0;
    }
}

void pi_uart_init(uint8_t robot_id)
{
    g_robot_id = robot_id;
    memset(&g_latest_pose, 0, sizeof(g_latest_pose));
    g_fresh_pose = 0;
    g_line_idx = 0;

    GPIO_setAsPeripheralModuleFunctionInputPin(
        UART_RX_PORT,
        UART_RX_PIN,
        GPIO_PRIMARY_MODULE_FUNCTION
    );

    GPIO_setAsPeripheralModuleFunctionOutputPin(
        UART_TX_PORT,
        UART_TX_PIN,
        GPIO_PRIMARY_MODULE_FUNCTION
    );

#if PI_UART_USE_460800
    const eUSCI_UART_Config uartConfig = {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        1,
        10,
        0,
        EUSCI_A_UART_NO_PARITY,
        EUSCI_A_UART_LSB_FIRST,
        EUSCI_A_UART_ONE_STOP_BIT,
        EUSCI_A_UART_MODE,
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
    };
#else
    const eUSCI_UART_Config uartConfig = {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        6,
        8,
        0x20,
        EUSCI_A_UART_NO_PARITY,
        EUSCI_A_UART_LSB_FIRST,
        EUSCI_A_UART_ONE_STOP_BIT,
        EUSCI_A_UART_MODE,
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
    };
#endif

    UART_initModule(EUSCI_A0_BASE, &uartConfig);
    UART_enableModule(EUSCI_A0_BASE);
}

void pi_uart_poll(uint32_t now_ms)
{
    while (UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)) {
        uint8_t byte = UART_receiveData(EUSCI_A0_BASE);
        uart_feed_byte(byte, now_ms);
    }
}

uint8_t pi_uart_get_pose(RobotPoseMsg_t *out_pose)
{
    if (!g_fresh_pose) {
        return 0;
    }

    *out_pose = g_latest_pose;
    g_fresh_pose = 0;
    return 1;
}

static void uart_tx_byte(uint8_t b)
{
    while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A0->TXBUF = b;
}

void pi_uart_send_pose(const RobotPoseMsg_t *pose)
{
    if (!pose) {
        return;
    }

    char line[256];
    float x = FP_TO_FLOAT(pose->x_fp);
    float y = FP_TO_FLOAT(pose->y_fp);
    float z = FP_TO_FLOAT(pose->z_fp);
    float qx = FP_TO_FLOAT(pose->qx_fp);
    float qy = FP_TO_FLOAT(pose->qy_fp);
    float qz = FP_TO_FLOAT(pose->qz_fp);
    float qw = FP_TO_FLOAT(pose->qw_fp);

    int n = snprintf(
        line,
        sizeof(line),
        "{\"tag\":%u,\"seq\":%u,\"t_ns\":%llu,\"x\":%.6f,\"y\":%.6f,\"z\":%.6f,\"qx\":%.6f,\"qy\":%.6f,\"qz\":%.6f,\"qw\":%.6f}\n",
        (unsigned)pose->robot_id,
        (unsigned)pose->seq,
        (unsigned long long)pose->t_ns,
        x, y, z, qx, qy, qz, qw
    );

    int i;
    if (n <= 0 || n >= (int)sizeof(line)) {
        return;
    }

    for (i = 0; i < n; i++) {
        uart_tx_byte((uint8_t)line[i]);
    }
}

void pi_uart_test_dummy_json(void)
{
    const char *test =
        "{\"tag\":1,\"seq\":1,\"t_ns\":1234000000,\"x\":1.234,\"y\":2.345,\"z\":0.000,\"qx\":0.000,\"qy\":0.000,\"qz\":0.000,\"qw\":1.000}\n";
    uint32_t fake_time_ms = 1234;
    uint16_t i;

    for (i = 0; test[i] != '\0'; i++) {
        uart_feed_byte((uint8_t)test[i], fake_time_ms);
    }
}
