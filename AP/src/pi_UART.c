#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <msp432p401r.h>
#include "MSP432P4xx/gpio.h"
#include "MSP432P4xx/uart.h"
#include "board_config.h"
#include "pi_UART.h"

// local varibales
static RobotPoseMsg_t g_latest_pose;
static uint8_t g_robot_id = 0;
static uint8_t g_fresh_pose = 0;


static char g_line_buf[PI_UART_MAX_PAYLOAD + 1];
static uint8_t g_line_idx = 0;

static void process_json_line(const char *json, uint32_t now_ms)
{
    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
    float vx = 0.0f;
    float wz = 0.0f;

    int matched = sscanf(
        json,
        "{\"x\": %f, \"y\": %f, \"yaw\": %f, \"vx\": %f, \"wz\": %f}",
        &x, &y, &yaw, &vx, &wz
    );

    if (matched < 4) {
        matched = sscanf(
            json,
            "{\"x\":%f,\"y\":%f,\"yaw\":%f,\"vx\":%f,\"wz\":%f}",
            &x, &y, &yaw, &vx, &wz
        );
    }

    if (matched < 4) {
        return;
    }

    g_latest_pose.timestamp_ms = now_ms;
    g_latest_pose.robot_id = g_robot_id;
    g_latest_pose.status = POSE_STATUS_VALID | POSE_STATUS_INITIALISED;

    g_latest_pose.x_fp = FP_FROM_FLOAT(x);
    g_latest_pose.y_fp = FP_FROM_FLOAT(y);
    g_latest_pose.theta_fp = FP_FROM_FLOAT(yaw);
    g_latest_pose.v_fp = FP_FROM_FLOAT(vx);
    g_latest_pose.wz_fp = FP_FROM_FLOAT(wz);


    g_fresh_pose = 1;
}

static void uart_feed_byte(uint8_t byte, uint32_t now_ms)
{
    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        g_line_buf[g_line_idx] = '\0';

        if (g_line_idx > 0) {
            process_json_line(g_line_buf, now_ms);
        }

        g_line_idx = 0;
        return;
    }

    if (g_line_idx < PI_UART_MAX_PAYLOAD) {
        g_line_buf[g_line_idx++] = (char)byte;
    } else {
        // line too long, discard and wait for next newline
        g_line_idx = 0;
    }
}

// public API
void pi_uart_init(uint8_t robot_id) {
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

    // UART CONFIGURATIONS
    const eUSCI_UART_Config uartConfig = {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        6,
        8,
        0x20,
        EUSCI_A_UART_NO_PARITY,
        EUSCI_A_UART_LSB_FIRST,
        EUSCI_A_UART_ONE_STOP_BIT,
        EUSCI_A_UART_MODE,
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION // how were all these variables chosen
    };

    UART_initModule(EUSCI_A0_BASE, &uartConfig);
    UART_enableModule(EUSCI_A0_BASE);
}

// polling and processing
void pi_uart_poll(uint32_t now_ms) {
    while (UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)) {
        uint8_t byte = UART_receiveData(EUSCI_A0_BASE);
        uart_feed_byte(byte, now_ms);
    }
}

// receiving pose
uint8_t pi_uart_get_pose(RobotPoseMsg_t *out_pose) {
    if (!g_fresh_pose) {
        return 0;
    }
 
    *out_pose = g_latest_pose;
    g_fresh_pose = 0;
    return 1;
}

// sends pose
void pi_uart_send_pose(const RobotPoseMsg_t *pose) {
    if (!pose)  {
        return;
    }

    char line[128];

    float x   = FP_TO_FLOAT(pose->x_fp);
    float y   = FP_TO_FLOAT(pose->y_fp);
    float yaw = FP_TO_FLOAT(pose->theta_fp);
    float vx  = FP_TO_FLOAT(pose->v_fp);
    float wz  = FP_TO_FLOAT(pose->wz_fp);
    
    int n = snprintf(
        line,
        sizeof(line),
        "{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.3f,\"vx\":%.3f,\"wz\":%.3f}\n",
        x,
        y,
        yaw,
        vx,
        wz
    );

    if (n <= 0 || n >= sizeof(line)) {
        return;
    }

    int i;

    for (i = 0; i < n; i++) {
        while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
        EUSCI_A0->TXBUF = (uint8_t)line[i];
    }
}


void pi_uart_test_dummy_json(void)
{
    const char *test =
        "{\"x\": 1.234, \"y\": 2.345, \"yaw\": 0.500, \"vx\": 0.100, \"wz\": 0.050}\n";

    uint32_t fake_time_ms = 1234;
    uint16_t i;

    for (i = 0; test[i] != '\0'; i++) {
        uart_feed_byte((uint8_t)test[i], fake_time_ms);
    }
}